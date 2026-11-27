#!/usr/bin/env python3
"""Observe native Edgar mesh contacts, lidar returns and TSDF without motion."""

import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import struct
import subprocess
import sys
import time


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def cloud_metadata(message, topic):
    return {
        'topic': topic, 'frame_id': message.header.frame_id,
        'stamp': {'sec': message.header.stamp.sec, 'nanosec': message.header.stamp.nanosec},
        'width': message.width, 'height': message.height,
        'point_step': message.point_step, 'row_step': message.row_step,
        'is_bigendian': message.is_bigendian, 'is_dense': message.is_dense,
        'fields': [{'name': value.name, 'offset': value.offset,
                    'datatype': value.datatype, 'count': value.count} for value in message.fields],
        'data_bytes': len(message.data),
    }


def field_values(message, name):
    fields = [value for value in message.fields if value.name == name]
    require(len(fields) == 1, f'Missing or duplicate cloud field {name}')
    field = fields[0]
    formats = {1: 'b', 2: 'B', 3: 'h', 4: 'H', 5: 'i', 6: 'I', 7: 'f', 8: 'd'}
    require(field.count == 1 and field.datatype in formats, f'Unsupported field {name}')
    decoder = struct.Struct(('>' if message.is_bigendian else '<') + formats[field.datatype])
    require(field.offset + decoder.size <= message.point_step, f'Invalid field offset {name}')
    require(message.row_step >= message.width * message.point_step
            and len(message.data) == message.row_step * message.height, 'Invalid cloud dimensions')
    return [decoder.unpack_from(message.data, row*message.row_step + col*message.point_step + field.offset)[0]
            for row in range(message.height) for col in range(message.width)]


def stop_process_tree(process):
    """Signal the whole launch group even if its leader has already exited."""
    if process is None:
        return {'started': False}
    result = {'started': True, 'process_group': process.pid, 'forced_kill': False}
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=1.5)
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, 0)
    except ProcessLookupError:
        pass
    else:
        # The launch leader can exit before its child nodes. Always terminate
        # any remaining members, not merely the parent returned by Popen.
        result['forced_kill'] = True
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    try:
        result['launch_returncode'] = process.wait(timeout=.5)
    except subprocess.TimeoutExpired:
        result['launch_returncode'] = None
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))
    started = time.monotonic()
    deadline = started + 17.0  # Reserve three seconds of the 20 s budget for cleanup.

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    process, node, log = None, None, None
    log_path = args.output.with_suffix('.log')
    summary = {
        'status': 'failed', 'domain': os.environ['ROS_DOMAIN_ID'], 'wall_budget_s': 20,
        'scope': 'Original Edgar collision mesh → Bullet natural settling → native lidar PointCloud2 → Voxblox TSDF → accepted planner map',
        'motion_requested': False, 'exploration_requested': False,
        'action_or_pci_requests': 0, 'command_publisher_created': False,
        'checks': {},
    }

    class Probe(Node):
        def __init__(self):
            super().__init__('edgar_mapping_probe')
            self.smb = {}
            self.states = []
            self.odom = deque(maxlen=250)
            self.cloud = None
            self.raw_map = None
            self.accepted_map = None
            self.cloud_count = self.raw_map_count = self.accepted_map_count = 0
            self.command_count = self.nonzero_commands = 0
            self.map_updates = []
            self.cloud_stamps, self.raw_stamps, self.accepted_stamps = set(), set(), set()
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Odometry, 'odometry', self.on_odom, qos_profile_sensor_data)
            self.create_subscription(PointCloud2, 'points', self.on_cloud, qos_profile_sensor_data)
            self.create_subscription(String, 'smb_status', self.on_smb, 10)
            self.create_subscription(Twist, 'cmd_vel', self.on_command, 10)
            self.create_subscription(PointCloud2, 'voxel_map', self.on_raw_map, durable)
            self.create_subscription(PointCloud2, 'accepted_voxel_map', self.on_accepted_map, durable)
            self.create_subscription(String, 'map_updates', self.on_map_update, durable)
            self.state_clients = {name: self.create_client(Trigger, name)
                             for name in ('smb_state', 'mapper_status', 'map_status')}

        def on_odom(self, message):
            require(message.header.frame_id == 'map' and message.child_frame_id == 'base_link',
                    'Unexpected SMB odometry frames')
            p, q = message.pose.pose.position, message.pose.pose.orientation
            require(all(math.isfinite(value) for value in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)),
                    'Non-finite odometry')
            self.odom.append(message)

        def on_cloud(self, message):
            require(message.header.frame_id == 'smb_lidar', 'Lidar is not in its native sensor frame')
            self.cloud = message
            self.cloud_count += 1
            self.cloud_stamps.add(stamp_ns(message))

        def on_raw_map(self, message):
            require(message.header.frame_id == 'map', 'Unexpected mapper frame')
            self.raw_map = message
            self.raw_map_count += 1
            self.raw_stamps.add(stamp_ns(message))

        def on_accepted_map(self, message):
            require(message.header.frame_id == 'map', 'Unexpected accepted map frame')
            self.accepted_map = message
            self.accepted_map_count += 1
            self.accepted_stamps.add(stamp_ns(message))

        def on_map_update(self, message):
            self.map_updates.append({'elapsed_s': time.monotonic()-started, **json.loads(message.data)})

        def on_command(self, message):
            values = (message.linear.x, message.linear.y, message.linear.z,
                      message.angular.x, message.angular.y, message.angular.z)
            require(all(math.isfinite(value) for value in values), 'Non-finite velocity command')
            self.command_count += 1
            if any(abs(value) > 1.e-10 for value in values):
                self.nonzero_commands += 1
                raise AssertionError('A nonzero command appeared during the stationary mapping run')

        def on_smb(self, message):
            state = json.loads(message.data)
            require(state['physics'] == 'Bullet', 'Unexpected physics backend')
            require(state['scene'] == {'infinite_ground': False, 'room_boxes': False,
                                       'mesh_vertices': 13770, 'mesh_triangles': 22557},
                    f'Wrong collision scene: {state.get("scene")}')
            require(state['chassis_contacts'] == 0 and state['chassis_contact_steps'] == 0,
                    'The chassis touched scene geometry')
            require(abs(state['command_linear']) < 1.e-10 and abs(state['command_angular']) < 1.e-10,
                    'Nonzero motor command during stationary observation')
            self.smb = state
            self.states.append({
                'elapsed_s': time.monotonic()-started, 'simulated_seconds': state['simulated_seconds'],
                'position': state['position'], 'speed_mps': state['speed_mps'],
                'wheel_contacts': state['wheel_contacts'], 'scans': state['scans'],
            })

        def until(self, predicate, label):
            while not predicate():
                if process is not None and process.poll() is not None:
                    raise RuntimeError(f'Launch exited {process.returncode}; see {log_path}')
                if time.monotonic() >= deadline:
                    raise TimeoutError(f'20 s run budget: {label}')
                rclpy.spin_once(self, timeout_sec=.01)

        def states_from_services(self):
            self.until(lambda: all(client.service_is_ready() for client in self.state_clients.values()),
                       'service discovery')
            futures = {name: client.call_async(Trigger.Request()) for name, client in self.state_clients.items()}
            self.until(lambda: all(future.done() for future in futures.values()), 'state service responses')
            result = {}
            for name, future in futures.items():
                response = future.result()
                require(response.success, f'{name}: {response.message}')
                result[name] = json.loads(response.message)
            return result

    try:
        rclpy.init(args=[])
        node = Probe()
        log = log_path.open('w')
        command = ['ros2', 'launch', 'gbplanner_smb', 'edgar_mine.launch.py',
                   'control:=false', 'description:=false']
        summary['launch_command'] = command
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=dict(os.environ))
        node.until(lambda: bool(node.smb) and bool(node.odom) and node.cloud_count >= 2
                   and node.raw_map_count >= 2 and node.accepted_map_count >= 2
                   and bool(node.map_updates), 'initial native scan and accepted TSDF map')
        initial = node.states_from_services()
        observation_started = time.monotonic()
        require(observation_started + 6.0 < deadline, 'Startup left less than six seconds for observation')
        node.until(lambda: time.monotonic() >= observation_started + 6.0, 'six-second observation')
        final = node.states_from_services()
        recent = [row for row in node.states if row['elapsed_s'] >= time.monotonic()-started-1.0]
        require(len(recent) >= 10 and recent[-1]['elapsed_s']-recent[0]['elapsed_s'] >= .7,
                'Insufficient final stable-contact samples')
        require(all(row['wheel_contacts'] == 4 for row in recent), 'Four-wheel contact is not stable')
        require(all(.37 <= row['position'][2] <= .43 for row in recent),
                'Settled base height does not match the original mine floor')
        require(max(row['speed_mps'] for row in recent) < .03, 'Vehicle did not settle')
        require(max(math.hypot(row['position'][0], row['position'][1]) for row in recent) < .05,
                'Unexpected lateral movement without a command')
        require(final['smb_state']['scans'] >= initial['smb_state']['scans'] + 5, 'Native scans did not advance')
        require(final['mapper_status']['integrated_scans'] >= initial['mapper_status']['integrated_scans'] + 5,
                'TSDF integration did not advance')
        require(final['map_status']['map_revision'] >= initial['map_status']['map_revision'] + 5,
                'Planner map revisions did not advance')
        require(final['mapper_status']['backend'] == 'voxblox_tsdf'
                and final['mapper_status']['known_cells'] > 0
                and final['mapper_status']['allocated_blocks'] > 0, 'No populated native TSDF')
        require(final['map_status']['tsdf_fields'] and final['map_status']['tsdf_cells'] > 0,
                'Planner did not accept the TSDF fields')
        require(final['mapper_status']['freespace']['unknown_box_operations'] == 0,
                'The run cleared unknown space instead of observing it')
        require(min(len(node.cloud_stamps), len(node.raw_stamps), len(node.accepted_stamps)) >= 5,
                'Distinct native scan/map publications are missing')
        matching_pipeline_stamps = node.cloud_stamps & node.raw_stamps & node.accepted_stamps
        require(len(matching_pipeline_stamps) >= 5,
                'Fewer than five acquisition timestamps traverse scan, TSDF and accepted map topics')
        cloud = node.cloud
        coordinates = [field_values(cloud, axis) for axis in ('x', 'y', 'z')]
        require(coordinates[0] and all(math.isfinite(value) for axis in coordinates for value in axis),
                'Native scan is empty or non-finite')
        matching_odom = min(node.odom, key=lambda value: abs(stamp_ns(value)-stamp_ns(cloud)))
        delta_ns = stamp_ns(matching_odom)-stamp_ns(cloud)
        require(abs(delta_ns) <= 50_000_000, 'No nearby odometry for the saved scan')
        map_cloud = node.raw_map
        distances = field_values(map_cloud, 'tsdf_distance')
        weights = field_values(map_cloud, 'tsdf_weight')
        require(weights and all(math.isfinite(value) for value in distances + weights)
                and max(weights) > 0, 'Published TSDF distances/weights are invalid')
        scan_data_path = args.output.with_name(args.output.stem + '_points.bin')
        scan_info_path = args.output.with_name(args.output.stem + '_frame.json')
        scan_data = bytes(cloud.data)
        scan_data_path.write_bytes(scan_data)
        scan_info = cloud_metadata(cloud, 'points')
        scan_info.update(data_file=str(scan_data_path), data_sha256=hashlib.sha256(scan_data).hexdigest(),
                         associated_odometry=message_to_ordereddict(matching_odom),
                         odometry_minus_cloud_stamp_ns=delta_ns,
                         association='Nearest observed odometry timestamp; original ROS messages retained')
        scan_info_path.write_text(json.dumps(scan_info, indent=2, allow_nan=False)+'\n')
        summary.update({
            'status': 'passed', 'observation_s': time.monotonic()-observation_started,
            'checks': {
                'original_mesh_without_room_or_plane': True,
                'no_chassis_contact': True, 'stable_four_wheel_contact': True,
                'floor_height_from_native_odometry': True,
                'native_scans_and_tsdf_revisions_advance': True,
                'planner_accepts_tsdf_fields': True, 'no_nonzero_command_or_unknown_box_clear': True,
                'native_cloud_and_odometry_saved': True,
            },
            'initial_counters': {
                'scans': initial['smb_state']['scans'],
                'integrated_scans': initial['mapper_status']['integrated_scans'],
                'map_revision': initial['map_status']['map_revision'],
                'known_cells': initial['mapper_status']['known_cells'],
            },
            'final_smb': final['smb_state'], 'final_mapper': final['mapper_status'],
            'final_planner_map': final['map_status'],
            'stable_contact_window': {
                'samples': len(recent), 'duration_s': recent[-1]['elapsed_s']-recent[0]['elapsed_s'],
                'base_z_min': min(row['position'][2] for row in recent),
                'base_z_max': max(row['position'][2] for row in recent),
                'maximum_speed_mps': max(row['speed_mps'] for row in recent),
            },
            'publications': {'points': node.cloud_count, 'voxel_map': node.raw_map_count,
                             'accepted_voxel_map': node.accepted_map_count,
                             'matching_scan_tsdf_accepted_stamps': len(matching_pipeline_stamps),
                             'map_updates': len(node.map_updates), 'cmd_vel': node.command_count,
                             'nonzero_cmd_vel': node.nonzero_commands},
            'final_received_map': {**cloud_metadata(map_cloud, 'voxel_map'),
                                   'distance_min': min(distances), 'distance_max': max(distances),
                                   'weight_min': min(weights), 'weight_max': max(weights)},
            'saved_frame': {'metadata': str(scan_info_path), 'data': str(scan_data_path),
                            'points': cloud.width * cloud.height},
        })
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
        if node is not None:
            summary['last_smb'] = node.smb
            summary['last_map_update'] = node.map_updates[-1] if node.map_updates else None
            summary['received_counts'] = {'points': node.cloud_count, 'voxel_map': node.raw_map_count,
                                          'accepted_voxel_map': node.accepted_map_count}
    finally:
        summary['cleanup'] = stop_process_tree(process)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if log is not None:
            log.close()
            failures = re.findall(r'\[([^\]]+)\]: process has died \[pid \d+, exit code (-?\d+)',
                                  log_path.read_text())
            summary['node_exit_failures'] = [
                {'node': name, 'exit_code': int(code)} for name, code in failures]
            if failures:
                summary['functional_checks_passed'] = summary['status'] == 'passed'
                summary['status'] = 'failed'
                summary['shutdown_error'] = 'One or more launched nodes exited abnormally; see log'
        summary['elapsed_s'] = time.monotonic()-started
        summary['log'] = str(log_path)
        if summary['elapsed_s'] > 20.0:
            summary['status'] = 'failed'
            summary['budget_error'] = 'Exceeded the 20 s total wall budget'
        args.output.write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
        print(json.dumps({'status': summary['status'], 'output': str(args.output),
                          'elapsed_s': summary['elapsed_s'], 'error': summary.get('error')}, indent=2))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    sys.exit(main())
