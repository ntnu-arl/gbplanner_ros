#!/usr/bin/env python3
"""Brief free-frustum producer/mapper check; run in a sourced ROS 2 workspace."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['voxblox', 'occupancy'], required=True)
    parser.add_argument('--output', type=Path, required=True, help='JSON result file')
    parser.add_argument('--timeout', type=float, default=12.0)
    parser.add_argument('--domain-id', type=int,
                        help='Override ROS_DOMAIN_ID; otherwise use its current value or a process-specific domain')
    args = parser.parse_args()
    if not 5 <= args.timeout <= 15:
        parser.error('--timeout must be between 5 and 15 seconds')
    domain = args.domain_id
    if domain is None:
        domain = int(os.environ.get('ROS_DOMAIN_ID', 100 + os.getpid() % 80))
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    # Import after selecting the DDS domain so the child launch and probe agree.
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from std_srvs.srv import Trigger
    from tf2_ros import TransformBroadcaster

    resolution, origin = .25, (-5., -5., -1.5)
    position = (.125, .125, .125)

    def index(point):
        return tuple(math.floor((point[i] - origin[i]) / resolution) for i in range(3))

    def records(message, include_status=False):
        offsets = {field.name: field.offset for field in message.fields}
        endian = '>' if message.is_bigendian else '<'
        for row in range(message.height):
            for col in range(message.width):
                start = row * message.row_step + col * message.point_step
                point = tuple(struct.unpack_from(endian + 'f', message.data, start + offsets[name])[0]
                              for name in ('x', 'y', 'z'))
                status = struct.unpack_from('b', message.data, start + offsets['occupancy'])[0] if include_status else None
                yield point, status

    class Probe(Node):
        def __init__(self):
            super().__init__('free_space_check')
            self.odom = self.create_publisher(Odometry, 'odometry', 10)
            self.tf = TransformBroadcaster(self)
            self.previous_stamp = None
            self.pose_messages = 0
            self.clouds = 0
            self.points = 0
            self.endpoints = set()
            self.cells = {}
            self.snapshots = 0
            self.max_occupied = 0
            self.map_stamp = (0, 0)
            self.cloud_stamp = (0, 0)
            self.create_timer(.05, self.publish_pose)
            self.create_subscription(PointCloud2, 'freespace_pointcloud', self.cloud, qos_profile_sensor_data)
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(PointCloud2, 'voxel_map', self.snapshot, durable)
            names = ['reset_map', 'free_space_status', 'mapper_status']
            if args.backend == 'voxblox':
                names.append('clear_robot_unknown_box')
            self.service_clients = {name: self.create_client(Trigger, name) for name in names}

        def publish_pose(self):
            stamp = self.get_clock().now().to_msg()
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = 'map', 'lidar'
            transform.header.stamp = stamp
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = position
            transform.transform.rotation.w = 1.
            self.tf.sendTransform(transform)
            if self.previous_stamp is not None:
                odom = Odometry()
                odom.header.frame_id, odom.child_frame_id = 'map', 'base_link'
                # The preceding TF sample is already in flight; it still has
                # the exact acquisition time and is only 50 ms old.
                odom.header.stamp = self.previous_stamp
                odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position
                odom.pose.pose.orientation.w = 1.
                self.odom.publish(odom)
                self.pose_messages += 1
            self.previous_stamp = stamp

        def cloud(self, message):
            self.clouds += 1
            self.points += message.width * message.height
            self.cloud_stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
            # The fixture has identity sensor rotation and fixed translation.
            self.endpoints.update(index(tuple(p[i] + position[i] for i in range(3)))
                                  for p, _ in records(message))

        def snapshot(self, message):
            self.cells = {index(point): status for point, status in records(message, True)}
            self.snapshots += 1
            self.max_occupied = max(self.max_occupied, sum(value == 100 for value in self.cells.values()))
            self.map_stamp = (message.header.stamp.sec, message.header.stamp.nanosec)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    log_path = args.output.with_suffix('.log')
    result = {'status': 'failed', 'backend': args.backend, 'domain_id': domain,
              'launch_log': str(log_path), 'checks': {}}
    process, probe = None, None
    started = time.monotonic()
    deadline = started + args.timeout

    def spin_until(predicate, label):
        while not predicate():
            if process.poll() is not None:
                raise RuntimeError(f'Launch exited with code {process.returncode}: {log_path}')
            if time.monotonic() >= deadline:
                raise TimeoutError(label)
            rclpy.spin_once(probe, timeout_sec=.05)

    def call(name):
        client = probe.service_clients[name]
        spin_until(client.service_is_ready, f'{name} did not become ready')
        future = client.call_async(Trigger.Request())
        spin_until(future.done, f'{name} did not respond')
        reply = future.result()
        try:
            message = json.loads(reply.message)
        except json.JSONDecodeError:
            message = reply.message
        return {'success': reply.success, 'message': message}

    try:
        rclpy.init(args=[])
        probe = Probe()
        with log_path.open('w') as log:
            process = subprocess.Popen(['ros2', 'launch', 'gbplanner_local', 'free_space.launch.py',
                                        f'backend:={args.backend}'], stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=dict(os.environ))
        spin_until(lambda: all(client.service_is_ready() for client in probe.service_clients.values()),
                   'Planner/mapper services did not become ready')
        # Both backends start with an unknown map; reset publishes that empty
        # snapshot so the planner can start augmentation without fake returns.
        result['reset'] = call('reset_map')
        if not result['reset']['success']:
            raise RuntimeError('Map reset failed')
        spin_until(lambda: probe.clouds >= 2 and any(value == 0 for value in probe.cells.values()),
                   'No synthetic cloud -> free map update')
        result['producer'] = call('free_space_status')
        result['mapper_before_box'] = call('mapper_status')
        if args.backend == 'voxblox':
            result['clear_robot_unknown_box'] = call('clear_robot_unknown_box')
        # Observe a few complete cycles in one short run, without triggering
        # exploration, changing seeds, or generating a persistent experiment.
        spin_until(lambda: time.monotonic() - started >= 5., 'Minimum observation interval')
        result['producer'] = call('free_space_status')
        result['mapper'] = call('mapper_status')
        producer = result['producer']['message']
        mapper = result['mapper']['message']
        free = mapper['freespace']
        endpoint_counts = {str(value): sum(probe.cells.get(key, -1) == value for key in probe.endpoints)
                           for value in (-1, 0, 100)}
        result['observed'] = {
            'odometry_messages': probe.pose_messages, 'clouds': probe.clouds, 'points': probe.points,
            'map_snapshots': probe.snapshots, 'known_cells': sum(v != -1 for v in probe.cells.values()),
            'free_cells': sum(v == 0 for v in probe.cells.values()), 'occupied_cells': sum(v == 100 for v in probe.cells.values()),
            'max_occupied_cells': probe.max_occupied, 'unique_endpoint_cells': len(probe.endpoints),
            'endpoint_unknown': endpoint_counts['-1'], 'endpoint_free': endpoint_counts['0'],
            'endpoint_occupied': endpoint_counts['100'], 'map_stamp': probe.map_stamp,
            'latest_cloud_stamp': probe.cloud_stamp,
        }
        result['checks'] = {
            'producer_published': result['producer']['success'] and producer['clouds'] >= 2 and producer['points'] > 0,
            'mapper_integrated': result['mapper']['success'] and free['integrated_clouds'] >= 1 and free['accepted_points'] > 0,
            'map_became_known': mapper['known_cells'] > 0 and result['observed']['free_cells'] > 0,
            'no_obstacle_returns_injected': mapper['integrated_scans'] == 0,
            'no_occupied_endpoints': len(probe.endpoints) > 0 and endpoint_counts['100'] == 0,
            'no_occupied_map_cells': probe.max_occupied == 0,
            'producer_no_failures': producer['failed_updates'] == 0,
        }
        if args.backend == 'voxblox':
            result['checks']['clear_box_completed_once'] = (
                result['clear_robot_unknown_box']['success'] and free['unknown_box_operations'] == 1)
        result['status'] = 'passed' if all(result['checks'].values()) else 'failed'
    except Exception as error:
        result['error'] = f'{type(error).__name__}: {error}'
    finally:
        result['elapsed_seconds'] = round(time.monotonic() - started, 3)
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=1.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=1.)
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        args.output.write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
    return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
