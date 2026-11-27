#!/usr/bin/env python3
"""Observe Pittsburgh support, native poses, dual lidar and TSDF without motion.

No command publisher, planner request or map mutation is created. Contact is
event-driven: four wheels need real collision pairs/positions in a >=1 s stable
window. A configured, connected but silent base sensor proves only that no
chassis contact was recorded, never absolute absence of collision.
"""
import argparse
import ast
from collections import deque
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import time
import traceback
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
from check_edgar_mapping import cloud_metadata, field_values, stamp_ns
from check_fortress_mapping import compose_native_pose, pose_record, wrap
from check_fortress_smb_sensors import SENSORS, shutdown_launch
from check_fortress_autonomy import shutdown_log_events


PARTS = ('base', 'lf', 'rf', 'lh', 'rh')
WHEELS = {'LF_wheel_joint', 'RF_wheel_joint', 'LH_wheel_joint', 'RH_wheel_joint'}


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def json_objects(text):
    """Transport11 can return several messages even with `ign topic -n 1`."""
    decoder, records = json.JSONDecoder(), []
    remaining = text.lstrip()
    while remaining:
        record, end = decoder.raw_decode(remaining)
        records.append(record)
        remaining = remaining[end:].lstrip()
    return records


def scene_model_pose(text, name):
    """Read only direct model/pose fields from Scene protobuf text.

    Braces inside quoted strings never affect nesting. Unknown blocks are
    skipped intact; the original response remains the authoritative artifact.
    This accepts the brace-delimited text printed by `ign service`, not every
    protobuf text syntax. Missing blocks are never supplied from the SDF.
    """
    def fields(block):
        tokens = list(re.finditer(r'"(?:\\.|[^"\\])*"|[{}:]|[^\s{}:]+', block))
        scalars, blocks, index = {}, {}, 0
        while index < len(tokens):
            key = tokens[index].group()
            require(re.fullmatch(r'[A-Za-z_][A-Za-z_0-9]*', key), 'Unexpected native scene field')
            index += 1
            if index < len(tokens) and tokens[index].group() == ':':
                index += 1
            require(index < len(tokens), 'Missing native scene value')
            token = tokens[index]
            if token.group() == '{':
                start, depth = token.end(), 1
                index += 1
                while index < len(tokens) and depth:
                    value = tokens[index].group()
                    depth += (value == '{') - (value == '}')
                    index += 1
                require(depth == 0, 'Unbalanced native scene block')
                blocks.setdefault(key, []).append(block[start:tokens[index-1].start()])
            else:
                require(token.group() not in ('}', ':'), 'Invalid native scene scalar')
                scalars.setdefault(key, []).append(token.group())
                index += 1
        return scalars, blocks

    def scalar(values, key, default=None):
        candidates = values.get(key, [])
        require(len(candidates) <= 1, 'Duplicate native scene scalar ' + key)
        return candidates[0] if candidates else default

    def block(values, key):
        candidates = values.get(key, [])
        require(len(candidates) == 1, 'Missing/duplicate native scene block ' + key)
        return fields(candidates[0])

    _, top_blocks = fields(text)
    matches = []
    for raw in top_blocks.get('model', []):
        values, blocks = fields(raw)
        raw_name = scalar(values, 'name')
        if raw_name is not None and ast.literal_eval(raw_name) == name:
            matches.append((values, blocks))
    require(len(matches) == 1, f'Expected one native scene model {name}, got {len(matches)}')
    values, model_blocks = matches[0]
    _, pose_blocks = block(model_blocks, 'pose')
    p, _ = block(pose_blocks, 'position')
    q, _ = block(pose_blocks, 'orientation')
    # Omitted scalar numeric fields have protobuf's zero default. The three
    # containing blocks above must nevertheless be present in the response.
    position = [float(scalar(p, key, '0')) for key in ('x', 'y', 'z')]
    orientation = [float(scalar(q, key, '0')) for key in ('x', 'y', 'z', 'w')]
    require(all(math.isfinite(x) for x in position + orientation) and
            abs(math.hypot(*orientation) - 1.) < 1.e-5, 'Invalid native static pose')
    return {'name': name, 'id': int(scalar(values, 'id', '0')),
            'position': position, 'quaternion': orientation}


def mine_support_pair(pair, part):
    """The other body must be the mine, not another SMB collision."""
    if len(pair) != 2:
        return False
    # fortress_model.CONTACT_LINKS retains LF_WHEEL_LINK etc.; sdformat
    # appends _collision to the explicit URDF collision name.
    expected = part + '_wheel_link_collision'
    names = [name.lower().split('::') for name in pair]
    return any('pittsburgh_mine' in names[i] and 'smb' in names[1-i] and
               expected in names[1-i] for i in (0, 1))


def stable_window(rows, earliest_ns, seconds=1.):
    """Mapping probe tolerances, with an actual >=1 s source-time span."""
    if not rows:
        return None
    end = rows[-1]['stamp_ns']
    candidates = [r for r in rows if r['stamp_ns'] >= earliest_ns and
                  end - r['stamp_ns'] <= (seconds + .1) * 1.e9]
    if len(candidates) < 5 or end - candidates[0]['stamp_ns'] < seconds * 1.e9:
        return None
    last = candidates[-1]
    metrics = {
        'z_deviation_m': max(abs(r['pose'][2] - last['pose'][2]) for r in candidates),
        'xy_deviation_m': max(math.dist(r['pose'][:2], last['pose'][:2]) for r in candidates),
        'yaw_deviation_rad': max(abs(wrap(r['pose'][3] - last['pose'][3])) for r in candidates),
        'linear_speed_mps': max(math.hypot(*r['linear_velocity']) for r in candidates),
        'yaw_speed_rps': max(abs(r['angular_velocity'][2]) for r in candidates),
    }
    limits = dict(zip(metrics, (.01, .025, .04, .04, .05)))
    if not all(metrics[key] < limits[key] for key in metrics):
        return None
    return {'start_ns': candidates[0]['stamp_ns'], 'end_ns': end,
            'duration_s': (end - candidates[0]['stamp_ns']) * 1.e-9,
            'samples': len(candidates), 'metrics': metrics, 'strict_upper_limits': limits,
            'absolute_ground_z_assumption': None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--wall-budget', type=float, default=180.)
    parser.add_argument('--renderer-prefix', type=Path)
    parser.add_argument('--gpu-rays-min-cubemap-size', type=int, choices=(128, 256, 512, 1024), default=512)
    args = parser.parse_args()
    if not math.isfinite(args.wall_budget) or not 40. <= args.wall_budget <= 180.:
        parser.error('--wall-budget must be between 40 and 180 seconds')
    if args.renderer_prefix is not None:
        args.renderer_prefix = args.renderer_prefix.expanduser().resolve()
        if not args.renderer_prefix.is_dir():
            parser.error('--renderer-prefix must be an existing install directory')
    output = args.output.expanduser().resolve()
    evidence = output.with_name(output.stem + '_evidence')
    log_path = output.with_suffix('.log')
    require(len({output, log_path, evidence}) == 3 and
            not any(p.exists() for p in (output, log_path, evidence)), 'Use a fresh output name')
    output.parent.mkdir(parents=True, exist_ok=True)
    evidence.mkdir()
    # Reserve every artifact exclusively. A failed run remains a record.
    result_file, log = output.open('x'), log_path.open('x')
    history_file = evidence / 'history.jsonl'
    history = history_file.open('x')
    root = Path(__file__).resolve().parents[1]
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))
    native_env = dict(os.environ, IGN_PARTITION='gbplanner_fortress_' + os.environ['ROS_DOMAIN_ID'])
    started = time.monotonic()
    deadline = started + args.wall_budget - 15.
    result = {'status': 'failed', 'scope': __doc__, 'checks': {}, 'wall_budget_s': args.wall_budget,
              'domain': os.environ['ROS_DOMAIN_ID'], 'partition': native_env['IGN_PARTITION'],
              'motion_requested': False, 'command_publishers_created': 0, 'map_mutation_requests': 0,
              'planning_requested': False, 'action_or_pci_requests': 0,
              'static_pose_verified': False, 'odometry_world_pose_runtime_verified': False,
              'renderer': {'selection': 'project' if args.renderer_prefix else 'inherit',
                           'prefix': str(args.renderer_prefix) if args.renderer_prefix else None},
              'raw_stream_format': 'gzip of repeated uint32 little-endian CDR length then ROS2 CDR bytes; '
                                   'history.jsonl records topic, message type, index and receive/source time',
              'history_file': str(history_file), 'evidence_directory': str(evidence)}
    process = node = None
    initialized = False
    streams = {}
    source_paths, installed_paths = {}, {}
    try:
        import numpy as np
        import rclpy
        from ament_index_python.packages import get_package_share_directory, get_package_prefix
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rcl_interfaces.srv import GetParameters
        from rclpy.node import Node
        from rclpy.parameter import Parameter, parameter_value_to_python
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from rclpy.serialization import serialize_message
        from rclpy.time import Time
        from ros_gz_interfaces.msg import Contacts
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState, PointCloud2
        from std_srvs.srv import Trigger
        from tf2_ros import Buffer, TransformListener

        relative_sources = [
            'src/gbplanner_gazebo/launch/pittsburgh_fortress.launch.py',
            'src/gbplanner_gazebo/launch/edgar_fortress.launch.py',
            'src/gbplanner_gazebo/worlds/pittsburgh_fortress.sdf',
            'src/gbplanner_gazebo/config/pittsburgh_exploration.yaml',
            'src/gbplanner_gazebo/config/controllers.yaml', 'src/gbplanner_gazebo/config/smb_imu.yaml',
            'src/gbplanner_gazebo/scripts/fortress_model.py',
            'src/gbplanner_gazebo/scripts/fortress_smb_sensors.py',
            'src/gbplanner_gazebo/scripts/fortress_renderer.py',
            'src/gbplanner_gazebo/scripts/fortress_state_node.py',
            'src/gbplanner_gazebo/src/managed_bridge_node.cpp',
            'src/gbplanner_gazebo/src/rmf_imu_adapter_node.cpp',
            'src/gbplanner_smb/urdf/smb.urdf', 'src/gbplanner_smb/config/upstream_sensors.yaml',
            'src/gbplanner_smb/config/upstream_planning.yaml',
            'src/planner_gazebo_sim/models/pittsburgh_mine/model.sdf',
            'src/planner_gazebo_sim/models/pittsburgh_mine/pittsburgh_mine.dae',
            'src/gbplanner_voxblox/src/voxblox_mapper_node.cpp',
            'tools/check_pittsburgh_scene.py', 'tools/check_edgar_mapping.py',
            'tools/check_fortress_mapping.py', 'tools/check_fortress_smb_sensors.py',
            'tools/check_fortress_autonomy.py']
        source_paths = {s: root / s for s in relative_sources}
        result['source_sha256_at_start'] = {name: digest(path) for name, path in source_paths.items()}
        # Verify the installed entry really selects the resources being tested.
        shares = {pkg: Path(get_package_share_directory(pkg)) for pkg in
                  ('gbplanner_gazebo', 'gbplanner_smb', 'planner_gazebo_sim')}
        for relative in relative_sources:
            parts = Path(relative).parts
            if parts[0] != 'src' or parts[1] not in shares or parts[2] in ('src', 'scripts'):
                continue
            installed = shares[parts[1]].joinpath(*parts[2:])
            require(installed.is_file() and digest(installed) == result['source_sha256_at_start'][relative],
                    'Installed scene resource differs: ' + relative)
            installed_paths[str(installed)] = installed
        for executable in ('managed_bridge_node', 'fortress_model.py', 'fortress_smb_sensors.py',
                           'fortress_renderer.py', 'fortress_state_node.py', 'rmf_imu_adapter_node'):
            path = Path(get_package_prefix('gbplanner_gazebo')) / 'lib/gbplanner_gazebo' / executable
            installed_paths[str(path)] = path
        mapper_binary = Path(get_package_prefix('gbplanner_voxblox')) / 'lib/gbplanner_voxblox/voxblox_mapper_node'
        installed_paths[str(mapper_binary)] = mapper_binary
        result['installed_sha256_at_start'] = {name: digest(path) for name, path in installed_paths.items()}
        result['checks']['installed_scene_resources_match_source'] = True

        def save_message(name, message):
            path = evidence / (name + '.cdr')
            with path.open('xb') as target:
                target.write(serialize_message(message))
            return {'file': str(path), 'sha256': digest(path), 'bytes': path.stat().st_size}

        def describe_cloud(message, topic):
            require(message.width * message.height > 0, 'Empty cloud: ' + topic)
            xyz = np.column_stack([field_values(message, field) for field in ('x', 'y', 'z')])
            finite = xyz[np.isfinite(xyz).all(axis=1)]
            description = {**cloud_metadata(message, topic), 'finite_points': len(finite),
                           'data_sha256': hashlib.sha256(message.data).hexdigest()}
            if len(finite):
                eigenvalues = np.linalg.eigvalsh(np.cov(finite.T)) if len(finite) > 3 else np.zeros(3)
                description.update(xyz_min=finite.min(axis=0).tolist(), xyz_max=finite.max(axis=0).tolist(),
                                   covariance_eigenvalues_m2=eigenvalues.tolist(),
                                   z_span_m=float(np.ptp(finite[:, 2])),
                                   nonplanar_returns=bool(eigenvalues[0] > 1.e-5 and np.ptp(finite[:, 2]) > .05))
            return description

        class Probe(Node):
            def __init__(self):
                super().__init__('pittsburgh_scene_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
                self.clock_ns = 0
                self.ready_ns = None
                self.recent = deque(maxlen=1000)
                self.raw_recent = deque(maxlen=1000)
                self.contact_recent = {part: deque(maxlen=2500) for part in PARTS}
                self.counts = {}; self.contact_nonempty = {part: 0 for part in PARTS}
                self.latest = {}; self.joints = None; self.nonzero_commands = []
                self.cache = {name: {'raw': {}, 'adapted': {}} for name in SENSORS}
                self.pairs = {}
                self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
                self.mapper = self.create_client(Trigger, 'mapper_status')
                self.robot_parameters = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
                self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
                for topic in ('odometry', 'gazebo/odometry_raw'):
                    self.create_subscription(Odometry, topic, lambda m, t=topic: self.odom(m, t), qos_profile_sensor_data)
                self.create_subscription(JointState, 'joint_states', self.joint, qos_profile_sensor_data)
                for part in PARTS:
                    self.create_subscription(Contacts, '/smb/contact/' + part,
                                             lambda m, p=part: self.contact(m, p), qos_profile_sensor_data)
                for name, spec in SENSORS.items():
                    for kind in ('raw', 'adapted'):
                        self.create_subscription(PointCloud2, spec[kind],
                                                 lambda m, n=name, k=kind: self.scan(m, n, k), qos_profile_sensor_data)
                durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.create_subscription(PointCloud2, 'voxel_map', self.map_cloud, durable)
                self.create_subscription(Twist, 'cmd_vel', self.command, 10)

            def timed(self, data):
                return {'wall_s': time.monotonic() - started, 'received_ros_ns': self.clock_ns, **data}

            def record(self, topic, message, data):
                index = self.counts.get(topic, 0)
                self.counts[topic] = index + 1
                if topic not in streams:
                    path = evidence / (topic.strip('/').replace('/', '_') + '.cdr.gz')
                    streams[topic] = {'file': str(path), 'handle': gzip.open(path, 'xb'), 'messages': 0}
                raw = serialize_message(message)
                streams[topic]['handle'].write(struct.pack('<I', len(raw)) + raw)
                streams[topic]['messages'] += 1
                row = self.timed({'topic': topic, 'message_type': message.__class__.__module__ + '.' +
                                 message.__class__.__name__, 'raw_index': index,
                                 'stamp_ns': stamp_ns(message), **data})
                history.write(json.dumps(row, allow_nan=False) + '\n')
                return row

            def clock(self, message):
                value = message.clock.sec * 1000000000 + message.clock.nanosec
                require(value >= self.clock_ns, 'Simulation clock regressed')
                self.clock_ns = value

            def odom(self, message, topic):
                q, v = message.pose.pose.orientation, message.twist.twist
                data = {'pose': pose_record(message), 'quaternion': [q.x, q.y, q.z, q.w],
                        'linear_velocity': [v.linear.x, v.linear.y, v.linear.z],
                        'angular_velocity': [v.angular.x, v.angular.y, v.angular.z],
                        'frame_id': message.header.frame_id, 'child_frame_id': message.child_frame_id}
                require(all(math.isfinite(x) for key in ('pose', 'quaternion', 'linear_velocity', 'angular_velocity')
                            for x in data[key]), 'Nonfinite odometry')
                row = self.record(topic, message, data)
                target = self.recent if topic == 'odometry' else self.raw_recent
                require(not target or row['stamp_ns'] >= target[-1]['stamp_ns'], 'Odometry timestamp regressed')
                target.append(row)

            def joint(self, message):
                require(all(math.isfinite(v) for v in [*message.position, *message.velocity]),
                        'Nonfinite physical joint position or velocity')
                self.joints = message
                # The velocity-controlled hardware reports unavailable effort
                # as NaN. Keep its original CDR; express unobserved effort
                # explicitly in JSON without treating it as measured zero.
                self.record('joint_states', message, {'name': list(message.name), 'position': list(message.position),
                            'velocity': list(message.velocity),
                            'effort': [v if math.isfinite(v) else None for v in message.effort],
                            'nonfinite_effort': [{'index': i, 'value': repr(v)}
                                                 for i, v in enumerate(message.effort) if not math.isfinite(v)]})

            def contact(self, message, part):
                pairs = [[c.collision1.name, c.collision2.name] for c in message.contacts]
                positions = [[p.x, p.y, p.z] for c in message.contacts for p in c.positions]
                supporting = [{'pair': [c.collision1.name, c.collision2.name],
                               'positions': [[p.x, p.y, p.z] for p in c.positions]}
                              for c in message.contacts
                              if mine_support_pair([c.collision1.name, c.collision2.name], part) and c.positions and
                              all(math.isfinite(v) for p in c.positions for v in (p.x, p.y, p.z))]
                row = self.record('/smb/contact/' + part, message,
                                  {'part': part, 'pairs': pairs, 'positions': positions,
                                   'mine_support_contacts': supporting,
                                   'normals': [[n.x, n.y, n.z] for c in message.contacts for n in c.normals],
                                   'depths': [d for c in message.contacts for d in c.depths]})
                self.contact_recent[part].append(row)
                if message.contacts:
                    self.contact_nonempty[part] += 1
                require(part != 'base' or not message.contacts, 'A nonempty native chassis contact was recorded')

            def scan(self, message, name, kind):
                topic = SENSORS[name][kind]
                self.latest[topic] = message
                self.counts[topic] = self.counts.get(topic, 0) + 1
                history.write(json.dumps(self.timed({'topic': topic, **cloud_metadata(message, topic)})) + '\n')
                cache = self.cache[name]
                cache[kind][stamp_ns(message)] = message
                other = 'adapted' if kind == 'raw' else 'raw'
                if stamp_ns(message) in cache[other]:
                    self.pairs[name] = {k: cache[k][stamp_ns(message)] for k in ('raw', 'adapted')}
                if len(cache[kind]) > 10:
                    cache[kind].pop(next(iter(cache[kind])))

            def map_cloud(self, message):
                self.latest['voxel_map'] = message
                self.counts['voxel_map'] = self.counts.get('voxel_map', 0) + 1
                history.write(json.dumps(self.timed({'topic': 'voxel_map', **cloud_metadata(message, 'voxel_map')})) + '\n')

            def command(self, message):
                values = [message.linear.x, message.linear.y, message.linear.z,
                          message.angular.x, message.angular.y, message.angular.z]
                require(all(math.isfinite(v) for v in values), 'Nonfinite velocity command')
                if max(map(abs, values)) > 1.e-9:
                    self.nonzero_commands.append(self.timed({'velocity': values}))
                    raise AssertionError('Unexpected nonzero cmd_vel during passive observation')

            def until(self, predicate, label, local_deadline=None):
                end = min(deadline, local_deadline) if local_deadline else deadline
                while not predicate():
                    require(process is None or process.poll() is None, 'Launch exited during ' + label)
                    if time.monotonic() >= end:
                        raise TimeoutError('Deadline during ' + label)
                    rclpy.spin_once(self, timeout_sec=.01)

            def call(self, client, request, label):
                self.until(client.service_is_ready, label + ' discovery')
                future = client.call_async(request)
                self.until(future.done, label + ' response')
                response = future.result()
                require(response is not None, 'No response from ' + label)
                return response

            def support_window(self):
                window = stable_window(self.recent, self.ready_ns or self.clock_ns)
                if window is None:
                    return None
                evidence_by_part = {}
                for part in PARTS:
                    rows = [r for r in self.contact_recent[part]
                            if window['start_ns'] <= r['stamp_ns'] <= window['end_ns'] and
                            r['received_ros_ns'] >= self.ready_ns]
                    nonempty = [r for r in rows if r['mine_support_contacts']]
                    if part != 'base' and not nonempty:
                        return None
                    evidence_by_part[part] = {'messages': len(rows), 'nonempty_pairs_and_positions': len(nonempty),
                        'first_source_ns': rows[0]['stamp_ns'] if rows else None,
                        'last_source_ns': rows[-1]['stamp_ns'] if rows else None,
                        'sample': nonempty[0] if nonempty else None}
                return {**window, 'contacts': evidence_by_part}

        rclpy.init(args=[]); initialized = True; node = Probe()
        command = ['ros2', 'launch', 'gbplanner_gazebo', 'pittsburgh_fortress.launch.py',
                   'planning:=false', 'mapping:=true', 'contacts:=true', 'sensor_profile:=upstream']
        if args.renderer_prefix:
            command += ['renderer:=project', 'renderer_prefix:=' + str(args.renderer_prefix),
                        'gpu_rays_min_cubemap_size:=' + str(args.gpu_rays_min_cubemap_size)]
        result['launch_command'] = command
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   env=dict(os.environ), start_new_session=True)
        node.until(lambda: node.clock_ns > 0 and node.recent and node.raw_recent and node.joints is not None and
                   set(node.pairs) == set(SENSORS) and 'voxel_map' in node.latest and
                   node.latest['voxel_map'].width * node.latest['voxel_map'].height > 0,
                   'native odometry/joints/dual-lidar/initial voxel map')
        node.ready_ns = node.clock_ns
        result['ready_after_all_inputs_ns'] = node.ready_ns
        require(WHEELS <= set(node.joints.name), 'Missing physical wheel joint states')
        require(node.recent[-1]['frame_id'] == 'map' and node.recent[-1]['child_frame_id'] == 'base_link',
                'Adapted odometry frame contract differs')
        node.until(lambda: node.support_window() is not None, 'stable physical support window')
        result['stable_support_window'] = node.support_window()
        result['checks']['stable_four_wheel_contact_window'] = True

        response = node.call(node.robot_parameters, GetParameters.Request(names=['robot_description']), 'robot_description')
        require(len(response.values) == 1, 'Missing live robot description')
        description = parameter_value_to_python(response.values[0])
        with (evidence / 'robot_description.urdf').open('x') as f:
            f.write(description)
        robot = ET.fromstring(description)
        result['contact_sources'] = {}
        for part in PARTS:
            topic = '/smb/contact/' + part
            sensors = [s for s in robot.findall('.//sensor')
                       if s.get('type') == 'contact' and s.findtext('contact/topic') == topic]
            require(len(sensors) == 1 and sensors[0].findtext('contact/collision'), 'Missing actual contact sensor ' + part)
            extensions = [g for g in robot.findall('gazebo') if sensors[0] in list(g)]
            require(len(extensions) == 1, 'Contact sensor is not bound to one actual link')
            link = robot.find("link[@name='" + extensions[0].get('reference', '') + "']")
            collisions = link.findall('collision') if link is not None else []
            require(any(c.get('name', '') + '_collision' == sensors[0].findtext('contact/collision')
                        for c in collisions), 'Contact sensor collision binding does not resolve: ' + part)
            node.until(lambda t=topic: bool(node.get_publishers_info_by_topic(t)), 'publisher for ' + topic)
            endpoints = [{'node_name': e.node_name, 'node_namespace': e.node_namespace, 'topic_type': e.topic_type,
                          'gid': list(e.endpoint_gid)} for e in node.get_publishers_info_by_topic(topic)]
            require(any(e['topic_type'] == 'ros_gz_interfaces/msg/Contacts' for e in endpoints), 'Wrong contact source type')
            result['contact_sources'][part] = {'topic': topic, 'sensor_xml': ET.tostring(sensors[0], encoding='unicode'),
                                               'publishers': endpoints}
        result['contact_sources']['base']['observation'] = ('silent_configured_event_sensor' if
            node.counts.get('/smb/contact/base', 0) == 0 else 'only_empty_messages_observed')
        result['checks']['configured_contact_sources_and_no_recorded_chassis_contact'] = True

        result['clouds'] = {}; result['sensor_tf'] = {}
        for name, spec in SENSORS.items():
            pair = dict(node.pairs[name])
            for kind in ('raw', 'adapted'):
                message = pair[kind]
                details = describe_cloud(message, spec[kind])
                details['raw_message'] = save_message(name + '_' + kind, message)
                result['clouds'][spec[kind]] = details
                require(details['finite_points'] > 100 and details.get('nonplanar_returns', False),
                        'No sufficient finite 3D returns: ' + spec[kind])
            require(pair['adapted'].header.frame_id == spec['frame'], 'Wrong adapted lidar frame')
            at = Time(nanoseconds=stamp_ns(pair['adapted']))
            result['sensor_tf'][name] = []
            for target in ('base_link', 'map'):
                node.until(lambda t=target: node.tf.can_transform(t, spec['frame'], at), 'acquisition-time sensor TF')
                tf = node.tf.lookup_transform(target, spec['frame'], at)
                t, q = tf.transform.translation, tf.transform.rotation
                values = [t.x, t.y, t.z, q.x, q.y, q.z, q.w]
                require(all(math.isfinite(v) for v in values) and abs(math.hypot(q.x, q.y, q.z, q.w)-1.) < 1.e-5,
                        'Invalid sensor TF')
                result['sensor_tf'][name].append({'target': target, 'source': spec['frame'],
                    'requested_stamp_ns': stamp_ns(pair['adapted']), 'stamp_ns': stamp_ns(tf),
                    'translation': values[:3], 'quaternion': values[3:],
                    'raw_message': save_message(name + '_tf_' + target, tf)})
        result['checks']['both_lidars_have_3d_returns_and_tf'] = True
        response = node.call(node.mapper, Trigger.Request(), 'mapper_status')
        result['mapper_status_response'] = node.timed({'success': response.success, 'message': response.message})
        require(response.success, 'mapper_status failed')
        mapper = json.loads(response.message)
        result['mapper_status'] = mapper
        require(mapper['backend'] == 'voxblox_tsdf' and mapper['known_cells'] > 0 and
                mapper['freespace']['unknown_box_operations'] == 0, 'Empty TSDF or unexpected unknown-box clearing')
        for name, spec in SENSORS.items():
            source = mapper['input_sources'].get(name, {})
            require(source.get('integrated_scans', 0) > 0 and source.get('topic', '').lstrip('/') == spec['adapted'] and
                    source.get('frame') == spec['frame'], 'No integrated scans from ' + name)
        cloud = node.latest['voxel_map']
        occupancy = field_values(cloud, 'occupancy')
        require(cloud.header.frame_id == 'map' and stamp_ns(cloud) > 0 and
                all(v in (-1, 0, 100) for v in occupancy), 'Invalid voxel snapshot')
        known = sum(v in (0, 100) for v in occupancy)
        require(known > 0, 'Received snapshot has no known cells')
        result['voxel_snapshot'] = {**cloud_metadata(cloud, 'voxel_map'), 'known_cells': known,
                                    'raw_message': save_message('voxel_map', cloud)}
        result['checks']['both_sources_integrated_known_snapshot_no_unknown_box_clear'] = True

        def native_query(label, argv, timeout_s=6.):
            record = node.timed({'command': argv, 'partition': native_env['IGN_PARTITION']})
            out_path, err_path = evidence / (label + '.stdout'), evidence / (label + '.stderr')
            with out_path.open('x') as stdout, err_path.open('x') as stderr:
                query = subprocess.Popen(argv, stdout=stdout, stderr=stderr, env=native_env)
                try:
                    node.until(lambda: query.poll() is not None, label, time.monotonic() + timeout_s)
                except TimeoutError:
                    record['timed_out'] = True
                finally:
                    if query.poll() is None:
                        query.terminate()
                        try: query.wait(timeout=1.)
                        except subprocess.TimeoutExpired:
                            query.kill(); query.wait(timeout=1.)
                    record.update(returncode=query.returncode, end_wall_s=time.monotonic()-started,
                                  end_ros_ns=node.clock_ns, stdout_file=str(out_path), stderr_file=str(err_path))
                    result.setdefault('native_queries', {})[label] = record
            return out_path.read_text(), record

        # Observe actual static placement. Query failure remains unverified.
        static_text, static_query = native_query('native_scene', ['ign', 'service', '-s', '/world/pittsburgh/scene/info',
            '--reqtype', 'ignition.msgs.Empty', '--reptype', 'ignition.msgs.Scene', '--timeout', '3000', '--req', ''])
        try:
            require(static_query['returncode'] == 0 and not static_query.get('timed_out'), 'Native static query unavailable')
            actual = scene_model_pose(static_text, 'pittsburgh_mine')
            result['static_mine_pose'] = actual
            require(math.dist(actual['position'], [-22., 92.5, 0.]) < 1.e-6 and
                    min(math.dist(actual['quaternion'], [0., 0., 0., 1.]),
                        math.dist(actual['quaternion'], [0., 0., 0., -1.])) < 1.e-6, 'Native mine pose differs')
            result['static_pose_verified'] = True
        except Exception as error:
            result['static_pose_error'] = type(error).__name__ + ': ' + str(error)

        # A fresh window must cover the dynamic snapshot, not just startup.
        node.until(lambda: node.support_window() is not None, 'stable pose before native snapshot')
        dynamic_text, dynamic_query = native_query('native_dynamic_poses',
            ['ign', 'topic', '-e', '-t', '/world/pittsburgh/dynamic_pose/info', '-n', '1', '--json-output'])
        require(dynamic_query['returncode'] == 0 and not dynamic_query.get('timed_out'), 'Native dynamic query failed')
        messages = json_objects(dynamic_text)
        require(messages, 'No native dynamic snapshot')
        native = messages[0]
        models = [p for p in native['pose'] if p.get('name') == 'smb']
        bases = [p for p in native['pose'] if p.get('name', '').split('::')[-1] == 'base_link']
        require(len(models) == 1 and len(bases) == 1, 'Ambiguous native SMB/base pose')
        position, orientation = compose_native_pose(models[0], bases[0])
        stamp = native['header']['stamp']
        native_ns = int(stamp.get('sec', 0))*1000000000 + int(stamp.get('nsec', 0))
        node.until(lambda: node.raw_recent and node.raw_recent[-1]['stamp_ns'] >= native_ns and
                   node.recent[-1]['stamp_ns'] >= native_ns + 100000000,
                   'odometry after native snapshot')
        # Find a stable >=1 s interval ending just after the snapshot.
        rows = [r for r in node.recent if r['stamp_ns'] <= native_ns + 100000000]
        window = stable_window(rows, node.ready_ns)
        require(window is not None and window['start_ns'] <= native_ns <= window['end_ns'],
                'Native snapshot is not inside a recorded stable window')
        nearest = min(node.raw_recent, key=lambda row: abs(row['stamp_ns'] - native_ns))
        separation = abs(nearest['stamp_ns'] - native_ns)*1.e-9
        position_error = math.dist(position, nearest['pose'][:3])
        quaternion_error = min(math.dist(orientation, nearest['quaternion']),
                               math.dist(orientation, [-v for v in nearest['quaternion']]))
        result['native_world_pose_audit'] = {'model': models[0], 'base_relative_to_model': bases[0],
            'composed_position': position, 'composed_quaternion': orientation, 'native_stamp_ns': native_ns,
            'nearest_odometry': nearest, 'stamp_separation_s': separation, 'position_error_m': position_error,
            'quaternion_component_distance': quaternion_error, 'stable_window': window,
            'captured_native_messages': len(messages)}
        require(separation <= .03 and position_error < 1.e-4 and quaternion_error < 1.e-4,
                'Native world pose and odometry differ')
        result['odometry_world_pose_runtime_verified'] = True
        result['checks']['native_world_pose_matches_odometry'] = True
        result['checks']['native_static_mine_pose'] = result['static_pose_verified']
        require(result['static_pose_verified'], 'Static mine placement remains unverified')
        result['status'] = 'passed'
    except Exception as error:
        result['error'] = type(error).__name__ + ': ' + str(error)
        result['traceback'] = traceback.format_exc()
    finally:
        if node is not None:
            result['received_counts'] = node.counts
            result['nonempty_contact_messages'] = node.contact_nonempty
            result['nonzero_velocity_commands'] = node.nonzero_commands
            result['last_sim_ns'] = node.clock_ns
            result['last_odometry'] = node.recent[-1] if node.recent else None
            # A support timeout must still preserve the scans/map that arrived.
            # These are last-received observations, not silently promoted to
            # the matched snapshots used by the successful checks above.
            try:
                result['last_received_clouds'] = {}
                for topic, cloud in node.latest.items():
                    result['last_received_clouds'][topic] = {
                        **cloud_metadata(cloud, topic),
                        'raw_message': save_message('last_' + topic.replace('/', '_'), cloud)}
                if 'mapper_status_response' not in result and node.mapper.service_is_ready() and process.poll() is None:
                    future = node.mapper.call_async(Trigger.Request())
                    exit_deadline = time.monotonic() + 2.
                    while not future.done() and time.monotonic() < exit_deadline:
                        rclpy.spin_once(node, timeout_sec=.01)
                    if future.done() and future.result() is not None:
                        response = future.result()
                        result['mapper_status_response_at_exit'] = node.timed(
                            {'success': response.success, 'message': response.message})
            except Exception as error:
                result['status'] = 'failed'
                result['exit_evidence_error'] = type(error).__name__ + ': ' + str(error)
        result['cleanup'] = shutdown_launch(process)
        log.close(); history.close()
        for record in streams.values():
            record.pop('handle').close()
            record['sha256'] = digest(record['file'])
        result['raw_streams'] = streams
        log_text = log_path.read_text(errors='replace')
        deaths, escalations = shutdown_log_events(log_text)
        result['node_exit_failures'], result['launch_signal_escalations'] = deaths, escalations
        cleanup = result['cleanup']
        result['checks']['clean_shutdown'] = bool(cleanup.get('started') and cleanup.get('launch_returncode') == 0 and
            not cleanup.get('forced_kill') and not cleanup.get('emergency_sigterm') and
            not cleanup.get('process_group_remaining') and not deaths and not escalations and
            'Traceback (most recent call last)' not in log_text)
        result['source_sha256_at_end'] = {name: digest(path) if path.is_file() else None
                                          for name, path in source_paths.items()}
        result['installed_sha256_at_end'] = {name: digest(path) if path.is_file() else None
                                             for name, path in installed_paths.items()}
        result['checks']['source_and_install_unchanged'] = bool(source_paths and
            result.get('source_sha256_at_start') == result['source_sha256_at_end'] and
            result.get('installed_sha256_at_start') == result['installed_sha256_at_end'])
        if not result['checks']['clean_shutdown'] or not result['checks']['source_and_install_unchanged']:
            result['status'] = 'failed'
            result.setdefault('error', 'Unclean shutdown or source/install changed')
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        result.update(wall_elapsed_s=time.monotonic()-started, log_file=str(log_path))
        result_file.write(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
        result_file.close()
    print(json.dumps({'status': result['status'], 'output': str(output), 'error': result.get('error')}, ensure_ascii=False))
    return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
