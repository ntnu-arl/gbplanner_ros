#!/usr/bin/env python3
"""Observe one native RMF mission after a single start request.

No trajectory, takeoff, home, PCI trigger, landing or disarm commands are sent
by this observer. The installed mission node owns the complete sequence.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

from check_edgar_mapping import cloud_metadata, require, stamp_ns, stop_process_tree


IMU_STREAMS = {
    'adis_raw': ('/rmf/imu/adis/raw', None, 'imu_link', 500.),
    'adis_output': ('/rmf/imu', 'imu_link', 'imu_link', 500.),
    'ground_truth_raw': ('/rmf/imu/ground_truth/raw', None, 'imugt_link', 500.),
    'ground_truth_output': ('/rmf/ground_truth/imu', 'imugt_link', 'imugt_link', 500.),
    'os0_raw': ('/rmf/imu/os0/raw', None, 'os0_imu', 100.),
    'os0_output': ('/os0_cloud_node/imu', 'os0_imu', 'os0_imu', 100.),
}


class VectorMoments:
    """Constant-memory statistics for all received finite observations."""
    def __init__(self):
        self.count = 0
        self.mean, self.m2 = [0.]*3, [0.]*3
        self.norm_sum = self.norm_square_sum = 0.
        self.norm_min, self.norm_max = None, None

    def add(self, values):
        self.count += 1
        for index, value in enumerate(values):
            delta = value-self.mean[index]
            self.mean[index] += delta/self.count
            self.m2[index] += delta*(value-self.mean[index])
        norm = math.hypot(*values)
        self.norm_sum += norm
        self.norm_square_sum += norm*norm
        self.norm_min = min(self.norm_min, norm) if self.norm_min is not None else norm
        self.norm_max = max(self.norm_max, norm) if self.norm_max is not None else norm

    def summary(self):
        return {'count': self.count, 'mean_xyz': self.mean if self.count else None,
                'stddev_xyz': [math.sqrt(max(0., x/self.count)) for x in self.m2] if self.count else None,
                'norm_mean': self.norm_sum/self.count if self.count else None,
                'norm_rms': math.sqrt(self.norm_square_sum/self.count) if self.count else None,
                'norm_min': self.norm_min, 'norm_max': self.norm_max}


class ImuObservations:
    def __init__(self, specification):
        self.topic, self.expected_frame, self.source_link, self.nominal_rate = specification
        self.count = self.nonfinite = self.nonpositive_stamp = self.nonmonotonic = self.bad_frame = 0
        self.first_stamp = self.last_stamp = None
        self.interval_count = 0
        self.interval_sum = 0.
        self.interval_min = self.interval_max = None
        self.frames = set()
        self.samples = {key: [] for key in ('ground', 'takeoff', 'flight', 'landing')}
        self.phase_counts = {key: 0 for key in self.samples}
        self.moments = {key: {field: VectorMoments() for field in ('gyro_rad_s', 'accel_m_s2')}
                        for key in ('all', 'stationary', 'flight')}
        self.violations = []

    def add(self, record, group, stationary, flying):
        self.count += 1
        stamp, frame = record['stamp_ns'], record['frame_id']
        errors = []
        if stamp <= 0:
            self.nonpositive_stamp += 1
            errors.append('nonpositive_stamp')
        if self.last_stamp is not None:
            interval = (stamp-self.last_stamp)*1.e-9
            if interval <= 0:
                self.nonmonotonic += 1
                errors.append('nonmonotonic_stamp')
            else:
                self.interval_count += 1
                self.interval_sum += interval
                self.interval_min = min(self.interval_min, interval) if self.interval_min is not None else interval
                self.interval_max = max(self.interval_max, interval) if self.interval_max is not None else interval
        self.first_stamp = stamp if self.first_stamp is None else self.first_stamp
        self.last_stamp = stamp
        self.frames.add(frame)
        if not frame or (self.expected_frame is not None and frame != self.expected_frame):
            self.bad_frame += 1
            errors.append('frame_contract')
        keys = ('gyro_rad_s', 'accel_m_s2', 'orientation_xyzw', 'gyro_covariance',
                'accel_covariance', 'orientation_covariance')
        finite = all(math.isfinite(x) for key in keys for x in record[key])
        if not finite:
            self.nonfinite += 1
            errors.append('nonfinite_values')
            record = dict(record)
            for key in keys:
                record[key] = [x if math.isfinite(x) else str(x) for x in record[key]]
        else:
            categories = ['all']+(['stationary'] if stationary else [])+(['flight'] if flying else [])
            for category in categories:
                for field, moments in self.moments[category].items():
                    moments.add(record[field])
        if errors and len(self.violations) < 10:
            self.violations.append({'errors': errors, 'record': record})
        if group in self.samples:
            self.phase_counts[group] += 1
            # Retain one in ten plus the first sample, capped per flight phase.
            # All observations still participate in counts and validity checks.
            if (self.phase_counts[group]-1) % 10 == 0 and len(self.samples[group]) < 1000:
                self.samples[group].append(record)

    def summary(self):
        span = (self.last_stamp-self.first_stamp)*1.e-9 if self.count > 1 else 0.
        valid = (self.count >= 2 and not any((self.nonfinite, self.nonpositive_stamp,
                  self.nonmonotonic, self.bad_frame)) and len(self.frames) == 1)
        return {'topic': self.topic, 'count': self.count, 'valid': valid,
                'expected_output_frame': self.expected_frame, 'observed_frames': sorted(self.frames),
                'source_link_mapping': self.source_link,
                'raw_frame_check': 'nonempty and stable; observed native name, not an asserted link-name match'
                    if self.expected_frame is None else None,
                'nominal_source_hz': self.nominal_rate,
                'observed_acquisition_hz': (self.count-1)/span if span > 0 else None,
                'first_stamp_ns': self.first_stamp, 'last_stamp_ns': self.last_stamp,
                'interval_mean_s': self.interval_sum/self.interval_count if self.interval_count else None,
                'interval_min_s': self.interval_min, 'interval_max_s': self.interval_max,
                'nonfinite': self.nonfinite, 'nonpositive_stamp': self.nonpositive_stamp,
                'nonmonotonic': self.nonmonotonic, 'bad_frame': self.bad_frame,
                'phase_counts': self.phase_counts,
                'retained_samples': {key: len(values) for key, values in self.samples.items()},
                'physical_summary': {key: {field: item.summary() for field, item in values.items()}
                                     for key, values in self.moments.items()},
                'violations': self.violations}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', choices=('upstream', 'quintic', 'quintic_collinear'), default='quintic')
    parser.add_argument('--timeout', type=float, default=360.)
    parser.add_argument('--cycles', type=int, default=1)
    parser.add_argument('--require-moving-continuation', action='store_true')
    parser.add_argument('--capture-validation', action='store_true')
    parser.add_argument('--trace-mapping', action='store_true',
                        help='Record mapper stages, planner map acceptance and ESDF product metadata')
    args = parser.parse_args()
    if args.cycles < 1 or (args.require_moving_continuation and args.cycles < 2):
        parser.error('cycles must be positive; moving continuation requires at least two cycles')
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be positive and finite')
    output = args.output.resolve()
    log_path, history_path = output.with_suffix('.log'), output.with_name(output.stem+'_history.json')
    metadata_path = output.with_name(output.stem+'_model.json')
    imu_history_path = output.with_name(output.stem+'_imu_history.json')
    cloud_path = output.with_name(output.stem+'_accepted_map.bin')
    mapper_trace_path = output.with_name(output.stem+'_mapper.jsonl')
    planner_trace_path = output.with_name(output.stem+'_planner.jsonl')
    validation_directory = output.with_name(output.stem+'_validation') if args.capture_validation else None
    if validation_directory is not None and validation_directory.exists():
        parser.error('Validation capture directory already exists: '+str(validation_directory))
    if any(path.exists() for path in (output, log_path, history_path, metadata_path, imu_history_path,
                                     cloud_path, mapper_trace_path, planner_trace_path)):
        parser.error('Choose an unused output name; previous results are retained')
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid() % 80))
    started, deadline = time.monotonic(), time.monotonic()+args.timeout
    root = Path(__file__).resolve().parents[1]
    sources = [root/'src/gbplanner_gazebo/scripts/rmf_mission_node.py',
               root/'src/gbplanner_gazebo/config/rmf_exploration.yaml',
               root/'src/gbplanner_gazebo/src/rmf_controller_node.cpp',
               root/'src/gbplanner_gazebo/launch/rmf_fortress.launch.py',
               root/'src/gbplanner_gazebo/config/rmf_controller.yaml',
               root/'src/gbplanner_gazebo/config/rmf_exploration.yaml',
               root/'src/gbplanner_gazebo/scripts/fortress_rmf_model.py',
               root/'src/gbplanner_gazebo/scripts/fortress_state_node.py',
               root/'src/gbplanner_gazebo/src/managed_bridge_node.cpp',
               root/'src/gbplanner_gazebo/src/rmf_imu_adapter_node.cpp',
               root/'src/gbplanner_gazebo/CMakeLists.txt',
               root/'src/gbplanner_gazebo/worlds/edgar_fortress.sdf',
               root/'src/gbplanner_local/scripts/control_node.py',
               root/'src/gbplanner_local/src/local_planner_node.cpp',
               root/'src/gbplanner_local/src/local_planner.cpp',
               root/'src/gbplanner_local/include/gbplanner_local/local_planner.hpp',
               root/'src/gbplanner_local/include/gbplanner_local/path_validation.hpp',
               root/'src/gbplanner_voxblox/src/voxblox_mapper_node.cpp',
               root/'src/gbplanner_voxblox/include/gbplanner_voxblox/distance_products.hpp',
               root/'src/gbplanner_voxblox/include/gbplanner_voxblox/map_visualization.hpp',
               root/'src/gbplanner_voxblox/include/gbplanner_voxblox/pointcloud_buffer.hpp',
               root/'src/gbplanner_local/src/sparse_voxel_map.cpp',
               root/'src/gbplanner_local/include/gbplanner_local/sparse_voxel_map.hpp',
               root/'src/gbplanner_local/include/gbplanner_local/voxel_index_table.hpp',
               root/'src/gbplanner_execution/scripts/path_follower_node.py',
               root/'src/gbplanner_execution/scripts/trajectory_tools.py',
               root/'src/gbplanner_execution/scripts/timed_continuation.py',
               root/'src/gbplanner_local/scripts/path_progress.py', Path(__file__).resolve()]
    hashes = {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    result = {'status': 'failed', 'scope': 'Native RMF autonomous mission, original Lee controller and DART physics',
              'profile': args.profile,
              'start_requests': 0, 'observer_trajectory_commands': 0, 'requested_local_cycles': args.cycles,
              'require_moving_continuation': args.require_moving_continuation,
              'source_sha256_at_start': hashes, 'checks': {}}
    history = {name: [] for name in ('odometry', 'controller', 'task', 'mission', 'control',
                                    'execution', 'map', 'contacts', 'trajectories', 'motor_commands',
                                    'joints', 'paths', 'scans', 'ahead_planning', 'gazebo')}
    if args.trace_mapping:
        history['esdf_products'] = []
    imu_observations = {name: ImuObservations(spec) for name, spec in IMU_STREAMS.items()}
    node = process = log = None
    initialized = False

    import rclpy
    from actuator_msgs.msg import Actuators
    from nav_msgs.msg import Odometry, Path as RosPath
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from ros_gz_interfaces.msg import Contacts
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, JointState, PointCloud2
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from trajectory_msgs.msg import MultiDOFJointTrajectory

    class Observer(Node):
        def __init__(self):
            super().__init__('rmf_mission_observer', parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.sim_s = 0.
            self.values = {}
            self.current = None
            self.accepted_cloud = None
            self.log_scan_due, self.log_offset, self.log_fragment = 0., 0, ''
            self.start_client = self.create_client(Trigger, 'rmf/start_mission')
            self.mapper_client = self.create_client(Trigger, 'mapper_status')
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
            self.create_subscription(Odometry, 'odometry', self.odom, qos_profile_sensor_data)
            self.create_subscription(String, 'rmf_controller_state', lambda m: self.status('controller', m), 10)
            self.create_subscription(String, 'gazebo_state', lambda m: self.status('gazebo', m), 10)
            for topic, name in [('rmf_mission_state', 'task'), ('mission_status', 'mission'),
                                ('control_status', 'control'), ('execution_status', 'execution'),
                                ('map_updates', 'map'), ('plan_ahead_metrics', 'ahead_planning')]:
                self.create_subscription(String, topic, lambda m, n=name: self.status(n, m), durable)
            self.create_subscription(MultiDOFJointTrajectory, 'command/trajectory', self.trajectory, 10)
            self.create_subscription(Actuators, 'rmf/command/motor_speed', self.motors, 10)
            self.create_subscription(JointState, 'joint_states', self.joints, qos_profile_sensor_data)
            self.create_subscription(RosPath, 'command_path', self.path, durable)
            self.create_subscription(PointCloud2, 'points', self.scan, qos_profile_sensor_data)
            self.create_subscription(PointCloud2, 'accepted_voxel_map',
                                     lambda m: setattr(self, 'accepted_cloud', m), durable)
            if args.trace_mapping:
                for topic in ('esdf_voxels', 'esdf_pointcloud', 'traversable'):
                    self.create_subscription(PointCloud2, topic,
                        lambda m, name=topic: history['esdf_products'].append(
                            self.record({'topic': name, 'stamp_ns': stamp_ns(m), 'width': m.width})), durable)
            imu_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
            for name, (topic, _, _, _) in IMU_STREAMS.items():
                self.create_subscription(Imu, topic, lambda m, n=name: self.imu(m, n), imu_qos)
            for part in ('base', 'rotor_0', 'rotor_1', 'rotor_2', 'rotor_3'):
                self.create_subscription(Contacts, '/rmf/contact/'+part,
                    lambda m, p=part: self.contact(m, p), qos_profile_sensor_data)

        def record(self, value):
            return {'wall_s': time.monotonic()-started, 'sim_s': self.sim_s,
                    'task_phase': self.values.get('task', {}).get('state', 'STARTUP'), **value}

        def clock(self, message):
            self.sim_s = message.clock.sec+message.clock.nanosec*1.e-9

        def status(self, name, message):
            self.values[name] = json.loads(message.data)
            history[name].append(self.record(self.values[name]))

        def odom(self, message):
            self.current = message
            p, q = message.pose.pose.position, message.pose.pose.orientation
            v, w = message.twist.twist.linear, message.twist.twist.angular
            history['odometry'].append(self.record({'stamp_ns': stamp_ns(message),
                'position': [p.x, p.y, p.z], 'quaternion': [q.x, q.y, q.z, q.w],
                'body_linear': [v.x, v.y, v.z], 'body_angular': [w.x, w.y, w.z]}))

        def motors(self, message):
            history['motor_commands'].append(self.record({'stamp_ns': stamp_ns(message),
                                                          'velocity': list(message.velocity)}))

        def joints(self, message):
            history['joints'].append(self.record({'stamp_ns': stamp_ns(message), 'names': list(message.name),
                'velocity': list(message.velocity), 'position': list(message.position)}))

        def scan(self, message):
            history['scans'].append(self.record(cloud_metadata(message, 'points')))

        def imu(self, message, name):
            gyro, accel, orientation = message.angular_velocity, message.linear_acceleration, message.orientation
            flight = self.values.get('controller', {}).get('flight', {}).get('state', 'IDLE')
            group = ('ground' if flight in {'IDLE', 'GROUNDED'} else 'takeoff' if flight == 'TAKING_OFF'
                     else 'landing' if flight in {'LANDING', 'LANDED'} else 'flight' if flight == 'AIRBORNE' else None)
            stationary = False
            if self.current is not None and abs(stamp_ns(message)-stamp_ns(self.current)) <= 100_000_000:
                v, w = self.current.twist.twist.linear, self.current.twist.twist.angular
                stationary = math.hypot(v.x, v.y, v.z) <= .05 and math.hypot(w.x, w.y, w.z) <= .05
            record = self.record({'stamp_ns': stamp_ns(message), 'frame_id': message.header.frame_id,
                'flight_state': flight, 'stationary_from_recent_odometry': stationary,
                'gyro_rad_s': [gyro.x, gyro.y, gyro.z], 'accel_m_s2': [accel.x, accel.y, accel.z],
                'orientation_xyzw': [orientation.x, orientation.y, orientation.z, orientation.w],
                'gyro_covariance': list(message.angular_velocity_covariance),
                'accel_covariance': list(message.linear_acceleration_covariance),
                'orientation_covariance': list(message.orientation_covariance)})
            imu_observations[name].add(record, group, stationary, flight in {'TAKING_OFF', 'AIRBORNE', 'LANDING'})

        def path(self, message):
            history['paths'].append(self.record({'stamp_ns': stamp_ns(message),
                'path': [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in message.poses]}))

        def contact(self, message, part):
            history['contacts'].append(self.record({'stamp_ns': stamp_ns(message), 'part': part,
                'pairs': [[c.collision1.name, c.collision2.name] for c in message.contacts],
                'positions': [[p.x, p.y, p.z] for c in message.contacts for p in c.positions],
                'normals': [[n.x, n.y, n.z] for c in message.contacts for n in c.normals],
                'depths': [d for c in message.contacts for d in c.depths if math.isfinite(d)]}))

        def trajectory(self, message):
            samples = []
            for point in message.points:
                samples.append({'offset_s': point.time_from_start.sec+point.time_from_start.nanosec*1.e-9,
                    'poses': [{'position': [p.translation.x, p.translation.y, p.translation.z],
                              'quaternion': [p.rotation.x, p.rotation.y, p.rotation.z, p.rotation.w]}
                             for p in point.transforms],
                    'velocities': [[v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z]
                                   for v in point.velocities],
                    'accelerations': [[v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z]
                                      for v in point.accelerations]})
            history['trajectories'].append(self.record({'stamp_ns': stamp_ns(message),
                'frame_id': message.header.frame_id, 'joint_names': list(message.joint_names), 'samples': samples}))

        def tick(self):
            require(time.monotonic() < deadline, 'Observer wall deadline')
            require(process.poll() is None, 'Launch exited before mission completed')
            if time.monotonic() >= self.log_scan_due:
                self.log_scan_due = time.monotonic()+.5
                with log_path.open() as stream:
                    stream.seek(self.log_offset)
                    lines = (self.log_fragment+stream.read()).split('\n')
                    self.log_offset = stream.tell()
                self.log_fragment = lines.pop()
                for line in lines:
                    require(not ('[ERROR]' in line and ('process has died' in line or
                                'exception occurred while executing process' in line)),
                            'Launch reported node failure: '+line)
            rclpy.spin_once(self, timeout_sec=.02)

        def capture_mapper_status(self):
            # This optional exit diagnostic must not use tick(): the mission
            # may already have failed or exhausted its unchanged deadline.
            capture_started, future = time.monotonic(), None
            capture = {'available': False, 'requested': False, 'timeout_s': 1.}
            result['mapper_status_capture'] = capture
            try:
                if not rclpy.ok() or process is None or process.poll() is not None:
                    capture['reason'] = 'ROS context or launch unavailable'
                    return
                if not self.mapper_client.service_is_ready():
                    capture['reason'] = 'mapper_status service unavailable at exit'
                    return
                future = self.mapper_client.call_async(Trigger.Request())
                capture['requested'] = True
                while (not future.done() and rclpy.ok() and process.poll() is None and
                       time.monotonic()-capture_started < capture['timeout_s']):
                    rclpy.spin_once(self, timeout_sec=.02)
                if not future.done():
                    capture['reason'] = 'mapper_status response unavailable within exit diagnostic budget'
                    return
                response = future.result()
                if response is None:
                    capture['reason'] = 'mapper_status returned no response'
                    return
                result['mapper_at_exit'] = json.loads(response.message)
                capture['available'], capture['response_success'] = True, response.success
            except Exception as error:
                capture['error'] = type(error).__name__+': '+str(error)
            finally:
                if future is not None and not future.done():
                    try:
                        future.cancel()
                    except Exception as error:
                        capture['cancel_error'] = type(error).__name__+': '+str(error)
                capture['wall_elapsed_s'] = time.monotonic()-capture_started

    try:
        log = log_path.open('x')
        command = ['ros2', 'launch', 'gbplanner_gazebo', 'rmf_fortress.launch.py',
            'mission:=true', 'planning:=true', 'mapping:=true', 'contacts:=true', f'max_cycles:={args.cycles}',
            'trajectory_profile:='+args.profile, f'model_metadata_file:={metadata_path}']
        if validation_directory is not None:
            command.append('validation_snapshot_directory:='+str(validation_directory))
        if args.trace_mapping:
            command.extend(['mapper_trace_file:='+str(mapper_trace_path),
                            'map_trace_file:='+str(planner_trace_path)])
            result['mapping_trace_files'] = [str(mapper_trace_path), str(planner_trace_path)]
        result['launch_command'] = command
        result['validation_snapshot_directory'] = str(validation_directory) if validation_directory else None
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init(args=[])
        initialized = True
        node = Observer()
        while not (node.start_client.service_is_ready() and node.values.get('task')):
            node.tick()
        require(node.values['task']['state'] == 'IDLE', 'Mission auto-started before the explicit request')
        result['start_requests'] = 1
        future = node.start_client.call_async(Trigger.Request())
        response_deadline = time.monotonic()+30.
        while not future.done():
            node.tick()
            require(time.monotonic() < response_deadline, 'start_mission response timeout')
        response = future.result()
        result['start_response'] = {'success': response.success, 'message': response.message}
        require(response.success, 'Mission start refused: '+response.message)
        printed = None
        while node.values.get('task', {}).get('state') not in {'COMPLETE', 'FAILED'}:
            node.tick()
            task = node.values.get('task', {})
            state = task.get('state')
            if state != printed:
                print(json.dumps({'state': state, 'reason': task.get('reason'), 'sim_s': node.sim_s}), flush=True)
                printed = state
        task = node.values['task']
        require(task['state'] == 'COMPLETE', 'Mission failed: '+json.dumps(task))
        # Observe independently after completion so terminal state is paired
        # with native contact messages and actual actuator commands.
        completed = node.sim_s
        while node.sim_s-completed < 1.:
            node.tick()
        flight = node.values['controller'].get('flight', {})
        require(flight.get('state') == 'LANDED' and flight.get('touchdown_confirmed') is True,
                'Controller did not confirm native touchdown')
        require(all(task['requests'].get(k) == 1 for k in ('takeoff', 'set_home', 'exploration', 'landing')),
                'Mission command counts differ from one per stage')
        require(task['requests'].get('stop') == 0 and task['requests'].get('hold') == 0,
                'Nominal mission issued failure stop/hold')
        require(task['root_checks'] and task['root_checks'][-1]['valid'], 'No accepted Extended root check')
        require(any(r['part'] == 'base' and r['pairs'] and r['sim_s'] >= completed+.2
                    for r in history['contacts']), 'No independent post-completion base contact')
        landed_motors = [r for r in history['motor_commands'] if r['sim_s'] >= completed+.2]
        require(landed_motors and all(len(r['velocity']) == 4 and
                    all(abs(v) <= 1.e-9 for v in r['velocity']) for r in landed_motors),
                'Motor commands were not consistently zero after touchdown')
        completed_actions = {r['completed_actions']: r['mode'] for r in history['control']
                             if r.get('state') == 'WAITING_OBSERVATION'}
        execution = node.values.get('execution', {})
        continuations = execution.get('continuations', 0)
        local_actions = list(completed_actions.values()).count('local')
        require(local_actions+continuations == args.cycles and
                list(completed_actions.values()).count('homing') == 1,
                'Local completed actions + continuations must match requested cycles, then one homing')
        commits = {row['continuations']: row['last_continuation'] for row in history['execution']
                   if row.get('last_continuation') and row.get('continuations', 0) > 0}
        result['continuation_commits'] = list(commits.values())
        result['completed_local_actions'] = local_actions
        if args.require_moving_continuation:
            require(any(c['old_remaining_time_s'] > 0. and c['old_endpoint_distance_m'] > .05 and
                        c['measured_speed_mps'] > .05 for c in commits.values()),
                    'No continuation committed while still moving before the old endpoint')
        require(history['trajectories'] and history['scans'] and
                node.values.get('map', {}).get('known_cells', 0) > 0, 'Missing native trajectory/lidar/map evidence')
        execution = node.values.get('execution', {})
        require(execution.get('succeeded') == local_actions+1 and execution.get('aborted') == 0 and
                execution.get('cancelled') == 0, 'Unexpected follower Action results')
        imu_summaries = {name: stream.summary() for name, stream in imu_observations.items()}
        require(all(item['valid'] for item in imu_summaries.values()),
                'Missing or invalid native IMU streams: '+json.dumps(
                    {name: item for name, item in imu_summaries.items() if not item['valid']}))
        result['checks'].update(single_start=True, mission_complete=True, accepted_extended_root=True,
            single_commands_per_stage=True, requested_local_cycles_and_homing=True, native_touchdown=True,
            moving_continuation=(True if args.require_moving_continuation else None),
            zero_motor_commands=True, native_trajectory_lidar_map=True,
            three_native_imu_raw_and_output=True, imu_frame_finite_monotonic=True)
        result['status'] = 'passed'
    except Exception as error:
        result['error'] = type(error).__name__+': '+str(error)
    finally:
        if node is not None:
            node.capture_mapper_status()
            result['final'] = node.values
            if node.accepted_cloud is not None:
                cloud = node.accepted_cloud
                cloud_path.write_bytes(bytes(cloud.data))
                result['accepted_map'] = {**cloud_metadata(cloud, 'accepted_voxel_map'), 'data_file': str(cloud_path)}
        # End this experiment's simulation group; never send airborne disarm
        # or a replacement trajectory from the observer's cleanup path.
        if process is not None:
            try:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=8.)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
        result['cleanup'] = stop_process_tree(process)
        if log is not None:
            log.close()
        text = log_path.read_text() if log_path.exists() else ''
        failures = re.findall(r'\[([^\]]+)\]: process has died \[pid \d+, exit code (-?\d+)', text)
        result['node_exit_failures'] = [{'name': name, 'exit_code': int(code)} for name, code in failures]
        result['bridge_counts'] = [{'topic': topic, 'received': int(received), 'transferred': int(transferred),
                                   'publish_failures': int(errors)}
            for topic, received, transferred, errors in re.findall(
                r'Bridge drained: GZ->ROS (\S+) received=(\d+) transferred=(\d+) publish_failures=(\d+)', text)]
        result['node_start_failures'] = re.findall(
            r'\[ERROR\] \[([^\]]+)\]: exception occurred while executing process:', text)
        result['checks']['nodes_started'] = not result['node_start_failures']
        result['callback_errors'] = [line for line in text.splitlines()
                                     if 'exception was never retrieved' in line or 'Exception ignored' in line]
        result['checks']['clean_shutdown'] = not (failures or result['callback_errors'] or
                                                  result['cleanup'].get('forced_kill', False))
        if not result['checks']['clean_shutdown'] or not result['checks']['nodes_started']:
            result['status'] = 'failed'
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        result['source_sha256_at_end'] = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                                          for p in sources}
        result['checks']['sources_unchanged_during_run'] = result['source_sha256_at_end'] == hashes
        if not result['checks']['sources_unchanged_during_run']:
            result['status'] = 'failed'
        result['counts'] = {name: len(rows) for name, rows in history.items()}
        result['imu_summary'] = {name: stream.summary() for name, stream in imu_observations.items()}
        result['imu_history_file'] = str(imu_history_path)
        imu_history = {'sampling': {'validity_and_statistics': 'every received sample',
            'retention': 'first and every tenth per stream/phase, maximum 1000 per phase',
            'phases': ['ground', 'takeoff', 'flight', 'landing'], 'maximum_retained_samples': 24000,
            'frequency': 'received count over acquisition-stamp span; DDS drops remain visible',
            'stationary': 'recent odometry within 0.1 source seconds; linear and angular norms <=0.05',
            'flight': 'controller TAKING_OFF, AIRBORNE or LANDING; overlaps stationary hover',
            'physics': 'IMU values are native samples; odometry only labels motion categories',
            'noise_statistics': 'descriptive only; no statistical equivalence assertion'},
            'streams': {name: stream.samples for name, stream in imu_observations.items()}}
        imu_history_path.write_text(json.dumps(imu_history, separators=(',', ':'), allow_nan=False)+'\n')
        result['wall_elapsed_s'] = time.monotonic()-started
        result['history_file'], result['model_file'] = str(history_path), str(metadata_path)
        history_path.write_text(json.dumps(history, separators=(',', ':'), allow_nan=False)+'\n')
        output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        print(json.dumps({'status': result['status'], 'output': str(output), 'checks': result['checks'],
                          'error': result.get('error', '')[:500]}, indent=2))
    return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
