#!/usr/bin/env python3
"""Replay the recorded RMF path through the production follower and native physics.

The source polyline and allocated waypoint headings are retained. This probe
requests native takeoff, submits one fixed PathFollower goal, and lands after
successful execution. It does not request a new plan or publish trajectories,
maps, odometry, motor commands, or replacement hold commands.
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
import sys
import time

from check_edgar_mapping import cloud_metadata, require, stamp_ns, stop_process_tree


def recorded_path(history):
    """Recover every fixed vertex and its ordered heading from the old command."""
    source = next(row for row in history['paths'] if row.get('path'))
    trajectory = next(row for row in history['trajectories'] if len(row.get('samples', [])) > 1)
    samples, cursor, states = trajectory['samples'], 0, []
    for point in source['path']:
        require(len(point) == 3 and all(math.isfinite(v) for v in point), 'Invalid source vertex')
        while cursor < len(samples):
            poses = samples[cursor]['poses']
            if len(poses) == 1 and math.dist(poses[0]['position'], point) <= 1.e-10:
                break
            cursor += 1
        require(cursor < len(samples), 'Source vertex absent from ordered recorded trajectory')
        qx, qy, qz, qw = samples[cursor]['poses'][0]['quaternion']
        yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
        states.append((*point, yaw))
    executing = next(row for row in history['execution'] if row.get('state') == 'EXECUTING')
    require(executing.get('manual_path') is False, 'Source was not an automatic prepared path')
    return states, int(executing['bound_mode']), source, trajectory


def tracking_summary(states, history):
    """Replay the same ordered progress monitor without changing its tolerances."""
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root/'src/gbplanner_local/scripts'))
    from path_progress import PathProgress
    commands = [row for row in history['trajectories'] if len(row['samples']) > 1]
    if not commands:
        return {'available': False, 'reason': 'no_motion_trajectory'}
    first = commands[0]
    xyz = [sample['poses'][0]['position'] for sample in first['samples']]
    cursor, retained = 0, []
    for state in states:
        while cursor < len(xyz) and math.dist(state[:3], xyz[cursor]) > 1.e-9:
            cursor += 1
        retained.append(cursor if cursor < len(xyz) else None)
    # prepare() prepends the new measured pose when its separation is <=0.1 m.
    # This run rejects a larger initial discrepancy before dispatch, so no
    # source vertex can take reconnect's optional dropped-root branch.
    route = [xyz[0]]+[state[:3] for state in states]
    progress = PathProgress(route)
    observed = [row for row in history['odometry'] if row['phase'] == 'fixed_path'
                and row['stamp_ns'] >= first['stamp_ns']]
    failure, accepted, last_stamp = None, 0, -1
    for row in observed:
        if row['stamp_ns'] <= last_stamp:
            continue
        last_stamp = row['stamp_ns']
        try:
            progress.update(row['position'])
            accepted += 1
        except ValueError as error:
            failure = {'reason': str(error), 'stamp_ns': last_stamp,
                       'position': row['position'], 'progress_before_m': progress.progress}
            break
    corners = []
    for index in range(1, len(states)-1):
        a, b, c = (states[j][:3] for j in (index-1, index, index+1))
        first_leg, second_leg = math.dist(a, b), math.dist(b, c)
        if min(first_leg, second_leg) <= 1.e-9:
            continue
        cosine = sum((y-x)*(z-y) for x, y, z in zip(a, b, c))/(first_leg*second_leg)
        if cosine < 0 and observed:
            nearest = min(observed, key=lambda row: math.dist(row['position'], b))
            corners.append({'source_vertex_index': index, 'position': list(b),
                'turn_cosine': cosine, 'minimum_distance_m': math.dist(nearest['position'], b),
                'nearest_odometry': nearest})
    return {'available': True, 'progress_parameters': {'tolerance_m': .15, 'guard_m': .25, 'max_step_m': .75},
            'source_vertex_sample_indices': retained, 'all_source_vertices_retained': all(i is not None for i in retained),
            'progress_samples_accepted': accepted, 'progress_failure': failure,
            'observed_progress_m': progress.progress, 'total_path_length_m': progress.arc[-1],
            'reversal_vertices': corners, 'command_duration_s': first['samples'][-1]['offset_s'],
            'command_samples': len(first['samples']),
            'scope': 'Fixed recorded route, native odometry, unchanged production progress thresholds'}


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-history', type=Path, default=root/'results/course_demo/rmf1/mission_history.json')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', choices=('upstream', 'quintic', 'quintic_collinear'), default='quintic')
    parser.add_argument('--capture-validation', action='store_true',
                        help='Capture one exact rejected preflight request after its normal wait expires')
    parser.add_argument('--trace-mapping', action='store_true',
                        help='Record the production mapper stages and planner map acceptance')
    parser.add_argument('--timeout', type=float, default=360.)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be positive and finite')
    output = args.output.resolve()
    log_path, history_path = output.with_suffix('.log'), output.with_name(output.stem+'_history.json')
    metadata_path = output.with_name(output.stem+'_model.json')
    mapper_trace_path = output.with_name(output.stem+'_mapper.jsonl')
    planner_trace_path = output.with_name(output.stem+'_planner.jsonl')
    validation_directory = output.with_name(output.stem+'_validation') if args.capture_validation else None
    if validation_directory is not None and validation_directory.exists():
        parser.error('Validation capture directory already exists: '+str(validation_directory))
    if any(path.exists() for path in (output, log_path, history_path, metadata_path,
                                     mapper_trace_path, planner_trace_path)):
        parser.error('Choose an unused output name; previous results are retained')
    output.parent.mkdir(parents=True, exist_ok=True)
    source_path = args.source_history.resolve()
    source_bytes = source_path.read_bytes()
    source_history = json.loads(source_bytes)
    states, bound_mode, source_row, source_trajectory = recorded_path(source_history)
    baseline = tracking_summary(states, {'trajectories': source_history['trajectories'],
        'odometry': [dict(row, phase='fixed_path') for row in source_history['odometry']]})
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    started, process, node, log, initialized = time.monotonic(), None, None, None, False
    deadline = started+args.timeout
    result = {'status': 'failed', 'scope': __doc__, 'profile': args.profile,
        'autonomous_planning': False, 'domain': os.environ['ROS_DOMAIN_ID'],
        'comparison_scope': 'Identical fixed route in separately initialized native physics; not a deterministic state replay',
        'source_history': str(source_path), 'source_history_sha256': hashlib.sha256(source_bytes).hexdigest(),
        'source_command_stamp_ns': source_row['stamp_ns'],
        'source_trajectory_stamp_ns': source_trajectory['stamp_ns'],
        'fixed_path': [list(state) for state in states], 'bound_mode': bound_mode,
        'fixed_xyz_sha256': hashlib.sha256(json.dumps(source_row['path'], separators=(',', ':')).encode()).hexdigest(),
        'goal_manual_path': False, 'observer_trajectory_commands': 0, 'checks': {}, 'preflight_checks': [],
        'recorded_baseline': {'progress_failure': baseline.get('progress_failure'),
            'reversal_vertices': [{key: value for key, value in row.items() if key != 'nearest_odometry'}
                                  for row in baseline.get('reversal_vertices', [])]}}
    esdf_product_topics = ('esdf_voxels', 'esdf_pointcloud', 'traversable')
    history = {name: [] for name in ('odometry', 'controller', 'execution', 'map', 'contacts',
        'trajectories', 'motor_commands', 'scans', 'feedback', 'gazebo', 'esdf_products')}
    sources = [root/name for name in ('tools/check_rmf_fixed_path.py',
        'src/gbplanner_execution/scripts/path_follower_node.py',
        'src/gbplanner_execution/scripts/trajectory_tools.py',
        'src/gbplanner_local/scripts/path_progress.py',
        'src/gbplanner_local/src/sparse_voxel_map.cpp',
        'src/gbplanner_local/include/gbplanner_local/sparse_voxel_map.hpp',
        'src/gbplanner_local/include/gbplanner_local/voxel_index_table.hpp',
        'src/gbplanner_voxblox/src/voxblox_mapper_node.cpp',
        'src/gbplanner_gazebo/scripts/fortress_state_node.py',
        'src/gbplanner_gazebo/src/rmf_controller_node.cpp',
        'src/gbplanner_gazebo/config/rmf_exploration.yaml',
        'src/gbplanner_gazebo/config/rmf_controller.yaml',
        'src/gbplanner_gazebo/launch/rmf_fortress.launch.py')]
    hashes = {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    result['source_sha256_at_start'] = hashes

    import rclpy
    from action_msgs.msg import GoalStatus
    from actuator_msgs.msg import Actuators
    from geometry_msgs.msg import Pose, PoseStamped
    from nav_msgs.msg import Odometry
    from planner_msgs.action import PathFollower
    from planner_msgs.srv import ValidatePath
    from rcl_interfaces.srv import GetParameters, SetParameters
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from ros_gz_interfaces.msg import Contacts
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from trajectory_msgs.msg import MultiDOFJointTrajectory

    def pose(state):
        message = Pose()
        message.position.x, message.position.y, message.position.z = state[:3]
        message.orientation.z, message.orientation.w = math.sin(state[3]/2), math.cos(state[3]/2)
        return message

    class Probe(Node):
        def __init__(self):
            super().__init__('rmf_fixed_path_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.phase, self.sim_s, self.current, self.goal_handle = 'startup', 0., None, None
            self.values = {}
            self.takeoff, self.land = (self.create_client(Trigger, name) for name in ('rmf/takeoff', 'rmf/land'))
            self.validator = self.create_client(ValidatePath, 'validate_path')
            self.state_client = self.create_client(Trigger, 'execution_state')
            self.mapper_client = self.create_client(Trigger, 'mapper_status')
            self.parameters = self.create_client(GetParameters, '/gbplanner_path_follower/get_parameters')
            self.action = ActionClient(self, PathFollower, 'path_follower')
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
            self.create_subscription(Odometry, 'odometry', self.odom, qos_profile_sensor_data)
            self.create_subscription(String, 'rmf_controller_state', lambda m: self.status('controller', m), 10)
            self.create_subscription(String, 'gazebo_state', lambda m: self.status('gazebo', m), 10)
            for topic, name in (('execution_status', 'execution'), ('map_updates', 'map')):
                self.create_subscription(String, topic, lambda m, n=name: self.status(n, m), durable)
            self.create_subscription(MultiDOFJointTrajectory, 'command/trajectory', self.trajectory, 10)
            self.create_subscription(Actuators, 'rmf/command/motor_speed', self.motors, 10)
            self.create_subscription(PointCloud2, 'points', self.scan, qos_profile_sensor_data)
            for topic in esdf_product_topics:
                self.create_subscription(PointCloud2, topic,
                    lambda message, name=topic: self.esdf_product(message, name), durable)
            for part in ('base', 'rotor_0', 'rotor_1', 'rotor_2', 'rotor_3'):
                self.create_subscription(Contacts, '/rmf/contact/'+part,
                    lambda message, name=part: self.contact(message, name), qos_profile_sensor_data)

        def record(self, value):
            return {'wall_s': time.monotonic()-started, 'sim_s': self.sim_s, 'phase': self.phase, **value}

        def clock(self, message):
            self.sim_s = message.clock.sec+message.clock.nanosec*1.e-9

        def position(self):
            p = self.current.pose.pose.position
            return [p.x, p.y, p.z]

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
            history['motor_commands'].append(self.record({'stamp_ns': stamp_ns(message), 'velocity': list(message.velocity)}))

        def scan(self, message):
            history['scans'].append(self.record(cloud_metadata(message, 'points')))

        def esdf_product(self, message, topic):
            history['esdf_products'].append(self.record({'topic': topic,
                'stamp_ns': stamp_ns(message), 'width': message.width}))

        def contact(self, message, part):
            history['contacts'].append(self.record({'stamp_ns': stamp_ns(message), 'part': part,
                'pairs': [[contact.collision1.name, contact.collision2.name] for contact in message.contacts]}))

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

        def feedback(self, message):
            value = message.feedback
            history['feedback'].append(self.record({'remaining_waypoints': value.remaining_waypoints,
                'dist_to_goal': value.dist_to_goal, 'estimated_time_remaining': value.estimated_time_remaining}))

        def tick(self):
            require(time.monotonic() < deadline, 'Fixed-path wall deadline')
            require(process.poll() is None, 'Launch exited during fixed-path run')
            rclpy.spin_once(self, timeout_sec=.02)
            require(self.values.get('controller', {}).get('flight', {}).get('state') != 'FAULT',
                    'Native flight fault: '+json.dumps(self.values.get('controller', {})))

        def until(self, condition, timeout, description):
            end = min(deadline, time.monotonic()+timeout)
            while not condition():
                self.tick()
                require(time.monotonic() < end, description+' timed out')

        def call(self, client, request, timeout=30.):
            self.until(client.service_is_ready, timeout, 'Service discovery')
            future = client.call_async(request)
            self.until(future.done, timeout, 'Service response')
            return future.result()

    try:
        command = ['ros2', 'launch', 'gbplanner_gazebo', 'rmf_fortress.launch.py',
            'mission:=false', 'planning:=true', 'mapping:=true', 'contacts:=true',
            'trajectory_profile:='+args.profile, f'model_metadata_file:={metadata_path}']
        result['launch_command'] = command
        if validation_directory is not None:
            command.append('validation_snapshot_directory:='+str(validation_directory))
        if args.trace_mapping:
            command.extend(['mapper_trace_file:='+str(mapper_trace_path),
                            'map_trace_file:='+str(planner_trace_path)])
            result['mapping_trace_files'] = [str(mapper_trace_path), str(planner_trace_path)]
        result['validation_snapshot_directory'] = str(validation_directory) if validation_directory else None
        log = log_path.open('x')
        process = subprocess.Popen(command, cwd=root, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init(args=[])
        initialized, node = True, Probe()
        node.until(lambda: node.current is not None and
            node.values.get('controller', {}).get('flight', {}).get('state') == 'GROUNDED', 45., 'Native ground rest')
        parameters = GetParameters.Request()
        parameters.names = ['trajectory_profile', 'speed', 'sample_period', 'endpoint_tolerance']
        values = node.call(node.parameters, parameters).values
        result['follower_parameters'] = {'trajectory_profile': values[0].string_value,
            'speed': values[1].double_value, 'sample_period': values[2].double_value,
            'endpoint_tolerance': values[3].double_value}
        require(values[0].string_value == args.profile, 'Requested trajectory profile is not active')
        require(values[1].double_value == 1. and values[2].double_value == .05 and
                values[3].double_value == .05, 'Original follower speed/period/endpoint tolerance changed')
        node.phase = 'takeoff'
        response = node.call(node.takeoff, Trigger.Request())
        result['takeoff_response'] = {'success': response.success, 'message': response.message}
        require(response.success, 'Native takeoff refused: '+response.message)
        node.until(lambda: node.values.get('controller', {}).get('flight', {}).get('state') == 'AIRBORNE',
                   60., 'Native takeoff completion')
        result['hover_position'] = node.position()
        result['mapper_after_takeoff'] = json.loads(node.call(node.mapper_client, Trigger.Request()).message)
        result['initial_source_distance_m'] = math.dist(node.position(), states[0][:3])
        require(result['initial_source_distance_m'] <= .05, 'Hover differs from recorded path start by more than .05 m')
        node.phase = 'preflight'
        observed_after_ns = stamp_ns(node.current)
        checked_revision, preflight_deadline = -1, min(deadline, time.monotonic()+60.)
        while True:
            def new_map():
                value = node.values.get('map', {})
                stamp = value.get('stamp_sec', 0)*1_000_000_000+value.get('stamp_nanosec', 0)
                return (value.get('known_cells', 0) > 0 and stamp >= observed_after_ns and
                        value.get('observation_revision', 0) > checked_revision)
            node.until(new_map, max(.01, preflight_deadline-time.monotonic()), 'Fresh native map')
            checked_revision = node.values['map']['observation_revision']
            request = ValidatePath.Request()
            request.header.frame_id = 'map'
            request.header.stamp = node.get_clock().now().to_msg()
            request.bound_mode = bound_mode
            request.first_pose_is_observation = True
            request.path = [node.current.pose.pose]+[pose(state) for state in states]
            checked = node.call(node.validator, request)
            result['preflight_checks'].append({'valid': checked.valid, 'reason': checked.reason,
                'map_revision': checked.map_revision, 'observation_revision': checked_revision,
                'failure_segment': checked.failure_segment})
            if checked.valid:
                break
            require(time.monotonic() < preflight_deadline, 'Original fixed path not free: '+checked.reason)
        node.until(node.action.server_is_ready, 30., 'Follower Action discovery')
        result['dispatch_source_distance_m'] = math.dist(node.position(), states[0][:3])
        require(result['dispatch_source_distance_m'] <= .05,
                'Hover drifted beyond .05 m from the retained source start before dispatch')
        node.phase = 'fixed_path'
        goal = PathFollower.Goal()
        goal.bound_mode, goal.manual_path, goal.vel_max = bound_mode, False, 0.
        for state in states:
            stamped = PoseStamped()
            stamped.header.frame_id = 'map'
            stamped.header.stamp = node.get_clock().now().to_msg()
            stamped.pose = pose(state)
            goal.path.append(stamped)
        future = node.action.send_goal_async(goal, feedback_callback=node.feedback)
        node.until(future.done, 30., 'Follower Action acceptance')
        node.goal_handle = future.result()
        result['action_accepted'] = node.goal_handle.accepted
        require(node.goal_handle.accepted, 'Follower rejected fixed path')
        future = node.goal_handle.get_result_async()
        printed_at = time.monotonic()
        while not future.done():
            node.tick()
            if time.monotonic()-printed_at >= 10.:
                print(json.dumps({'phase': node.phase, 'sim_s': node.sim_s, 'position': node.position(),
                    'execution': node.values.get('execution', {}).get('reason')}), flush=True)
                printed_at = time.monotonic()
        action_result = future.result()
        result['action_result'] = {'status': action_result.status, 'success': action_result.result.success,
                                   'sim_s': node.sim_s, 'position': node.position()}
        result['endpoint_error_m'] = math.dist(node.position(), states[-1][:3])
        result['execution_terminal'] = json.loads(node.call(node.state_client, Trigger.Request()).message)
        result['tracking'] = tracking_summary(states, history)
        require(action_result.status == GoalStatus.STATUS_SUCCEEDED and action_result.result.success,
                'Fixed-path Action failed: '+json.dumps(result['execution_terminal']))
        require(result['endpoint_error_m'] <= .05, 'Original endpoint tolerance not met')
        require(result['tracking']['all_source_vertices_retained'] and not result['tracking']['progress_failure'],
                'Fixed geometry was omitted or ordered native progress failed')
        require(not any(row['phase'] == 'fixed_path' and row['pairs'] for row in history['contacts']),
                'Native contact during fixed-path execution')
        result['checks'].update(fixed_path_completed=True, every_source_vertex_retained=True,
            ordered_progress=True, no_flight_contacts=True, real_map_validation=True)
        execution = result['execution_terminal']
        require(not execution['active'] and not execution['output_in_flight'], 'Follower still owns output')
        node.phase = 'landing'
        response = node.call(node.land, Trigger.Request())
        result['landing_response'] = {'success': response.success, 'message': response.message}
        require(response.success, 'Native landing refused: '+response.message)
        node.until(lambda: node.values.get('controller', {}).get('flight', {}).get('state') == 'LANDED',
                   90., 'Native touchdown')
        landed_at = node.sim_s
        node.until(lambda: node.sim_s >= landed_at+.5, 10., 'Post-touchdown observations')
        require(any(row['part'] == 'base' and row['pairs'] and row['sim_s'] > landed_at
                    for row in history['contacts']), 'No native base contact after landing')
        motors = [row for row in history['motor_commands'] if row['sim_s'] > landed_at+.1]
        require(motors and all(len(row['velocity']) == 4 and all(abs(v) <= 1.e-9 for v in row['velocity'])
                               for row in motors), 'Motors not zero after native touchdown')
        result['checks'].update(native_touchdown=True, zero_landed_motor_commands=True)
        result['mapper_after_landing'] = json.loads(node.call(node.mapper_client, Trigger.Request()).message)
        result['status'] = 'passed'
    except Exception as error:
        result['error'] = type(error).__name__+': '+str(error)
        # Diagnose only the already failed preflight. The original request,
        # zero clearance and production validity decision remain unchanged.
        # A separate callback snapshot is recorded as such, never substituted
        # for an earlier map revision or used to permit this failed run to fly.
        if args.capture_validation and node is not None and node.phase == 'preflight':
            try:
                client = node.create_client(SetParameters, '/gbplanner_local/set_parameters')
                arm = SetParameters.Request()
                arm.parameters = [Parameter('diagnostics.capture_next_validation_rejection', value=True).to_parameter_msg()]
                armed = node.call(client, arm)
                require(armed.results and all(item.successful for item in armed.results), 'Diagnostic arming refused')
                request = ValidatePath.Request()
                request.header.frame_id = 'map'
                request.header.stamp = node.get_clock().now().to_msg()
                request.bound_mode = bound_mode
                request.first_pose_is_observation = True
                request.path = [node.current.pose.pose]+[pose(state) for state in states]
                captured = node.call(node.validator, request)
                result['diagnostic_recheck'] = {'valid': captured.valid, 'reason': captured.reason,
                    'map_revision': captured.map_revision, 'clearance_margin': request.clearance_margin,
                    'failure_segment': captured.failure_segment,
                    'independent_callback_after_preflight_failure': True}
            except Exception as capture_error:
                result['diagnostic_capture_error'] = type(capture_error).__name__+': '+str(capture_error)
    finally:
        if node is not None:
            result['final'] = node.values
            result['last_position'] = node.position() if node.current is not None else None
        if 'tracking' not in result:
            try:
                result['tracking'] = tracking_summary(states, history)
            except Exception as error:
                result['tracking'] = {'available': False, 'error': type(error).__name__+': '+str(error)}
        # Stop this simulation group. Never disarm a flying vehicle or insert
        # a probe-owned hold that could conceal the production stop outcome.
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
        result['bridge_counts'] = [{'topic': topic, 'received': int(received), 'transferred': int(transferred), 'publish_failures': int(errors)}
            for topic, received, transferred, errors in re.findall(
                r'Bridge drained: GZ->ROS (\S+) received=(\d+) transferred=(\d+) publish_failures=(\d+)', text)]
        result['signal_escalations'] = [line for line in text.splitlines() if 'escalating to' in line]
        result['callback_errors'] = [line for line in text.splitlines()
                                    if 'exception was never retrieved' in line or 'Exception ignored' in line]
        result['checks']['clean_shutdown'] = (not failures and not result['signal_escalations'] and
            not result['callback_errors'] and not result['cleanup'].get('forced_kill', False))
        if initialized:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        result['source_sha256_at_end'] = {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
                                         for path in sources}
        result['checks']['sources_unchanged_during_run'] = result['source_sha256_at_end'] == hashes
        if not result['checks']['clean_shutdown'] or not result['checks']['sources_unchanged_during_run']:
            result['status'] = 'failed'
        result['counts'] = {name: len(rows) for name, rows in history.items()}
        result['esdf_product_counts'] = {topic: sum(row['topic'] == topic for row in history['esdf_products'])
                                         for topic in esdf_product_topics}
        result['history_file'] = str(history_path)
        result['wall_elapsed_s'] = time.monotonic()-started
        history_path.write_text(json.dumps(history, separators=(',', ':'), allow_nan=False)+'\n')
        output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        print(json.dumps({key: result.get(key) for key in
            ('status', 'error', 'profile', 'endpoint_error_m', 'tracking', 'cleanup', 'wall_elapsed_s')}, indent=2))
    return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
