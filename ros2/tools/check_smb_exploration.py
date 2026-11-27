#!/usr/bin/env python3
"""Check live SMB/TSDF/PCI tracking after explicit physical teleop mapping."""
import argparse
import copy
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def validate_homing_record(record, graph_home):
    """Check a ground return against its persistent XY home and current terrain.

    TerrainProjector reprojects the route's Z against the current TSDF. The
    graph's stored home is immutable, but its old height is not an exact output
    constraint after new scans. This function also supports recorded-run review.
    """
    endpoint = record['commanded_path']['path'][-1]
    record['commanded_home_xy_error_m'] = math.dist(endpoint[:2], record['home_pose'][:2])
    record['commanded_home_z_update_m'] = endpoint[2] - record['home_pose'][2]
    require(graph_home['home_unchanged'], 'Persistent home vertex changed')
    require(record['native_validation']['valid'], 'Returned terrain route is not valid')
    require(record['commanded_home_xy_error_m'] <= 1.e-6, 'Return route changed home XY')
    require(record['planar_home_error_m'] <= .04, 'Actual return exceeded XY tolerance')
    require(abs(record['yaw_error_rad']) <= .1, 'Actual return exceeded route-heading tolerance')
    require(record['home_error_m'] <= .1, 'Actual return exceeded original 3D tolerance')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--explore', action='store_true',
                      help='After the search goal, execute one local-gain step and return home')
    mode.add_argument('--auto-cycles', type=int, metavar='N',
                      help='Run one automatic PCI mission for N local cycles, then native homing')
    parser.add_argument('--auto-action-budget', type=float, metavar='SECONDS',
                        help='Observation budget per automatic action, including planning and new-map wait '
                             '(default: 45); requires --auto-cycles and does not change ROS controller timeouts')
    parser.add_argument('--tsdf-projection-mode', choices=('upstream_step', 'zero_crossing', 'zero_crossing_simplex'),
                        help='Set this ground projection mode in the generated automatic-mission configuration')
    parser.add_argument('--terminal-integral-gain', type=float,
                        help='Explicit nonnegative terminal velocity integral gain for an automatic mission')
    parser.add_argument('--tsdf-integration-order', choices=('mixed', 'sorted'),
                        help='Explicit mapper order, e.g. sorted to retain the earlier experiment configuration')
    args = parser.parse_args()
    if args.auto_cycles is not None and args.auto_cycles < 1:
        parser.error('--auto-cycles must be a positive integer')
    automatic = args.auto_cycles is not None
    if args.auto_action_budget is not None:
        if not automatic:
            parser.error('--auto-action-budget requires --auto-cycles')
        if not math.isfinite(args.auto_action_budget) or args.auto_action_budget <= 0.:
            parser.error('--auto-action-budget must be finite and positive')
    auto_action_budget = 45. if args.auto_action_budget is None else args.auto_action_budget
    if args.tsdf_projection_mode is not None and not automatic:
        parser.error('--tsdf-projection-mode requires --auto-cycles')
    if args.tsdf_integration_order is not None and not automatic:
        parser.error('--tsdf-integration-order requires --auto-cycles')
    if args.terminal_integral_gain is not None:
        if not automatic:
            parser.error('--terminal-integral-gain requires --auto-cycles')
        if not math.isfinite(args.terminal_integral_gain) or args.terminal_integral_gain < 0.:
            parser.error('--terminal-integral-gain must be finite and nonnegative')
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))

    import rclpy
    from geometry_msgs.msg import Pose, PoseStamped, Twist
    from nav_msgs.msg import Odometry, Path as RosPath
    from planner_msgs.msg import Graph
    from planner_msgs.srv import PciHomingTrigger, PciSearch, PciTrigger, PlannerSearch, ValidatePath
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.srv import GetParameters
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    started = time.monotonic()
    # This bounds the observer only. Native control timeouts and success
    # tolerances remain defined by the ROS configuration.
    wall_budget = 90.+auto_action_budget*(args.auto_cycles+1) if automatic else 180. if args.explore else 90.
    deadline = started + wall_budget - 6.  # Leave time for the launch process tree.
    process, node = None, None
    log_path = args.output.with_suffix('.log')
    trace_path = args.output.with_suffix('.csv')
    commands_path = args.output.with_name(args.output.stem + '_commands.csv')
    history_path = args.output.with_name(args.output.stem + '_history.json')
    summary = {
        'status': 'failed', 'domain': os.environ['ROS_DOMAIN_ID'],
        'scope': 'live Bullet SMB → sparse Voxblox TSDF → ground planner → PCI → '
                 'PathFollower Action → pure pursuit → wheel contact dynamics',
        'initialization_scope': 'explicit teleop-style mapping in the obstacle-free starting '
                                'area; this motion is not autonomous exploration',
        'explore_enabled': args.explore, 'auto_cycles': args.auto_cycles,
        'auto_action_budget_s': auto_action_budget if automatic else None,
        'wall_budget_s': wall_budget,
        'checks': {},
    }

    def pose(x, y, z=.24, yaw=0.):
        value = Pose()
        value.position.x, value.position.y, value.position.z = float(x), float(y), float(z)
        value.orientation.z, value.orientation.w = math.sin(yaw/2.), math.cos(yaw/2.)
        return value

    def record_pose(value):
        p, q = value.position, value.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return [p.x, p.y, p.z, yaw]

    class Probe(Node):
        def __init__(self):
            super().__init__('smb_exploration_probe')
            self.phase = 'discovery'
            self.current = None
            self.odom_at, self.speed, self.yaw_rate = 0., math.inf, math.inf
            self.odom_stamp = 0.
            self.odom_timeout = .5
            self.max_odom_receive_gap = self.max_odom_source_gap = self.max_odom_source_age = 0.
            self.odom_timing_events, self.slow_spins = [], []
            self.spin_calls, self.last_spin_end = 0, None
            self.max_spin_wall, self.max_spin_cpu = 0., 0.
            self.odom_message_info_samples = {'source_timestamp': 0, 'received_timestamp': 0}
            self.last_map = None
            # Keep a bounded window so a failed plan can save the revision it
            # actually queried, rather than the next scan received on exit.
            self.map_snapshots = []
            self.failure_metric, self.failure_snapshot, self.failure_graph = None, None, None
            self.execution_validation_failure, self.execution_failure_snapshot = None, None
            self.execution_failure_candidates = []
            self.smb, self.control, self.execution, self.tracking, self.map_update = {}, {}, {}, {}, {}
            self.trace, self.commands, self.maps, self.clouds = [], [], [], []
            self.smb_history, self.map_history, self.search_metrics = [], [], []
            self.control_history, self.execution_history, self.tracking_history = [], [], []
            self.search_paths, self.commanded_paths = [], []
            self.planning_metrics, self.global_metrics, self.local_paths, self.global_paths = [], [], [], []
            self.global_graph, self.global_graph_history = None, []
            self.last_commanded_message = None
            self.last_command = None
            self.auto_record, self.auto_validation_futures = None, []
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Odometry, 'odometry', self.odometry, qos_profile_sensor_data)
            self.create_subscription(Twist, 'cmd_vel', self.velocity, 10)
            self.create_subscription(String, 'smb_status', self.vehicle, 10)
            self.create_subscription(PointCloud2, 'points', self.cloud, qos_profile_sensor_data)
            self.create_subscription(PointCloud2, 'accepted_voxel_map', self.snapshot, durable)
            self.create_subscription(String, 'map_updates', self.mapping, durable)
            self.create_subscription(String, 'search_metrics',
                                     lambda m: self.search_metrics.append(json.loads(m.data)), durable)
            self.create_subscription(RosPath, 'search_path',
                                     lambda m: self.path(m, self.search_paths), durable)
            self.create_subscription(RosPath, 'command_path',
                                     lambda m: self.path(m, self.commanded_paths), durable)
            for topic, attr, history in [('control_status', 'control', self.control_history),
                                         ('execution_status', 'execution', self.execution_history),
                                         ('tracking_status', 'tracking', self.tracking_history)]:
                self.create_subscription(String, topic,
                    lambda m, a=attr, h=history: self.status(m, a, h), durable)
            self.teleop = self.create_publisher(Twist, 'joy_teleop/cmd_vel', 10)
            self.target = self.create_publisher(PoseStamped, 'search_target', durable)
            self.search = self.create_client(PlannerSearch, 'planner_search')
            self.validate = self.create_client(ValidatePath, 'validate_path')
            self.pci = self.create_client(PciSearch, 'pci_search')
            if args.explore or automatic:
                self.planner_parameters = self.create_client(GetParameters, 'gbplanner_local/get_parameters')
                self.control_parameters = self.create_client(GetParameters, 'gbplanner_control/get_parameters')
                self.local_trigger = self.create_client(PciTrigger, 'pci_trigger')
                self.homing_trigger = self.create_client(PciHomingTrigger, 'pci_homing_trigger')
                self.create_subscription(Graph, 'global_graph', self.graph, durable)
                for topic, history in [('planning_metrics', self.planning_metrics),
                                       ('global_metrics', self.global_metrics)]:
                    self.create_subscription(String, topic,
                        lambda message, destination=history: self.metric(message, destination), durable)
                self.create_subscription(RosPath, 'local_path',
                                         lambda message: self.path(message, self.local_paths), durable)
                self.create_subscription(RosPath, 'global_path',
                                         lambda message: self.path(message, self.global_paths), durable)
            self.states = {name: self.create_client(Trigger, name) for name in
                           ('smb_state', 'mapper_status', 'map_status', 'control_state',
                            'execution_state', 'tracking_state', 'velocity_mux_state')}

        def status(self, message, attribute, history):
            value = json.loads(message.data)
            setattr(self, attribute, value)
            sample = {'time_s': time.monotonic()-started, 'phase': self.phase, **value}
            if self.auto_record is not None:
                sample['observed_pose'] = record_pose(self.current) if self.current else None
                sample['observed_odom_stamp_s'] = self.odom_stamp
                sample['observed_odom_fresh'] = self.fresh_odometry()
            history.append(sample)
            if (attribute == 'execution' and value.get('state') in ('STOPPING', 'ERROR')
                    and value.get('validation_failure')):
                if self.execution_validation_failure is None:
                    # This revision comes from the rejected ValidatePath
                    # response, not the execution status's latest map field.
                    self.execution_validation_failure = copy.deepcopy(value['validation_failure'])
                    # Retain the current bounded window while its matching
                    # map_updates stamp may still be in flight.
                    self.execution_failure_candidates = list(self.map_snapshots)
                self.pin_failed_input()

        def metric(self, message, destination):
            value = {'time_s': time.monotonic()-started, 'phase': self.phase,
                     **json.loads(message.data)}
            destination.append(value)
            if destination is self.planning_metrics:
                # Pin before diagnostic service calls spin and evict the small
                # map ring. Matching is retried when either topic arrives later.
                self.failure_metric = value if not value['success'] else None
                self.failure_snapshot = None
                self.failure_graph = self.global_graph if self.failure_metric else None
                self.pin_failed_input()

        def pin_failed_input(self):
            for failure, attribute, candidates in (
                    (self.failure_metric, 'failure_snapshot', self.map_snapshots),
                    (self.execution_validation_failure, 'execution_failure_snapshot',
                     self.execution_failure_candidates+self.map_snapshots)):
                if failure is None or getattr(self, attribute) is not None:
                    continue
                update = next((entry for entry in reversed(self.map_history)
                               if entry['map_revision'] == failure['map_revision']), None)
                if update is not None:
                    snapshot = next((entry for entry in reversed(candidates)
                        if entry.header.stamp.sec == update['stamp_sec']
                        and entry.header.stamp.nanosec == update['stamp_nanosec']), None)
                    setattr(self, attribute, snapshot)
                    if attribute == 'execution_failure_snapshot' and snapshot is not None:
                        # Keep the exact message independently of ring eviction.
                        self.execution_failure_candidates = []

        def odometry(self, message):
            callback_entered = time.monotonic()
            callback_ros_ns = self.get_clock().now().nanoseconds
            require(message.header.frame_id == 'map' and message.child_frame_id == 'base_link',
                    'Unexpected Bullet odometry frame')
            self.current = copy.deepcopy(message.pose.pose)
            value = record_pose(self.current)
            v, w = message.twist.twist.linear, message.twist.twist.angular
            require(all(math.isfinite(x) for x in [*value, v.x, v.y, v.z, w.z]),
                    'Non-finite odometry')
            self.speed = math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
            received = time.monotonic()
            stamp = message.header.stamp.sec+message.header.stamp.nanosec*1.e-9
            gap = received-self.odom_at if self.odom_at else 0.
            source_gap = stamp-self.odom_stamp if self.odom_stamp else 0.
            source_age = self.get_clock().now().nanoseconds*1.e-9-stamp
            self.max_odom_receive_gap = max(self.max_odom_receive_gap, gap)
            self.max_odom_source_gap = max(self.max_odom_source_gap, source_gap)
            self.max_odom_source_age = max(self.max_odom_source_age, source_age)
            self.yaw_rate, self.odom_at, self.odom_stamp = w.z, received, stamp
            self.trace.append((received-started, self.phase, *value, self.speed, w.z,
                               stamp, source_age, gap))
            # The installed Humble Executor._take_subscription returns only
            # msg_info[0]; its public subscription callback receives one
            # message, not MessageInfo. Keep unavailable RMW timestamps null
            # rather than replacing the executor or changing delivery.
            middleware = {'source_timestamp': None, 'received_timestamp': None}
            if source_age >= .1 or gap >= .1:
                self.odom_timing_events.append({
                    'time_s': received-started, 'phase': self.phase,
                    'sample': len(self.trace), 'spin_call': self.spin_calls,
                    'callback_entry_time_s': callback_entered-started,
                    'callback_entry_ros_ns': callback_ros_ns,
                    'callback_before_trace_wall_s': received-callback_entered,
                    'header_stamp_ns': message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec,
                    'source_age_s': source_age, 'receive_gap_s': gap, 'source_stamp_gap_s': source_gap,
                    'rmw_source_timestamp_ns': middleware['source_timestamp'],
                    'rmw_received_timestamp_ns': middleware['received_timestamp'],
                    'rmw_receive_to_callback_s': (
                        (callback_ros_ns-middleware['received_timestamp'])*1.e-9
                        if middleware['received_timestamp'] is not None else None),
                })
                del self.odom_timing_events[:-8]

        def velocity(self, message):
            values = [message.linear.x, message.linear.y, message.linear.z,
                      message.angular.x, message.angular.y, message.angular.z]
            require(all(math.isfinite(v) for v in values), 'Non-finite cmd_vel')
            require(abs(values[0]) <= .500001 and abs(values[5]) <= .500001,
                    f'cmd_vel exceeds tracker limits: {values}')
            require(all(abs(values[i]) < 1.e-10 for i in (1, 2, 3, 4)), 'Non-planar cmd_vel')
            self.last_command = (values[0], values[5])
            self.commands.append((time.monotonic()-started, self.phase, *self.last_command))

        def vehicle(self, message):
            value = json.loads(message.data)
            self.smb = value
            self.smb_history.append({'time_s': time.monotonic()-started, 'phase': self.phase, **value})
            require(value['physics'] == 'Bullet', value)
            require(value['chassis_contact_steps'] == 0 and value['chassis_contacts'] == 0,
                    f'Chassis contacted geometry: {value}')

        def cloud(self, message):
            self.clouds.append({'time_s': time.monotonic()-started, 'phase': self.phase,
                                'frame': message.header.frame_id,
                                'points': int(message.width*message.height)})

        def snapshot(self, message):
            self.last_map = message
            self.map_snapshots.append(message)
            if len(self.map_snapshots) > 8:
                self.map_snapshots.pop(0)
            self.maps.append({'time_s': time.monotonic()-started, 'phase': self.phase,
                              'frame': message.header.frame_id,
                              'cells': int(message.width*message.height),
                              'stamp_s': message.header.stamp.sec+message.header.stamp.nanosec*1.e-9,
                              'fields': [field.name for field in message.fields]})
            self.pin_failed_input()

        def mapping(self, message):
            self.map_update = json.loads(message.data)
            self.map_history.append({'time_s': time.monotonic()-started, 'phase': self.phase,
                                     **self.map_update})
            self.pin_failed_input()

        def graph(self, message):
            require(message.header.frame_id == 'map', 'Unexpected global graph frame')
            self.global_graph = message
            home = next((vertex for vertex in message.vertices if vertex.id == 0), None)
            self.global_graph_history.append({
                'time_s': time.monotonic()-started, 'phase': self.phase,
                'frame': message.header.frame_id,
                'stamp_s': message.header.stamp.sec+message.header.stamp.nanosec*1.e-9,
                'vertices': len(message.vertices), 'edges': len(message.edges),
                'home_planning_pose': record_pose(home.pose) if home is not None else None,
            })

        def path(self, message, destination):
            if destination is self.commanded_paths:
                self.last_commanded_message = message
            entry = {'time_s': time.monotonic()-started, 'phase': self.phase,
                     'frame': message.header.frame_id,
                     'path': [record_pose(p.pose) for p in message.poses]}
            destination.append(entry)
            if self.auto_record is not None and destination is self.commanded_paths and message.poses:
                action = {'index': len(self.auto_record['actions'])+1, 'commanded_path': entry,
                          'start': record_pose(self.current), 'native_validation': None}
                self.auto_record['actions'].append(action)
                # Validate each actual dispatched route without pausing or
                # replacing the PCI's own stop-observe-replan sequence.
                check = ValidatePath.Request()
                check.header.frame_id = 'map'
                check.header.stamp = self.get_clock().now().to_msg()
                check.bound_mode, check.path = 3, [copy.deepcopy(p.pose) for p in message.poses]
                # In this nonmanual PCI mission command_path now includes the
                # controller's measured preflight connector. Match its native
                # contract; all subsequent planner vertices remain fixed.
                check.first_pose_is_observation = True
                future = self.validate.call_async(check)
                self.auto_validation_futures.append(future)

                def validated(done, item=action):
                    try:
                        response = done.result()
                        item['native_validation'] = {'valid': response.valid, 'reason': response.reason,
                                                     'map_revision': response.map_revision}
                    except Exception as error:
                        item['native_validation'] = {'valid': False, 'reason': str(error)}
                future.add_done_callback(validated)

        def spin_once(self):
            entered = time.monotonic()
            entered_ros_ns = self.get_clock().now().nanoseconds
            cpu_started = time.thread_time()
            entry_gap = entered-self.last_spin_end if self.last_spin_end is not None else None
            self.spin_calls += 1
            sequence, odometry_before = self.spin_calls, len(self.trace)
            try:
                rclpy.spin_once(self, timeout_sec=.01)
            finally:
                exited, cpu = time.monotonic(), time.thread_time()-cpu_started
                wall = exited-entered
                self.last_spin_end = exited
                self.max_spin_wall, self.max_spin_cpu = max(self.max_spin_wall, wall), max(self.max_spin_cpu, cpu)
                if wall >= .05 or (entry_gap is not None and entry_gap >= .1):
                    self.slow_spins.append({
                        'spin_call': sequence, 'phase': self.phase,
                        'time_s': entered-started, 'end_time_s': exited-started,
                        'entry_ros_ns': entered_ros_ns,
                        'exit_ros_ns': self.get_clock().now().nanoseconds,
                        'wall_s': wall, 'thread_cpu_s': cpu, 'entry_gap_s': entry_gap,
                        'odometry_samples_before': odometry_before,
                        'odometry_samples_after': len(self.trace),
                    })
                    del self.slow_spins[:-8]

        def until(self, predicate, label, seconds=5.):
            end = min(time.monotonic()+seconds, deadline)
            while not predicate():
                if process is not None and process.poll() is not None:
                    raise RuntimeError(f'Exploration launch exited {process.returncode}; see {log_path}')
                if time.monotonic() > end:
                    raise TimeoutError(f'{self.phase}: {label}; control={self.control}; '
                                       f'execution={self.execution}; map={self.map_update}')
                self.spin_once()

        def spin_for(self, seconds):
            end = time.monotonic()+seconds
            self.until(lambda: time.monotonic() >= end, 'Observation interval', seconds+.25)

        def call(self, client, request, seconds=8.):
            self.until(client.service_is_ready, 'Service discovery', seconds)
            future = client.call_async(request)
            self.until(future.done, 'Service response', seconds)
            return future.result()

        def state(self, name, seconds=3.):
            response = self.call(self.states[name], Trigger.Request(), seconds)
            return {'service_success': response.success, **json.loads(response.message)}

        def fresh_odometry(self):
            source_age = self.get_clock().now().nanoseconds*1.e-9-self.odom_stamp
            return (self.current is not None and time.monotonic()-self.odom_at <= self.odom_timeout
                    and self.odom_stamp > 0. and -.1 <= source_age <= self.odom_timeout)

        def stationary(self):
            return (self.fresh_odometry()
                    and self.speed < .03 and abs(self.yaw_rate) < .05
                    and self.smb and max(map(abs, self.smb['wheel_target'])) < 1.e-6)

        def stop_teleop(self):
            self.teleop.publish(Twist())

        def settle(self):
            self.until(self.stationary, 'Physical stop', 3.)
            before = record_pose(self.current)
            self.spin_for(.25)
            require(self.stationary() and math.dist(before[:3], record_pose(self.current)[:3]) < .015,
                    'Vehicle did not remain stationary')

        def mapping_leg(self, distance):
            require(self.control.get('state') in {'IDLE', 'STOPPED'} and not self.tracking.get('active', False),
                    'Teleop initialization requires idle PCI and tracker')
            previous_stamp = self.odom_stamp
            self.until(lambda: self.odom_stamp > previous_stamp and self.fresh_odometry(),
                       'Fresh odometry before manual mapping leg', 3.)
            start = record_pose(self.current)
            direction = 1. if distance > 0 else -1.
            c, s = math.cos(start[3]), math.sin(start[3])
            def traveled():
                point = record_pose(self.current)
                return direction*((point[0]-start[0])*c+(point[1]-start[1])*s)
            leg_end = min(time.monotonic()+14., deadline)
            command = Twist()
            command.linear.x = direction*.25
            try:
                while traveled() < abs(distance):
                    require(time.monotonic() < leg_end, 'Manual mapping leg timed out')
                    require(self.fresh_odometry(),
                            'Manual mapping lost fresh odometry: '
                            f'receive_age={time.monotonic()-self.odom_at:.3f}s, '
                            f'source_age={self.get_clock().now().nanoseconds*1.e-9-self.odom_stamp:.3f}s, '
                            f'timeout={self.odom_timeout:.3f}s')
                    self.teleop.publish(command)
                    self.spin_for(.04)
            finally:
                self.stop_teleop()
            self.settle()
            return {'requested_signed_distance_m': distance, 'speed_command_mps': direction*.25,
                    'start': start, 'end': record_pose(self.current),
                    'observed_longitudinal_distance_m': traveled()}

        def validate_path(self, values):
            request = ValidatePath.Request()
            request.header.frame_id = 'map'
            request.header.stamp = self.get_clock().now().to_msg()
            request.bound_mode, request.path = 3, values
            response = self.call(self.validate, request)
            return {'valid': response.valid, 'reason': response.reason,
                    'map_revision': response.map_revision}

        def observe_home(self):
            self.phase = 'observe_graph_home'
            self.until(lambda: self.global_graph is not None and
                       any(vertex.id == 0 for vertex in self.global_graph.vertices),
                       'Published global graph home')
            graph = self.global_graph
            home_vertex = next(vertex for vertex in graph.vertices if vertex.id == 0)
            parameter_request = GetParameters.Request()
            parameter_request.names = ['robot.type', 'ground.robot_height', 'ground.max_ground_height']
            parameters = self.call(self.planner_parameters, parameter_request).values
            require(len(parameters) == 3 and parameters[0].type == ParameterType.PARAMETER_STRING
                    and parameters[0].string_value == 'kGround'
                    and all(value.type == ParameterType.PARAMETER_DOUBLE
                            and math.isfinite(value.double_value) and value.double_value > 0.
                            for value in parameters[1:]), 'Missing ground-planner height parameters')
            heights = dict(zip(parameter_request.names[1:],
                               (value.double_value for value in parameters[1:])))
            base_z_offset = heights['ground.robot_height']-heights['ground.max_ground_height']
            home_pose = copy.deepcopy(home_vertex.pose)
            home_pose.position.z += base_z_offset
            record = {
                'source': 'global_graph vertex 0', 'vertex_id': 0, 'frame': graph.header.frame_id,
                'graph_stamp_s': graph.header.stamp.sec+graph.header.stamp.nanosec*1.e-9,
                'graph_vertices': len(graph.vertices), 'graph_edges': len(graph.edges),
                'planning_pose': record_pose(home_vertex.pose), 'base_pose': record_pose(home_pose),
                'height_parameters': heights, 'planning_to_base_z_offset_m': base_z_offset,
                'observed_current_pose': record_pose(self.current),
                'native_validation': self.validate_path([home_pose]),
                'map_revision': self.map_update['map_revision'],
            }
            summary['checks']['graph_home'] = record
            require(record['native_validation']['valid'], record)
            return record

        def finish_homing(self, record, graph_home):
            record.update(home_pose=graph_home['base_pose'], home_planning_pose=graph_home['planning_pose'],
                          home_vertex_id=0)
            home = record['home_pose']
            record['home_error_m'] = math.dist(record['end'][:3], home[:3])
            record['planar_home_error_m'] = math.dist(record['end'][:2], home[:2])
            record['vertical_home_error_m'] = abs(record['end'][2]-home[2])
            record['commanded_home_error_m'] = math.dist(record['commanded_path']['path'][-1][:3], home[:3])
            final_home = next((vertex for vertex in self.global_graph.vertices if vertex.id == 0), None)
            require(final_home is not None, 'Global graph lost its original home')
            graph_home['planning_pose_after_homing'] = record_pose(final_home.pose)
            original = graph_home['planning_pose']
            observed = [entry['home_planning_pose'] for entry in self.global_graph_history
                        if entry['stamp_s'] >= graph_home['graph_stamp_s']]
            observed.append(graph_home['planning_pose_after_homing'])
            graph_home['home_unchanged'] = all(value is not None
                and math.dist(original[:3], value[:3]) <= 1.e-9
                and abs(wrap(original[3]-value[3])) <= 1.e-9 for value in observed)
            validate_homing_record(record, graph_home)

        def collect_auto_actions(self):
            """Keep partial evidence useful even if the native policy stops early."""
            record = self.auto_record
            indices, before = record['history_start'], record['before']
            control = self.control_history[indices['control']:]
            execution = self.execution_history[indices['execution']:]
            tracking = self.tracking_history[indices['tracking']:]
            record['control_result'] = copy.deepcopy(self.control)
            record['observed_completed_actions'] = self.control.get('completed_actions', 0)
            record['observed_action_counter_increases'] = {
                name: {counter: observed.get(counter, 0)-before[name].get(counter, 0)
                       for counter in ('accepted', 'succeeded', 'cancelled', 'aborted')}
                for name, observed in [('execution_state', self.execution), ('tracking_state', self.tracking)]}
            previous_time = record['started_at_s']
            for item in record['actions']:
                index = item['index']
                for key, events, state_name in [('tracking_success', tracking, 'SUCCEEDED'),
                                                 ('execution_success', execution, 'DONE')]:
                    baseline = before['tracking_state' if key.startswith('tracking') else 'execution_state']
                    found = next((event for event in events if event.get('state') == state_name
                                  and event.get('succeeded') == baseline['succeeded']+index), None)
                    if found is not None:
                        item[key] = found
                completed = next((event for event in control if event['state'] == 'WAITING_OBSERVATION'
                                  and event['completed_actions'] == index), None)
                planning = [event for event in control if event['state'] == 'PLANNING'
                            and event['completed_actions'] == index-1]
                item['planning_events'] = planning
                if completed is not None:
                    item['control_completion'] = completed
                    item['mode'] = completed['mode']
                end_time = completed['time_s'] if completed else time.monotonic()-started
                item['planning_metrics'] = [entry for entry in self.planning_metrics[indices['local_metrics']:]
                                            if previous_time <= entry['time_s'] <= end_time]
                item['global_metrics'] = [entry for entry in self.global_metrics[indices['global_metrics']:]
                                          if previous_time <= entry['time_s'] <= end_time]
                if completed is not None:
                    previous_time = end_time
                if item.get('tracking_success') is not None:
                    observed = item['tracking_success'].get('observed_pose')
                    if observed is not None:
                        endpoint = item['commanded_path']['path'][-1]
                        item['pose_received_at_tracking_success'] = observed
                        item['received_pose_xy_error_m'] = math.dist(observed[:2], endpoint[:2])
                        item['received_pose_yaw_error_rad'] = abs(wrap(observed[3]-endpoint[3]))
            record['completed_local_actions'] = sum(
                item.get('mode') == 'local' and 'execution_success' in item and 'tracking_success' in item
                for item in record['actions'])
            record['completed_homing_actions'] = sum(
                item.get('mode') == 'homing' and 'execution_success' in item and 'tracking_success' in item
                for item in record['actions'])

        def automatic_policy(self, record, graph_home):
            """Issue exactly one auto trigger; every subsequent request belongs to PCI."""
            self.phase = 'automatic_policy'
            parameters = GetParameters.Request()
            parameters.names = ['max_cycles', 'planner_trigger_lead_time']
            values = self.call(self.control_parameters, parameters).values
            require(len(values) == 2 and values[0].type == ParameterType.PARAMETER_INTEGER
                    and values[0].integer_value == args.auto_cycles
                    and values[1].type == ParameterType.PARAMETER_DOUBLE and values[1].double_value == 0.,
                    'Automatic check requires the requested max_cycles and native stop-observe mode')
            record.update(requested_cycles=args.auto_cycles, set_auto=True, trigger_requests=0,
                          bound_mode=3, velocity_mps=.3, per_action_budget_s=auto_action_budget, actions=[],
                          start=record_pose(self.current), started_at_s=time.monotonic()-started,
                          runtime_parameters={'max_cycles': values[0].integer_value,
                                              'planner_trigger_lead_time': values[1].double_value},
                          map_revision_before=self.map_update['map_revision'],
                          known_cells_before=self.map_update['known_cells'],
                          before={name: self.state(name) for name in ('execution_state', 'tracking_state')},
                          history_start={'control': len(self.control_history), 'execution': len(self.execution_history),
                                         'tracking': len(self.tracking_history),
                                         'local_metrics': len(self.planning_metrics),
                                         'global_metrics': len(self.global_metrics)})
            self.auto_record = record
            try:
                request = PciTrigger.Request()
                request.not_exe_path, request.set_auto = False, True
                request.bound_mode, request.vel_max = 3, .3
                record['trigger_requests'] += 1
                response = self.call(self.local_trigger, request)
                record['request_accepted'] = response.success
                require(response.success, f'Automatic PCI request rejected: {self.control}')
                # Do not call local/homing again or hold the robot between
                # cycles. Observe the actual native controller transitions.
                while True:
                    require(time.monotonic() < deadline, 'Automatic mission exceeded wall budget')
                    require(process.poll() is None, 'Exploration launch exited during automatic mission')
                    require(self.fresh_odometry(), 'Automatic mission lost fresh odometry')
                    self.spin_once()
                    self.collect_auto_actions()
                    for item in record['actions']:
                        validation = item.get('native_validation')
                        require(validation is None or validation['valid'],
                                f'Automatic dispatched route failed native validation: {item}')
                    events = self.control_history[record['history_start']['control']:]
                    if events and self.control['state'] in {'DONE', 'ERROR', 'STOPPED'}:
                        break
                    completions = [event for event in events if event['state'] == 'WAITING_OBSERVATION']
                    stage_start = completions[-1]['time_s'] if completions else record['started_at_s']
                    require(time.monotonic()-started-stage_start <= auto_action_budget,
                            f'Automatic action/observation exceeded {auto_action_budget:g} s: {self.control}')
                record['control_result'] = copy.deepcopy(self.control)
                require(self.control['state'] == 'DONE' and self.control['mode'] == 'homing'
                        and self.control['reason'] == 'returned_home' and not self.control['auto'],
                        f'Automatic mission did not end with native homing: {self.control}')
                self.settle()
                self.until(lambda: all(item['native_validation'] is not None for item in record['actions']),
                           'Automatic route validation results', 5.)
                record['end'] = record_pose(self.current)
                record['final_states'] = {name: self.state(name) for name in self.states}
                record['map_revision_after'] = self.map_update['map_revision']
                record['known_cells_after'] = self.map_update['known_cells']
                self.collect_auto_actions()
                self.review_auto_actions(record, graph_home)
            finally:
                self.collect_auto_actions()
                record['elapsed_s'] = time.monotonic()-started-record['started_at_s']

        def review_auto_actions(self, record, graph_home):
            actions, before = record['actions'], record['before']
            require(record['completed_local_actions'] == args.auto_cycles,
                    f"Requested {args.auto_cycles} local cycles, observed {record['completed_local_actions']}; "
                    f"terminal={record['control_result']}")
            require(len(actions) == args.auto_cycles+1 and record['completed_homing_actions'] == 1,
                    'Automatic mission did not execute exactly N local actions and one homing action')
            previous, goal_ids, previous_gain_revision = None, set(), -1
            for item in actions:
                index = item['index']
                require(item.get('mode') == ('homing' if index == len(actions) else 'local'),
                        'Unexpected native action mode: '+str(item.get('mode')))
                require((item.get('native_validation') or {}).get('valid'), 'Missing valid native route check')
                require(len(item['commanded_path']['path']) >= 2, 'Automatic action lacks a motion segment')
                tracking, execution = item.get('tracking_success'), item.get('execution_success')
                require(tracking is not None and execution is not None, 'Missing per-action success evidence')
                require(tracking['accepted'] == before['tracking_state']['accepted']+index
                        and not tracking['active'] and tracking['position_tolerance'] == .04
                        and tracking['yaw_tolerance'] == .1 and tracking['dist_to_goal'] <= .04
                        and abs(tracking['yaw_error']) <= .1 and tracking['observed_odom_fresh'],
                        'Tracker did not meet unchanged terminal tolerances: '+str(tracking))
                require(tracking['last_command'] == {'linear_x': 0, 'angular_z': 0},
                        'Successful tracker action lacks a terminal zero command')
                require(execution['accepted'] == before['execution_state']['accepted']+index
                        and execution['output_type'] == 'kAction' and not execution['active']
                        and execution['action_commands'] == before['execution_state']['action_commands']+index
                        and execution['trajectory_commands'] == before['execution_state']['trajectory_commands'],
                        'Execution did not complete its real downstream Action: '+str(execution))
                for layer, baseline in [(tracking, before['tracking_state']),
                                        (execution, before['execution_state'])]:
                    require(all(layer[key] == baseline[key] for key in ('cancelled', 'aborted', 'rejected')),
                            'Automatic mission contains rejected, aborted or cancelled actions')
                require(tracking['goal_id'] not in goal_ids, 'Duplicate tracker success counted twice')
                goal_ids.add(tracking['goal_id'])
                require(item['planning_events'], 'Missing native planning transition')
                first_plan = item['planning_events'][0]
                if item['mode'] == 'local':
                    gains = [metric for metric in item['planning_metrics']
                             if metric.get('success') and metric.get('gain_evaluations', 0) > 0]
                    require(gains, 'Local cycle lacks a successful native gain evaluation')
                    item['gain_evaluated_map_revisions'] = [metric['map_revision'] for metric in gains]
                    gain_revision = gains[-1]['map_revision']
                    require(gain_revision >= first_plan['observation_revision']
                            and gain_revision > previous_gain_revision,
                            'Local gain evaluation did not use a new observed map')
                    previous_gain_revision = gain_revision
                if previous is not None:
                    completed = previous['control_completion']
                    item['observation_handoff'] = {
                        'completed_observation_revision': completed['observation_revision'],
                        'next_planning_observation_revision': first_plan['observation_revision'],
                        'completed_time_s': completed['time_s'], 'next_planning_time_s': first_plan['time_s']}
                    require(first_plan['time_s'] > completed['time_s']
                            and first_plan['observation_revision'] > completed['observation_revision'],
                            'Native policy replanned without a new endpoint observation')
                previous = item
            final = record['final_states']
            require(final['control_state']['completed_actions'] == len(actions)
                    and final['control_state']['cycles'] == len(actions), 'Native cycle count mismatch')
            for name in ('execution_state', 'tracking_state'):
                require(all(final[name][key]-before[name][key] == len(actions)
                            for key in ('accepted', 'succeeded'))
                        and all(final[name][key] == before[name][key]
                                for key in ('cancelled', 'aborted', 'rejected'))
                        and not final[name]['active'], 'Final Action totals differ from per-action evidence')
            require(self.last_command == (0., 0.) and self.stationary(), 'Automatic return did not stop')
            mapper = final['mapper_status']
            require(mapper['freespace']['unknown_box_operations'] == 0,
                    'Automatic mission cleared unknown space')
            require(mapper['integrated_scans'] > summary['checks']['manual_mapping_initialization']['mapper']['integrated_scans']
                    and record['map_revision_after'] > record['map_revision_before'],
                    'Automatic mission lacks new physical map observations')
            home_action = actions[-1]
            home_action['end'] = record['end']
            endpoint = home_action['commanded_path']['path'][-1]
            home_action['endpoint_error_m'] = math.dist(record['end'][:3], endpoint[:3])
            home_action['yaw_error_rad'] = abs(wrap(record['end'][3]-endpoint[3]))
            self.finish_homing(home_action, graph_home)
            summary['checks']['policy_loop_final_mapper'] = mapper

        def policy_action(self, mode, client, request, record):
            """Observe one PCI action without replacing its planner-selected route."""
            self.phase = 'local_gain_exploration' if mode == 'local' else 'autonomous_homing'
            self.until(client.service_is_ready, mode+' PCI service discovery')
            before = {name: self.state(name) for name in ('execution_state', 'tracking_state')}
            record.update(start=record_pose(self.current), before=before,
                          map_revision_before=self.map_update['map_revision'],
                          known_cells_before=self.map_update['known_cells'])
            traveled_before = self.smb['distance_m']
            command_index, control_index = len(self.commanded_paths), len(self.control_history)
            metrics = self.planning_metrics if mode == 'local' else self.global_metrics
            planned_paths = self.local_paths if mode == 'local' else self.global_paths
            metric_index, path_index = len(metrics), len(planned_paths)
            action_started = time.monotonic()
            action_deadline = min(action_started+45., deadline)

            def wait(predicate, label):
                self.until(predicate, label, max(0., action_deadline-time.monotonic()))
                require(time.monotonic() <= action_deadline, mode+' exceeded the 45 s action limit')

            def call(client, request):
                wait(client.service_is_ready, 'Action service discovery')
                future = client.call_async(request)
                wait(future.done, 'Action service response')
                return future.result()

            response = call(client, request)
            record['request_accepted'] = response.success
            require(response.success, f'{mode} PCI request rejected: {self.control}')
            wait(lambda: len(metrics) > metric_index, mode+' planning metrics')
            record['planning_metrics'] = metrics[metric_index:]
            require(metrics[-1]['success'], metrics[-1])
            if mode == 'local':
                require(metrics[-1]['gain_evaluations'] > 0, 'Local planner did not evaluate gain')
            else:
                require(metrics[-1]['mode'] == 'homing', metrics[-1])
            wait(lambda: len(planned_paths) > path_index, mode+' planned route publication')
            record['planned_path'] = planned_paths[-1]
            wait(lambda: len(self.commanded_paths) > command_index or
                 (len(self.control_history) > control_index and self.control.get('state') in
                  {'DONE', 'ERROR', 'STOPPED'}), mode+' route dispatch')
            require(len(self.commanded_paths) > command_index,
                    f'{mode} produced no executed path: {self.control}')
            record['commanded_path'] = self.commanded_paths[-1]
            path = copy.deepcopy([item.pose for item in self.last_commanded_message.poses])
            require(len(path) >= 2, f'{mode} has no motion segment')
            check = ValidatePath.Request()
            check.header.frame_id = 'map'
            check.header.stamp = self.get_clock().now().to_msg()
            check.bound_mode, check.path = 3, path
            validated = call(self.validate, check)
            record['native_validation'] = {'valid': validated.valid, 'reason': validated.reason,
                                           'map_revision': validated.map_revision}
            require(validated.valid, record['native_validation'])
            wait(lambda: len(self.control_history) > control_index and
                 self.control.get('state') in {'DONE', 'ERROR', 'STOPPED'},
                 mode+' action completion')
            record['control_result'] = copy.deepcopy(self.control)
            require(self.control.get('state') == 'DONE' and self.control.get('mode') == mode
                    and self.control.get('completed_actions') == 1 and not self.control.get('auto'),
                    self.control)
            wait(self.stationary, mode+' physical stop')
            stationary_pose = record_pose(self.current)
            observed_until = time.monotonic()+.25
            wait(lambda: time.monotonic() >= observed_until, mode+' stopped observation')
            require(self.stationary() and math.dist(stationary_pose[:3], record_pose(self.current)[:3]) < .015,
                    mode+' did not remain stopped')
            record['action_elapsed_s'] = time.monotonic()-action_started
            record['end'] = record_pose(self.current)
            record['observed_displacement_m'] = math.dist(record['start'][:3], record['end'][:3])
            record['measured_distance_m'] = self.smb['distance_m']-traveled_before
            endpoint = record_pose(path[-1])
            record['endpoint_error_m'] = math.dist(record['end'][:3], endpoint[:3])
            record['yaw_error_rad'] = abs(wrap(record['end'][3]-endpoint[3]))
            require(record['observed_displacement_m'] > .02 and record['measured_distance_m'] > .02,
                    mode+' did not produce measured robot travel')
            require(record['endpoint_error_m'] <= .1 and record['yaw_error_rad'] <= .15,
                    f'{mode} stopped outside its endpoint: {record}')
            require(self.last_command == (0., 0.), mode+' did not leave zero velocity')
            final = {name: self.state(name) for name in ('execution_state', 'tracking_state',
                                                        'control_state', 'smb_state')}
            record['final_states'] = final
            record['action_counter_increases'] = {
                name: {counter: final[name][counter]-before[name][counter]
                       for counter in ('accepted', 'succeeded')}
                for name in before}
            require(all(value == 1 for counts in record['action_counter_increases'].values()
                        for value in counts.values()), record['action_counter_increases'])
            require(final['execution_state']['output_type'] == 'kAction'
                    and final['execution_state']['action_commands'] == before['execution_state']['action_commands']+1
                    and final['execution_state']['trajectory_commands'] == before['execution_state']['trajectory_commands'],
                    final['execution_state'])
            record['map_revision_after'] = self.map_update['map_revision']
            record['known_cells_after'] = self.map_update['known_cells']
            record['planning_metrics'] = metrics[metric_index:]
            record['control_events'] = self.control_history[control_index:]

    try:
        rclpy.init(args=[])
        node = Probe()
        with log_path.open('w') as log:
            command = ['ros2', 'launch', 'gbplanner_smb', 'exploration.launch.py',
                       'visualization:=false', 'rviz:=false']
            if automatic:
                import yaml
                from ament_index_python.packages import get_package_share_directory
                source_config = Path(get_package_share_directory('gbplanner_smb')) / 'config/exploration.yaml'
                original_config = yaml.safe_load(source_config.read_text())
                auto_config = copy.deepcopy(original_config)
                auto_config['gbplanner_control']['ros__parameters']['max_cycles'] = args.auto_cycles
                overrides = {'gbplanner_control.max_cycles': args.auto_cycles}
                if args.tsdf_integration_order is not None:
                    auto_config['voxblox_mapper']['ros__parameters']['tsdf.integration_order_mode'] = args.tsdf_integration_order
                    overrides['voxblox_mapper.tsdf.integration_order_mode'] = args.tsdf_integration_order
                if args.tsdf_projection_mode is not None:
                    require(auto_config['gbplanner_local']['ros__parameters'].get(
                            'ground.interpolate_projection_distance') is True,
                            'An explicit TSDF projection mode requires interpolation enabled in the SMB configuration')
                    auto_config['gbplanner_local']['ros__parameters']['ground.tsdf_projection_mode'] = args.tsdf_projection_mode
                    overrides['gbplanner_local.ground.tsdf_projection_mode'] = args.tsdf_projection_mode
                config_path = args.output.with_name(args.output.stem+'_auto.yaml')
                config_path.write_text(yaml.safe_dump(auto_config, allow_unicode=True, sort_keys=False))
                summary['automatic_configuration'] = {
                    'source': str(source_config), 'saved_copy': str(config_path),
                    'overrides': overrides,
                }
                command.append('exploration_config:='+str(config_path))
                if args.terminal_integral_gain is not None:
                    tracking_source = Path(get_package_share_directory('gbplanner_tracking')) / 'config/smb.yaml'
                    tracking_config = yaml.safe_load(tracking_source.read_text())
                    tracking_config['gbplanner_pure_pursuit']['ros__parameters'][
                        'terminal_velocity_integral_gain'] = args.terminal_integral_gain
                    tracking_path = args.output.with_name(args.output.stem+'_tracking.yaml')
                    tracking_path.write_text(yaml.safe_dump(tracking_config, allow_unicode=True, sort_keys=False))
                    summary['tracking_configuration'] = {
                        'source': str(tracking_source), 'saved_copy': str(tracking_path),
                        'overrides': {'gbplanner_pure_pursuit.terminal_velocity_integral_gain':
                                      args.terminal_integral_gain},
                    }
                    command.append('tracking_config:='+str(tracking_path))
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=dict(os.environ))
            summary['launch'] = command
            node.until(lambda: all(client.service_is_ready() for client in
                       (node.search, node.validate, node.pci, *node.states.values()))
                       and node.current is not None and node.smb.get('wheel_contacts', 0) >= 2
                       and node.control.get('state') == 'IDLE' and node.tracking.get('state') == 'IDLE'
                       and len(node.maps) >= 3 and node.map_update.get('known_cells', 0) > 0,
                       'Live chain, initial scans and wheel-ground contact', 20.)
            if args.tsdf_integration_order is not None:
                actual_order = node.state('mapper_status')['tsdf_configuration']['integration_order_mode']
                require(actual_order == args.tsdf_integration_order, 'Mapper did not use the requested integration order')
                summary['automatic_configuration']['runtime_tsdf_integration_order'] = actual_order
            if args.terminal_integral_gain is not None:
                tracking_state = node.state('tracking_state')
                actual_gain = tracking_state.get('terminal_velocity_integral_gain')
                # The native JSON payload prints ten significant digits; allow
                # only that serialization precision, with no absolute slack at zero.
                confirmed = (tracking_state['service_success']
                             and isinstance(actual_gain, (int, float)) and not isinstance(actual_gain, bool)
                             and math.isfinite(actual_gain)
                             and math.isclose(actual_gain, args.terminal_integral_gain, rel_tol=1.e-9, abs_tol=0.))
                summary['tracking_configuration'].update(
                    runtime_terminal_velocity_integral_gain=actual_gain, runtime_confirmed=confirmed)
                require(confirmed, 'Native tracker did not confirm the explicit terminal integral gain: '
                        f'requested={args.terminal_integral_gain}, actual={actual_gain}')
            if args.tsdf_projection_mode is not None:
                request = GetParameters.Request()
                request.names = ['ground.tsdf_projection_mode', 'ground.interpolate_projection_distance']
                values = node.call(node.planner_parameters, request).values
                summary['automatic_configuration']['runtime_planner_parameters'] = {
                    name: {'type': value.type,
                           'value': value.string_value if value.type == ParameterType.PARAMETER_STRING else
                                    value.bool_value if value.type == ParameterType.PARAMETER_BOOL else None}
                    for name, value in zip(request.names, values)}
                require(len(values) == 2 and values[0].type == ParameterType.PARAMETER_STRING
                        and values[0].string_value == args.tsdf_projection_mode
                        and values[1].type == ParameterType.PARAMETER_BOOL and values[1].bool_value,
                        'Native planner did not confirm the explicit TSDF mode with interpolation enabled')
            node.settle()
            summary['before_initialization'] = {name: node.state(name) for name in
                                                ('mapper_status', 'map_status', 'smb_state')}
            require(summary['before_initialization']['mapper_status']['freespace']['unknown_box_operations'] == 0,
                    'Mapping check requires startup robot-box clearing to remain disabled')
            require(all(item['frame'] == 'smb_lidar' for item in node.clouds) and node.clouds,
                    'Missing physical SMB lidar stream')
            require(all(item['frame'] == 'map' for item in node.maps), 'Wrong accepted map frame')

            node.phase = 'manual_mapping_initialization'
            # The roof lidar cannot see through the chassis: its first visible
            # forward floor return is about 0.94 m away. Sweep two metres in
            # this scene's open doorway, then return into the observed floor.
            # A sub-metre out-and-back leaves an unobserved patch under base.
            legs = [node.mapping_leg(2.), node.mapping_leg(-1.5)]
            revision = node.map_update['map_revision']
            node.until(lambda: node.map_update.get('map_revision', 0) >= revision+3,
                       'Real TSDF fusion after teleop initialization', 6.)
            node.settle()
            node.until(lambda: node.state('velocity_mux_state')['gate'] == 'waiting_for_goal',
                       'Manual takeover fully canceled before new PCI goal', 5.)
            summary['checks']['manual_mapping_initialization'] = {
                'autonomous': False, 'method': 'teleop velocities stopped by measured odometry distance',
                'legs': legs, 'mapper': node.state('mapper_status'), 'map': node.state('map_status'),
                'smb': node.state('smb_state'),
            }

            if not automatic:
                node.phase = 'planner_search'
                source = copy.deepcopy(node.current)
                target = pose(-.8, 0.)
                request = PlannerSearch.Request()
                request.header.frame_id = 'map'
                request.header.stamp = node.get_clock().now().to_msg()
                request.use_current_state, request.bound_mode, request.target = True, 3, target
                before_metrics = len(node.search_metrics)
                response = node.call(node.search, request, 15.)
                node.until(lambda: len(node.search_metrics) > before_metrics, 'Search diagnostic publication')
                search_record = {'success': response.success, 'source': record_pose(source),
                                 'requested_target': record_pose(target),
                                 'path': [record_pose(value) for value in response.path],
                                 'metrics': node.search_metrics[-1]}
                summary['checks']['planner_search'] = search_record
                if not response.success or not response.path:
                    summary['initial_map_feasibility'] = {
                        'search_reason': node.search_metrics[-1].get('reason'),
                        'current_pose': node.validate_path([source]),
                        'target_pose': node.validate_path([target]),
                        'direct_segment': node.validate_path([source, target]),
                        'note': 'No retry, map injection or clearing was applied by this check.',
                    }
                    raise RuntimeError('First real-map search is infeasible: '+str(node.search_metrics[-1]))
                search_record['validation'] = node.validate_path(response.path)
                require(search_record['validation']['valid'], search_record['validation'])
                # Regression for physical odometry -> TSDF-root height attachment:
                # preserve collision/support checks while rejecting a real lift.
                attachment = node.validate_path([source, *response.path])
                raised = copy.deepcopy(source)
                raised.position.z += .2
                vertical = node.validate_path([source, raised])
                summary['checks']['ground_height_attachment'] = {
                    'observed_to_projected_root': attachment, 'unsupported_vertical_motion': vertical}
                require(attachment['valid'] and not vertical['valid'],
                        summary['checks']['ground_height_attachment'])
                require(math.dist(record_pose(response.path[-1])[:2], record_pose(target)[:2]) <= .1,
                        'Search stopped outside the requested short target neighborhood')

                node.phase = 'autonomous_pci_search'
                node.until(lambda: node.target.get_subscription_count() > 0, 'PCI target subscriber')
                stamped = PoseStamped()
                stamped.header.frame_id = 'map'
                stamped.header.stamp = node.get_clock().now().to_msg()
                stamped.pose = target
                node.target.publish(stamped)
                node.spin_for(.15)
                before_pose = record_pose(node.current)
                before_distance = node.smb['distance_m']
                before_revision = node.map_update['map_revision']
                before_completed = node.control.get('completed_actions', 0)
                request = PciSearch.Request()
                request.header.frame_id = 'map'
                request.header.stamp = node.get_clock().now().to_msg()
                request.use_current_state, request.not_exe_path, request.bound_mode = True, False, 3
                dispatched = node.call(node.pci, request, 15.)
                pci_record = {'accepted': dispatched.success,
                              'path': [record_pose(value) for value in dispatched.path],
                              'start': before_pose}
                summary['checks']['autonomous_pci_search'] = pci_record
                require(dispatched.success and dispatched.path, f'PCI did not dispatch: {node.control}')
                node.until(lambda: node.control.get('state') in {'DONE', 'ERROR', 'STOPPED'},
                           'Autonomous PCI path completion', 30.)
                require(node.control.get('state') == 'DONE', node.control)
                node.settle()
                final_states = {name: node.state(name) for name in node.states}
                pci_record['final_states'] = final_states
                pci_record['end'] = record_pose(node.current)
                pci_record['observed_displacement_m'] = math.dist(before_pose[:3], pci_record['end'][:3])
                pci_record['measured_distance_m'] = node.smb['distance_m']-before_distance
                endpoint = record_pose(dispatched.path[-1])
                pci_record['endpoint_error_m'] = math.dist(pci_record['end'][:3], endpoint[:3])
                pci_record['yaw_error_rad'] = abs(wrap(pci_record['end'][3]-endpoint[3]))
                require(final_states['control_state']['completed_actions'] == before_completed+1,
                        final_states['control_state'])
                require(final_states['execution_state']['output_type'] == 'kAction'
                        and final_states['execution_state']['action_commands'] == 1
                        and final_states['execution_state']['succeeded'] == 1
                        and final_states['execution_state']['trajectory_commands'] == 0,
                        final_states['execution_state'])
                require(final_states['tracking_state']['succeeded'] == 1,
                        final_states['tracking_state'])
                require(pci_record['observed_displacement_m'] > .5
                        and pci_record['endpoint_error_m'] <= .1 and pci_record['yaw_error_rad'] <= .15,
                        pci_record)
                require(node.last_command == (0., 0.), 'Tracker did not publish terminal zero cmd_vel')
                require(any(value['phase'] == 'autonomous_pci_search' and value['wheel_contacts'] >= 2
                            and max(map(abs, value['wheel_speed'])) > .1 for value in node.smb_history),
                        'Autonomous stage lacks observed driven wheel contacts')
                mapper = final_states['mapper_status']
                require(mapper['backend'] == 'voxblox_tsdf' and mapper['storage'] == 'sparse'
                        and mapper['integrated_scans'] > summary['before_initialization']['mapper_status']['integrated_scans'],
                        mapper)
                require(mapper['freespace']['unknown_box_operations'] == 0,
                        'An unknown-box clearing operation occurred during the live-map check')
                known = [value['known_cells'] for value in node.map_history if value.get('known_cells', 0) > 0]
                summary['checks']['mapping'] = {
                    'lidar_frame': 'smb_lidar', 'accepted_map_frame': 'map',
                    'first_known_cells': known[0], 'final_known_cells': known[-1],
                    'peak_known_cells': max(known), 'known_growth_from_first': max(known)-known[0],
                    'revision_before_autonomous_motion': before_revision,
                    'revision_after_autonomous_motion': node.map_update['map_revision'],
                    'configured_startup_freespace': mapper.get('freespace'),
                }
                require(max(known) > known[0] and node.map_update['map_revision'] > before_revision,
                        'No observed real-map growth/update')
            if args.explore:
                home_record = node.observe_home()

                local_record = {
                    'selection': 'native local RRG gain planner; no target pose supplied',
                    'execution_scope': 'one stop-and-observe step; the PCI executes the '
                                       'first segment of its selected local route',
                    'set_auto': False, 'bound_mode': 3, 'velocity_mps': .3,
                }
                summary['checks']['local_gain_exploration'] = local_record
                request = PciTrigger.Request()
                request.not_exe_path, request.set_auto = False, False
                request.bound_mode, request.vel_max = 3, .3
                node.policy_action('local', node.local_trigger, request, local_record)

                homing_record = {'home_pose': home_record['base_pose'],
                                 'home_planning_pose': home_record['planning_pose'], 'home_vertex_id': 0}
                summary['checks']['autonomous_homing'] = homing_record
                request = PciHomingTrigger.Request()
                request.not_exe_path = False
                node.policy_action('homing', node.homing_trigger, request, homing_record)
                node.finish_homing(homing_record, home_record)
                summary['checks']['policy_loop_final_mapper'] = node.state('mapper_status')
                require(summary['checks']['policy_loop_final_mapper']['freespace']['unknown_box_operations'] == 0,
                        'Policy loop must use the observed map without unknown-box clearing')
            if automatic:
                home_record = node.observe_home()
                auto_record = {
                    'selection': 'one native PCI automatic trigger from the measured initialization endpoint',
                    'search_stage_skipped': True,
                    'execution_scope': 'native stop-observe-local replanning followed by native homing',
                    'requested_cycles': args.auto_cycles, 'completed_local_actions': 0,
                    'completed_homing_actions': 0,
                }
                summary['checks']['automatic_policy'] = auto_record
                node.automatic_policy(auto_record, home_record)
            summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
        if node is not None:
            summary['last_observed'] = {'pose': record_pose(node.current) if node.current else None,
                                      'map': node.map_update, 'control': node.control,
                                      'execution': node.execution, 'tracking': node.tracking,
                                      'smb': node.smb, 'last_map': node.maps[-1:]}
            summary['diagnostic_states'] = {}
            for name, client in node.states.items():
                if not client.service_is_ready() or time.monotonic()+1. > deadline:
                    continue
                try:
                    summary['diagnostic_states'][name] = node.state(name, .8)
                except Exception as diagnostic_error:
                    summary['diagnostic_states'][name] = {'error': str(diagnostic_error)}
    finally:
        if node is not None and node.auto_record is not None:
            node.collect_auto_actions()
        if node is not None and node.phase == 'manual_mapping_initialization':
            node.stop_teleop()
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=1.)
        if node is not None:
            with trace_path.open('w', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(['time_s', 'phase', 'x_m', 'y_m', 'z_m', 'yaw_rad', 'speed_mps', 'yaw_rate_radps',
                                 'odom_stamp_s', 'odom_source_age_s', 'odom_receive_gap_s'])
                writer.writerows(node.trace)
            with commands_path.open('w', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(['time_s', 'phase', 'linear_x_mps', 'angular_z_radps'])
                writer.writerows(node.commands)
            history_path.write_text(json.dumps({
                'smb': node.smb_history, 'lidar_clouds': node.clouds, 'accepted_maps': node.maps,
                'map_updates': node.map_history, 'search_metrics': node.search_metrics,
                'search_paths': node.search_paths, 'commanded_paths': node.commanded_paths,
                'planning_metrics': node.planning_metrics, 'global_metrics': node.global_metrics,
                'local_paths': node.local_paths, 'global_paths': node.global_paths,
                'global_graphs': node.global_graph_history,
                'control': node.control_history, 'execution': node.execution_history,
                'tracking': node.tracking_history,
            }, ensure_ascii=False, indent=2)+'\n')
            summary['odometry_samples'] = len(node.trace)
            summary['odometry_timing'] = {
                'freshness_timeout_s': node.odom_timeout,
                'max_receive_gap_s': node.max_odom_receive_gap,
                'max_source_stamp_gap_s': node.max_odom_source_gap,
                'max_source_age_at_receive_s': node.max_odom_source_age,
            }
            summary['observer_timing'] = {
                'spin_calls': node.spin_calls,
                'max_spin_wall_s': node.max_spin_wall, 'max_spin_thread_cpu_s': node.max_spin_cpu,
                'spin_wall_event_threshold_s': .05, 'spin_entry_gap_event_threshold_s': .1,
                'odometry_age_or_gap_event_threshold_s': .1, 'retained_events_per_kind': 8,
                'message_info_samples': node.odom_message_info_samples,
                'message_info_availability': 'not_exposed_by_installed_humble_callback_api',
                'slow_spins': node.slow_spins, 'odometry_events': node.odom_timing_events,
            }
            summary['lidar_clouds'] = len(node.clouds)
            summary['accepted_map_snapshots'] = len(node.maps)
            summary['chassis_contact_steps'] = max((v['chassis_contact_steps'] for v in node.smb_history), default=0)
            known = [v['known_cells'] for v in node.map_history if v.get('known_cells', 0) > 0]
            summary['mapping_observations'] = {
                'lidar_frames': sorted({v['frame'] for v in node.clouds}),
                'accepted_map_frames': sorted({v['frame'] for v in node.maps}),
                'first_known_cells': known[0] if known else 0,
                'last_known_cells': known[-1] if known else 0,
                'peak_known_cells': max(known, default=0),
            }
            if summary['status'] != 'passed' and node.last_map is not None:
                snapshot = node.last_map
                requested_revision = None
                exact_revision = False
                capture_source = 'latest_received'
                if node.execution_validation_failure is not None:
                    requested_revision = node.execution_validation_failure['map_revision']
                    capture_source = 'execution_validation'
                    summary['execution_validation_failure'] = node.execution_validation_failure
                    node.pin_failed_input()
                    if node.execution_failure_snapshot is not None:
                        snapshot, exact_revision = node.execution_failure_snapshot, True
                elif node.planning_metrics and not node.planning_metrics[-1]['success']:
                    requested_revision = node.planning_metrics[-1]['map_revision']
                    capture_source = 'planning_failure'
                    node.pin_failed_input()
                    if node.failure_metric is node.planning_metrics[-1] and node.failure_snapshot is not None:
                        snapshot, exact_revision = node.failure_snapshot, True
                data_path = args.output.with_name(args.output.stem+'_map.bin')
                metadata_path = args.output.with_name(args.output.stem+'_map.json')
                data_path.write_bytes(bytes(snapshot.data))
                metadata = {
                    'topic': 'accepted_voxel_map', 'frame_id': snapshot.header.frame_id,
                    'stamp': {'sec': snapshot.header.stamp.sec, 'nanosec': snapshot.header.stamp.nanosec},
                    'width': snapshot.width, 'height': snapshot.height,
                    'point_step': snapshot.point_step, 'row_step': snapshot.row_step,
                    'is_bigendian': snapshot.is_bigendian, 'is_dense': snapshot.is_dense,
                    'fields': [{'name': field.name, 'offset': field.offset,
                                'datatype': field.datatype, 'count': field.count} for field in snapshot.fields],
                    'data_bytes': len(snapshot.data), 'data_file': str(data_path),
                    'requested_map_revision': requested_revision,
                    'exact_requested_revision': exact_revision,
                    'source': capture_source,
                }
                metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
                summary['failure_map_snapshot'] = {'metadata': str(metadata_path), 'data': str(data_path)}
                summary['failure_map_snapshot'].update(requested_map_revision=requested_revision,
                                                       exact_requested_revision=exact_revision,
                                                       source=capture_source)
                graph_snapshot = node.failure_graph if node.failure_graph is not None else node.global_graph
                if graph_snapshot is not None:
                    from rosidl_runtime_py.convert import message_to_ordereddict
                    graph_path = args.output.with_name(args.output.stem+'_global_graph.json')
                    graph_path.write_text(json.dumps(message_to_ordereddict(graph_snapshot),
                                                     ensure_ascii=False, indent=2)+'\n')
                    summary['failure_global_graph'] = str(graph_path)
                    summary['failure_global_graph_scope'] = (
                        'Latest graph received when the failed local metric arrived; '
                        'not guaranteed to be the pre-request graph' if node.failure_graph is not None else
                        'Latest received graph at shutdown; not guaranteed to be the pre-request graph')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        summary['elapsed_s'] = round(time.monotonic()-started, 3)
        summary['processes_stopped'] = process is None or process.poll() is not None
        summary['artifacts'] = {'odometry': str(trace_path), 'commands': str(commands_path),
                                'history': str(history_path), 'log': str(log_path)}
        args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(summary, ensure_ascii=False))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
