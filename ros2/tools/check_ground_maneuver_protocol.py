#!/usr/bin/env python3
"""Exercise the real follower/tracker GroundManeuver protocol in isolation.

This is NOT native SMB physics, a real map, or a collision/support test. A
50 Hz system-clock fixture integrates only the real tracker's external/cmd_vel
using q = p + a*heading, q_dot = v*heading, yaw_dot = w. ValidatePath explicitly
accepts finite fixture requests; it does not replace production geometry proof.
The unchanged saved candidate is encoded by ground_maneuver_transport.

Both completion and cancellation require fresh, advancing odometry, stationary
measurements and the follower's frozen stop-confirmed outcome. Action status
alone cannot pass. The fixture never publishes cmd_vel or follows reference
samples to generate motion. Only this script's child process groups are stopped.

Run after sourcing the rebuilt ROS overlay, for example:
  python3 tools/check_ground_maneuver_protocol.py --output results/protocol.json
Root orchestration owns building and running this tool; importing it does not
start ROS, a simulator, a search, or any child process.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import sys
import time
import traceback

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
PERIOD = .02
CAPS = dict(max_speed=.3, max_yaw_rate=.4, max_acceleration=.3,
            max_yaw_acceleration=.6, max_duration=30., max_path_length=5.)


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def wrap(value):
    return math.remainder(value, 2. * math.pi)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def advance_offset(pose, v, w, dt, offset):
    """Exact zero-order-held command integration, with the signed ICR offset."""
    half = .5 * w * dt
    heading = pose[3] + half
    distance = v * dt * (math.sin(half) / half if half else 1.)
    chord = 2. * offset * math.sin(half)
    return (pose[0] + distance*math.cos(heading) + chord*math.sin(heading),
            pose[1] + distance*math.sin(heading) - chord*math.cos(heading),
            pose[2], wrap(pose[3] + w*dt))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path,
                        default=ROOT/'results/pittsburgh_port/preflight1.json')
    parser.add_argument('--candidate', type=int, default=0)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int,
                        help='Isolated ROS domain; default chooses a fresh local value.')
    parser.add_argument('--case', choices=('both', 'complete', 'cancel'), default='both')
    parser.add_argument('--straight-crossings', action='store_true',
                        help='Declare three ordered crossings on each straight for the new wire-contract check')
    parser.add_argument('--case-timeout', type=float, default=36.)
    parser.add_argument('--cancel-after', type=float, default=1.2,
                        help='Seconds after acceptance before cancellation while moving.')
    args = parser.parse_args()
    if (not math.isfinite(args.case_timeout) or not 5. <= args.case_timeout <= 60. or
            not math.isfinite(args.cancel_after) or not .5 <= args.cancel_after <= 20.):
        parser.error('Use a finite 5–60 s case timeout and .5–20 s cancel delay')
    args.input, args.output = args.input.resolve(), args.output.resolve()
    if args.output.suffix != '.json':
        parser.error('--output must be a new .json file')
    cases = ('complete', 'cancel') if args.case == 'both' else (args.case,)
    trace_path = args.output.with_name(args.output.stem + '_trace.json')
    log_paths = {name: args.output.with_name(args.output.stem + '_' + name + '.log')
                 for name in cases}
    if any(path.exists() for path in (args.output, trace_path, *log_paths.values())):
        parser.error('Use a new output stem; existing evidence is never overwritten')
    domain = args.domain if args.domain is not None else random.SystemRandom().randrange(120, 230)
    if not 1 <= domain <= 232:
        parser.error('--domain must be between 1 and 232')
    # Set before importing/initializing ROS. Unique namespaces additionally
    # prevent accidental connection to another user's nodes in this domain.
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    report = {
        'status': 'failed', 'scope': 'Real ROS follower/tracker/action protocol with synthetic offset-kinematic feedback',
        'native_execution': False, 'physical_dynamics': False,
        'real_map_validation': False, 'production_defaults_changed': False,
        'fixture_validation': 'Accept finite map-frame fixture requests; no map, occupancy, terrain or sweep calculation',
        'feedback_model': 'Exact command-held q=p+a*heading; input is only received external/cmd_vel',
        'clock': 'System ROS time and monotonic wall time; no /clock or accelerated simulation',
        'requested_frequency_hz': 50., 'domain': domain, 'localhost_only': True,
        'input': str(args.input), 'candidate_index': args.candidate,
        'caps': CAPS, 'cases': [], 'trace_file': str(trace_path),
        'limitations': [
            'The fixture does not establish collision-free or supported motion.',
            'A fixed offset and ideal command response do not reproduce wheel/contact physics.',
            'Saved preflight geometry evidence is separate from this protocol result.',
        ],
    }
    traces = {}
    ros = None
    interrupted = [None]
    previous_handlers = {}
    try:
        import rclpy as ros
        from ament_index_python.packages import get_package_prefix
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from planner_msgs.action import PathFollower
        from planner_msgs.srv import ValidatePath
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        execution_lib = Path(get_package_prefix('gbplanner_execution'))/'lib/gbplanner_execution'
        sys.path.insert(0, str(execution_lib))
        import ground_maneuver_transport as transport
        from ground_maneuver import Candidate, Limits, Primitive, Sample

        source = json.loads(args.input.read_text())
        selected = source['construction']['candidates'][args.candidate]
        checks = next(row for row in source['candidate_checks'] if row['candidate'] == args.candidate)
        require(checks['status'] == 'passed', 'Saved candidate did not pass its preflight checks')
        limits = Limits(**source['limits'])
        for key, maximum in CAPS.items():
            require(getattr(limits, key) <= maximum, 'Candidate exceeds harness cap: ' + key)
        candidate = Candidate(
            samples=tuple(Sample(**row) for row in selected['samples']),
            primitives=tuple(Primitive(**{**row,
                'expected_start': tuple(row['expected_start']),
                'expected_end': tuple(row['expected_end'])}) for row in selected['primitives']),
            duration=selected['duration'], path_length=selected['path_length'],
            control_offset=selected['control_offset'], directions=tuple(selected['directions']))
        crossings = [tuple(abs(p.signed_amount)*f for f in (.25, .5, .75))
            if args.straight_crossings and p.kind == 'straight' else () for p in candidate.primitives]
        crossing_count = sum(map(len, crossings))
        z = float(source['candidate_base_z'])
        initial = (*candidate.samples[0].pose[:2], z, candidate.samples[0].yaw)
        target = (*candidate.samples[-1].pose[:2], z, candidate.samples[-1].yaw)
        require(all(math.isfinite(v) for v in (*initial, *target)), 'Nonfinite candidate pose')
        report.update(source_sha256={str(args.input): digest(args.input),
            str(Path(__file__).resolve()): digest(__file__),
            str(Path(transport.__file__).resolve()): digest(transport.__file__)},
            source_candidate_checks=checks, initial_pose=initial, target_pose=target,
            source_task_endpoint=source['source_route'][-1],
            reference_duration_s=candidate.duration, reference_points=len(candidate.samples),
            primitive_count=len(candidate.primitives), limits=asdict(limits))
        report.update(pass_through_distances=crossings,
                      crossing_scope='Synthetic protocol obligations on unchanged reference geometry')

        class Probe(Node):
            def __init__(self, name, namespace):
                super().__init__('protocol_fixture', namespace=namespace)
                self.case = name
                self.started = time.monotonic()
                self.pose, self.command = initial, (0., 0.)
                self.last_integration = self.started
                self.last_odom_pose, self.last_odom_ns = None, None
                self.phase = 'startup'
                self.revision = 1
                self.processes, self.failures = [], []
                self.goal_handle, self.result_future = None, None
                self.execution_status, self.tracking_status = {}, {}
                self.trace = {key: [] for key in
                              ('odometry', 'commands', 'states', 'validation_requests', 'feedback', 'events')}
                self.stream = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
                durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.odom = self.create_publisher(Odometry, 'odometry', self.stream)
                self.maps = self.create_publisher(String, 'map_updates', durable)
                self.create_subscription(Twist, 'external/cmd_vel', self.receive_command, self.stream)
                self.create_subscription(String, 'execution_status',
                    lambda msg: self.receive_status('execution', msg), durable)
                self.create_subscription(String, 'tracking_status',
                    lambda msg: self.receive_status('tracking', msg), durable)
                self.create_service(ValidatePath, 'validate_path', self.validate_fixture)
                self.action = ActionClient(self, PathFollower, 'path_follower')
                self.execution_state = self.create_client(Trigger, 'execution_state')
                self.tracking_state = self.create_client(Trigger, 'tracking_state')
                self.timer = self.create_timer(PERIOD, self.publish_odometry)
                self.map_timer = self.create_timer(.2, self.publish_map)
                self.publish_map()

            def event(self, kind, **values):
                row = {'wall_s': time.monotonic()-self.started, 'phase': self.phase,
                       'kind': kind, **values}
                self.trace['events'].append(row)
                return row

            def integrate_to(self, now):
                dt = now-self.last_integration
                require(dt >= 0., 'Monotonic clock regressed')
                self.pose = advance_offset(self.pose, *self.command, dt, candidate.control_offset)
                self.last_integration = now

            def receive_command(self, message):
                now = time.monotonic()
                values = [float(v) for v in (message.linear.x, message.linear.y, message.linear.z,
                                             message.angular.x, message.angular.y, message.angular.z)]
                require(all(math.isfinite(v) for v in values), 'Nonfinite tracker command')
                self.integrate_to(now)
                previous = self.command
                self.command = (values[0], values[5])
                row = {'wall_s': now-self.started, 'phase': self.phase,
                       'twist': values, 'pose_at_receipt': self.pose,
                       'delta_v': self.command[0]-previous[0],
                       'delta_w': self.command[1]-previous[1]}
                self.trace['commands'].append(row)
                if (abs(values[0]) > CAPS['max_speed']+1.e-8 or
                        abs(values[5]) > CAPS['max_yaw_rate']+1.e-8 or
                        any(abs(values[i]) > 1.e-10 for i in (1, 2, 3, 4))):
                    self.failures.append('tracker_command_outside_caps_or_nonplanar')
                # The controller clamps its slew interval to dt=.02. This is
                # a command-step bound, not measured actuator acceleration.
                if (abs(row['delta_v']) > CAPS['max_acceleration']*PERIOD+1.e-6 or
                        abs(row['delta_w']) > CAPS['max_yaw_acceleration']*PERIOD+1.e-6):
                    self.failures.append('received_command_step_exceeds_controller_slew_cap')

            def publish_odometry(self):
                now = time.monotonic()
                self.integrate_to(now)
                stamp = self.get_clock().now()
                stamp_ns = stamp.nanoseconds
                if self.last_odom_ns is not None:
                    require(stamp_ns > self.last_odom_ns, 'System/source clock did not advance')
                message = Odometry()
                message.header.stamp = stamp.to_msg()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                x, y, height, yaw = self.pose
                message.pose.pose.position.x = x
                message.pose.pose.position.y = y
                message.pose.pose.position.z = height
                message.pose.pose.orientation.z = math.sin(yaw/2.)
                message.pose.pose.orientation.w = math.cos(yaw/2.)
                v, w = self.command
                message.twist.twist.linear.x = v
                message.twist.twist.linear.y = -candidate.control_offset*w
                message.twist.twist.angular.z = w
                fd_speed = fd_rate = None
                if self.last_odom_pose is not None:
                    dt = (stamp_ns-self.last_odom_ns)*1.e-9
                    fd_speed = math.dist(self.pose[:3], self.last_odom_pose[:3])/dt
                    fd_rate = abs(wrap(yaw-self.last_odom_pose[3]))/dt
                self.odom.publish(message)
                self.trace['odometry'].append({
                    'wall_s': now-self.started, 'phase': self.phase, 'stamp_ns': stamp_ns,
                    'pose': self.pose, 'body_vx': v, 'body_vy': -candidate.control_offset*w,
                    'yaw_rate': w, 'fd_speed_mps': fd_speed, 'fd_yaw_rate_rps': fd_rate})
                self.last_odom_pose, self.last_odom_ns = self.pose, stamp_ns

            def publish_map(self):
                message = String()
                message.data = json.dumps({'map_revision': self.revision, 'known_cells': 1,
                                           'scope': 'protocol_fixture_not_a_map'})
                self.maps.publish(message)

            def receive_status(self, layer, message):
                value = json.loads(message.data)
                self.trace['states'].append({'wall_s': time.monotonic()-self.started,
                                             'phase': self.phase, 'layer': layer, 'payload': value})
                if layer == 'execution':
                    self.execution_status = value
                else:
                    self.tracking_status = value

            def validate_fixture(self, request, response):
                states = []
                valid = request.header.frame_id == 'map' and request.bound_mode == 3 and bool(request.path)
                for pose in request.path:
                    p, q = pose.position, pose.orientation
                    raw = [float(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)]
                    valid = valid and all(math.isfinite(v) for v in raw)
                    states.append(raw)
                response.valid = bool(valid)
                response.reason = ('protocol_fixture_finite_request_accepted' if valid
                                   else 'protocol_fixture_invalid_request')
                response.map_revision = self.revision
                response.failure_segment = -1
                self.trace['validation_requests'].append({
                    'wall_s': time.monotonic()-self.started, 'phase': self.phase,
                    'first_pose_is_observation': bool(request.first_pose_is_observation),
                    'bound_mode': int(request.bound_mode), 'clearance_margin': float(request.clearance_margin),
                    'clearance_segments': int(request.clearance_segments), 'poses_xyz_xyzw': states,
                    'valid': bool(response.valid), 'reason': str(response.reason),
                    'map_revision': self.revision, 'real_geometry_checked': False})
                return response

            def feedback(self, message):
                f = message.feedback
                self.trace['feedback'].append({'wall_s': time.monotonic()-self.started,
                    'remaining_waypoints': int(f.remaining_waypoints), 'dist_to_goal': float(f.dist_to_goal),
                    'estimated_time_remaining': float(f.estimated_time_remaining)})

            def until(self, predicate, seconds, label, *, strict=True):
                deadline = time.monotonic()+seconds
                while not predicate():
                    if strict:
                        require(interrupted[0] is None, 'Interrupted by '+str(interrupted[0]))
                        require(not self.failures, '; '.join(self.failures))
                        for item in self.processes:
                            require(item['process'].poll() is None,
                                    item['role']+' exited before case completion')
                    if time.monotonic() > deadline:
                        raise TimeoutError(label)
                    ros.spin_once(self, timeout_sec=.005)
                if strict:
                    require(not self.failures, '; '.join(self.failures))

            def spin_for(self, seconds, strict=True):
                end = time.monotonic()+seconds
                self.until(lambda: time.monotonic() >= end, seconds+.25,
                           'Observation interval', strict=strict)

            def snapshot(self, client, *, deadline=None):
                def remaining():
                    seconds = 2. if deadline is None else deadline-time.monotonic()
                    if seconds <= 0.:
                        raise TimeoutError('State snapshot shared deadline expired')
                    return seconds

                self.until(client.service_is_ready, remaining(), 'State service discovery')
                future = client.call_async(Trigger.Request())
                self.until(future.done, remaining(), 'State service response')
                require(future.result().success, 'State service failed')
                return json.loads(future.result().message)

            def stationary_after(self, stamp_ns):
                rows = self.trace['odometry'][-5:]
                return (len(rows) == 5 and rows[0]['stamp_ns'] > stamp_ns and
                    time.monotonic()-self.started-rows[-1]['wall_s'] < .1 and
                    all(math.hypot(row['body_vx'], row['body_vy']) <= .02 and
                        abs(row['yaw_rate']) <= .03 and
                        row['fd_speed_mps'] is not None and row['fd_speed_mps'] <= .02 and
                        row['fd_yaw_rate_rps'] <= .03 for row in rows) and
                    self.command == (0., 0.))

        def launch(probe, log, package, executable, role, parameters, remappings=()):
            binary = Path(get_package_prefix(package))/'lib'/package/executable
            command = [str(binary), '--ros-args', '-r', '__ns:='+probe.get_namespace(),
                       '-r', '__node:=protocol_'+role]
            for key, value in parameters.items():
                encoded = str(value).lower() if isinstance(value, bool) else str(value)
                command.extend(('-p', key+':='+encoded))
            for original, replacement in remappings:
                command.extend(('-r', original+':='+replacement))
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=dict(os.environ))
            item = {'role': role, 'command': command, 'pid': process.pid, 'process': process,
                    'executable_sha256': digest(binary), 'signals': []}
            probe.processes.append(item)

        def cleanup(probe):
            probe.phase = 'cleanup'
            if probe.goal_handle is not None and probe.goal_handle.accepted and (
                    probe.result_future is None or not probe.result_future.done()):
                try:
                    probe.goal_handle.cancel_goal_async()
                    probe.event('cleanup_cancel_requested')
                    probe.until(lambda: probe.result_future is not None and probe.result_future.done()
                                and probe.stationary_after(0), 4., 'Cleanup cancellation', strict=False)
                except Exception as error:
                    probe.event('cleanup_cancel_error', error=str(error))
            # The follower goes first, while tracker and odometry remain alive
            # for its cancellation handshake. No global process-name kill.
            for item in reversed(probe.processes):
                process = item['process']
                for sig, seconds in ((signal.SIGINT, 5.), (signal.SIGTERM, 1.), (signal.SIGKILL, 1.)):
                    if process.poll() is not None:
                        break
                    try:
                        os.killpg(process.pid, sig)
                        item['signals'].append(sig.name)
                        probe.until(lambda: process.poll() is not None, seconds,
                                    item['role']+' cleanup', strict=False)
                    except ProcessLookupError:
                        break
                    except Exception as error:
                        probe.event('cleanup_wait_error', role=item['role'], error=str(error))
                item['exit_code'] = process.poll()
            return [{k: v for k, v in item.items() if k != 'process'} for item in probe.processes]

        # Keep DDS and feedback alive while our finally block asks the real
        # nodes to brake. The default ROS handler would shut them down first.
        def request_stop(signum, _frame):
            interrupted[0] = signal.Signals(signum).name

        for sig in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[sig] = signal.signal(sig, request_stop)
        ros.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        for case in cases:
            namespace = '/ground_maneuver_protocol_'+str(os.getpid())+'_'+case
            probe = Probe(case, namespace)
            outcome = {'case': case, 'status': 'failed', 'namespace': namespace,
                       'log_file': str(log_paths[case])}
            report['cases'].append(outcome)
            try:
                with log_paths[case].open('x') as log:
                    try:
                        common = {'use_sim_time': False, 'allow_ground_maneuver': True,
                            **{'ground_maneuver_'+key: value for key, value in CAPS.items()}}
                        launch(probe, log, 'gbplanner_tracking', 'pure_pursuit_node', 'tracker', {
                            **common, 'dt': PERIOD, 'desired_vel': .3, 'max_lin_vel': .3,
                            'max_ang_vel': .4, 'goal_reaching_threshold': .04,
                            'goal_yaw_threshold': .1, 'odom_timeout': .5,
                            'preserve_corners': True, 'steering_curvature': 'signed_geometric'},
                            (('cmd_vel', 'external/cmd_vel'),))
                        launch(probe, log, 'gbplanner_execution', 'path_follower_node.py', 'follower', {
                            **common, 'frame_id': 'map', 'robot_type': 'kGround',
                            'output_type': 'kAction', 'output_action': 'pci_output_path',
                            'speed': .3, 'yaw_rate': .4, 'sample_period': .05,
                            'smooth_yaw': False, 'odom_timeout': .5,
                            'endpoint_tolerance': .075, 'yaw_tolerance': .1,
                            'validation_timeout': 5., 'execution_grace': 3.})
                        startup_deadline = time.monotonic()+10.
                        probe.until(lambda: probe.action.server_is_ready()
                            and probe.execution_state.service_is_ready()
                            and probe.tracking_state.service_is_ready()
                            and probe.odom.get_subscription_count() >= 2,
                            startup_deadline-time.monotonic(), 'Follower/tracker/fixture discovery')
                        # Status topics publish transitions, so their startup
                        # known_cells=0 can remain latched after map receipt.
                        # Read the actual state under the SAME startup budget.
                        while True:
                            state = probe.snapshot(probe.execution_state, deadline=startup_deadline)
                            probe.execution_status = state
                            probe.event('execution_discovery_snapshot', payload=state)
                            if (state.get('known_cells', 0) > 0 and state.get('output_ready') and
                                    state.get('odometry_freshness', {}).get('fresh')):
                                break
                            remaining = startup_deadline-time.monotonic()
                            if remaining <= 0.:
                                raise TimeoutError('Follower map/output/odometry readiness')
                            next_poll = time.monotonic()+min(.05, remaining)
                            probe.until(lambda: time.monotonic() >= next_poll, remaining,
                                        'Follower readiness polling')
                        probe.spin_for(.3)
                        require(probe.stationary_after(0), 'Initial state is not observed stationary')
                        outcome['initial_execution'] = probe.snapshot(probe.execution_state)
                        outcome['initial_tracking'] = probe.snapshot(probe.tracking_state)
                        require(outcome['initial_execution'].get('allow_ground_maneuver') is True and
                                outcome['initial_tracking'].get('allow_ground_maneuver') is True,
                                'Both production nodes must explicitly enable the maneuver protocol')
                        path, maneuver = transport.candidate_to_message(candidate, z=z,
                            frame_id='map', limits=limits, max_duration=30.,
                            stamp=probe.get_clock().now().to_msg(), pass_through_distances=crossings)
                        goal = PathFollower.Goal()
                        goal.path, goal.maneuver = path, maneuver
                        goal.execution_mode = PathFollower.Goal.GROUND_MANEUVER
                        goal.manual_path, goal.bound_mode, goal.vel_max = False, 3, .3
                        goal.pre_execution_timeout = 5.
                        outcome['submitted_reference'] = {
                            'execution_mode': int(goal.execution_mode), 'manual_path': bool(goal.manual_path),
                            'bound_mode': int(goal.bound_mode), 'vel_max': float(goal.vel_max),
                            'pre_execution_timeout_s': float(goal.pre_execution_timeout),
                            'max_duration_s': float(maneuver.max_duration),
                            'path_points': len(path), 'segments': len(maneuver.segments),
                            'pass_through_distances': [list(s.pass_through_distances) for s in maneuver.segments]}
                        probe.phase = 'executing'
                        probe.event('goal_sent')
                        future = probe.action.send_goal_async(goal, feedback_callback=probe.feedback)
                        probe.until(future.done, 6., 'Ground maneuver acceptance')
                        probe.goal_handle = future.result()
                        require(probe.goal_handle.accepted, 'Follower rejected ground maneuver')
                        outcome['goal_id'] = [int(v) for v in probe.goal_handle.goal_id.uuid]
                        accepted_at = time.monotonic()
                        probe.event('goal_accepted', goal_id=outcome['goal_id'])
                        probe.result_future = probe.goal_handle.get_result_async()
                        if case == 'cancel':
                            probe.until(lambda: (time.monotonic()-accepted_at >= args.cancel_after and
                                max(abs(probe.command[0]), abs(probe.command[1])) >= .15)
                                or probe.result_future.done(), min(args.case_timeout, 20.),
                                'Wait for nonzero real controller output before cancel')
                            require(not probe.result_future.done(), 'Action ended before deliberate moving cancel')
                            probe.phase = 'cancelling'
                            cancel = probe.event('cancel_requested', pose=probe.pose,
                                odometry_stamp_ns=probe.last_odom_ns, command=probe.command)
                            outcome['cancel_requested'] = cancel
                            future = probe.goal_handle.cancel_goal_async()
                            probe.until(future.done, 3., 'Cancellation acknowledgement')
                            answer = future.result()
                            cancelled_ids = [[int(v) for v in info.goal_id.uuid]
                                             for info in answer.goals_canceling]
                            require(answer.return_code == 0 and outcome['goal_id'] in cancelled_ids,
                                    'Input action did not acknowledge requested cancellation')
                            probe.event('cancel_acknowledged', return_code=int(answer.return_code),
                                        goals_canceling=cancelled_ids)
                        remaining = max(.1, args.case_timeout-(time.monotonic()-accepted_at))
                        probe.until(probe.result_future.done, remaining, 'Ground maneuver terminal result')
                        reply = probe.result_future.result()
                        observed_result = probe.event('action_result', status=int(reply.status),
                            success=bool(reply.result.success), pose=probe.pose, command=probe.command,
                            odometry_stamp_ns=probe.last_odom_ns)
                        outcome['result'] = observed_result
                        expected_status = GoalStatus.STATUS_SUCCEEDED if case == 'complete' else GoalStatus.STATUS_CANCELED
                        require(reply.status == expected_status and reply.result.success == (case == 'complete'),
                                'Unexpected input action result: '+str(reply.status))
                        # Require several NEW measurements after the result,
                        # and then a separate no-drift interval. The result or
                        # a last zero command is never sufficient evidence.
                        probe.phase = 'observing_stop'
                        probe.until(lambda: probe.stationary_after(observed_result['odometry_stamp_ns']),
                                    3., 'Fresh post-result stationary odometry')
                        stable_pose = probe.pose
                        probe.spin_for(.3)
                        require(probe.stationary_after(observed_result['odometry_stamp_ns']),
                                'Motion resumed after terminal result')
                        require(math.dist(stable_pose[:3], probe.pose[:3]) <= 1.e-4 and
                                abs(wrap(stable_pose[3]-probe.pose[3])) <= 1.e-4,
                                'Settled pose drifted')
                        execution = probe.snapshot(probe.execution_state)
                        tracking = probe.snapshot(probe.tracking_state)
                        terminal = execution.get('terminal_outcome') or {}
                        require(terminal.get('goal_id') == outcome['goal_id'] and
                                terminal.get('dispatched') and terminal.get('stop_confirmed') and
                                terminal.get('output_status') == expected_status,
                                'Follower lacks matching downstream terminal AND observed-stop evidence')
                        require(terminal.get('confirmation') != 'not_dispatched', 'Goal never reached tracker')
                        require(tracking.get('execution_mode') == PathFollower.Goal.GROUND_MANEUVER and
                                tracking.get('ground_maneuver', {}).get('stop_confirmed') is True,
                                'Tracker did not report this explicit maneuver mode and observed stop')
                        require(probe.trace['validation_requests'], 'Follower never called validation fixture')
                        require(any(abs(row['twist'][0]) > .01 or abs(row['twist'][5]) > .01
                                    for row in probe.trace['commands']), 'No real controller motion observed')
                        xy_error = math.dist(probe.pose[:2], target[:2])
                        yaw_error = abs(wrap(probe.pose[3]-target[3]))
                        if case == 'complete':
                            require(xy_error <= .04 and yaw_error <= .1,
                                    'Completion outside original XY/yaw thresholds')
                            require(tracking['ground_maneuver']['segment_index'] == len(candidate.primitives)-1,
                                    'Tracker did not finish the declared final primitive')
                            if crossing_count:
                                require(tracking['ground_maneuver']['pass_through_total'] == crossing_count
                                    and tracking['ground_maneuver']['pass_through_consumed'] == crossing_count,
                                    'Tracker did not observe every declared crossing')
                        else:
                            after_cancel = [row for row in probe.trace['commands']
                                            if row['wall_s'] >= cancel['wall_s']]
                            initial_w = abs(cancel['command'][1])
                            initial_v = abs(cancel['command'][0])
                            require(any(0. < abs(row['twist'][5]) < initial_w-1.e-6 or
                                        0. < abs(row['twist'][0]) < initial_v-1.e-6
                                        for row in after_cancel),
                                    'Cancellation showed no bounded nonzero braking command')
                            outcome['cancel_to_result_wall_s'] = observed_result['wall_s']-cancel['wall_s']
                        rows = probe.trace['odometry']
                        elapsed = rows[-1]['wall_s']-rows[0]['wall_s']
                        gaps = [(b['stamp_ns']-a['stamp_ns'])*1.e-9 for a, b in zip(rows, rows[1:])]
                        outcome.update(status='passed', final_pose=probe.pose,
                            endpoint_xy_error_m=xy_error, endpoint_yaw_error_rad=yaw_error,
                            terminal_execution=execution, terminal_tracking=tracking,
                            stop_evidence=rows[-5:], observed_odom_hz=(len(rows)-1)/elapsed,
                            maximum_odometry_gap_s=max(gaps), action_wall_s=time.monotonic()-accepted_at,
                            checks={'real_follower_and_tracker': True, 'real_action_round_trip': True,
                                'fixture_validator_called': True, 'only_tracker_drives_feedback': True,
                                'command_speed_and_step_caps': not probe.failures,
                                'advancing_post_terminal_odometry': True,
                                'observed_stop_and_zero_command': True,
                                'stationary_pose_persisted': True})
                    finally:
                        outcome['processes'] = cleanup(probe)
                        if any(item['exit_code'] != 0 or item['signals'] != ['SIGINT']
                               for item in outcome['processes']):
                            outcome['status'] = 'failed'
                            outcome['cleanup_error'] = 'Owned nodes did not exit normally after SIGINT'
            except Exception as error:
                outcome.update(status='failed', error=str(error), traceback=traceback.format_exc(),
                               final_pose=probe.pose, last_command=probe.command,
                               last_execution=probe.execution_status, last_tracking=probe.tracking_status)
            finally:
                traces[case] = probe.trace
                probe.destroy_node()
            if outcome['status'] != 'passed':
                break
        report['status'] = ('passed' if len(report['cases']) == len(cases) and
                            all(case['status'] == 'passed' for case in report['cases']) else 'failed')
    except Exception as error:
        report.update(error=str(error), traceback=traceback.format_exc())
    finally:
        if ros is not None and ros.ok():
            ros.shutdown()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
        report['wall_elapsed_s'] = time.monotonic()-started
        # Preserve the actual execution verdict independently BEFORE touching
        # the much larger trace. A trace encoding/write failure must not erase
        # successful/cancelled Action outcomes or their stop evidence.
        report['protocol_status'] = report['status']
        report['trace_status'] = 'pending'
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        try:
            trace_text = json.dumps(traces, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
            trace_path.write_text(trace_text)
            report['trace_status'] = 'saved'
        except Exception as error:
            report['status'] = 'failed'
            report['trace_status'] = 'failed'
            report['trace_error'] = type(error).__name__+': '+str(error)
        finally:
            args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'status': report['status'], 'output': str(args.output),
                      'trace': str(trace_path), 'cases': [
                          {'case': row['case'], 'status': row['status'], 'error': row.get('error')}
                          for row in report['cases']]}, ensure_ascii=False))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
