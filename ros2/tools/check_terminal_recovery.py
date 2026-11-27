#!/usr/bin/env python3
"""Check native terminal-recovery Action commands with controlled fresh odometry.

The supplied poses deliberately jump between observations. Commands are recorded,
not integrated into a vehicle: this checks ROS control logic, not contact dynamics
or physical convergence. The original 0.04 m / 0.1 rad success criteria remain.
"""
import argparse
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    started = time.monotonic()
    # Reserve two seconds for bounded process cleanup and artifact writes.
    deadline = started + 18.
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))
    log_path = args.output.with_suffix('.log')
    odometry_path = args.output.with_suffix('.csv')
    command_path = args.output.with_name(args.output.stem + '_commands.csv')
    summary = {
        'status': 'failed', 'domain': os.environ['ROS_DOMAIN_ID'],
        'scope': 'native ROS PathFollower Action, tracker and raw cmd_vel; '
                 'controlled fresh odometry; no vehicle or contact dynamics',
        'odometry_method': 'held artificial poses with new source timestamps; '
                           'commands do not move the fixture',
        'success_tolerances': {'position_m': .04, 'yaw_rad': .1},
        'execution_budget_s': 18., 'checks': {},
    }
    process = None
    node = None
    rclpy = None

    try:
        import rclpy
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import Pose, PoseStamped, Twist
        from nav_msgs.msg import Odometry
        from planner_msgs.action import PathFollower
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from std_srvs.srv import Trigger

        goal_pose = (0., 0., 1.2)

        def pose(values):
            message = Pose()
            message.position.x, message.position.y = float(values[0]), float(values[1])
            message.orientation.z = math.sin(values[2] / 2.)
            message.orientation.w = math.cos(values[2] / 2.)
            return message

        class Probe(Node):
            def __init__(self):
                super().__init__('terminal_recovery_probe')
                self.phase = 'discovery'
                self.position = [-.001, .061, 0.]
                self.commands, self.odometry = [], []
                self.carrots = []
                self.current_result = None
                self.odom = self.create_publisher(Odometry, 'tracking_odometry', 10)
                self.create_subscription(Twist, 'tracking_cmd_vel', self.command, 10)
                self.create_subscription(PoseStamped, 'tracking_carrot', self.carrot, 10)
                self.action = ActionClient(self, PathFollower, 'pci_output_path')
                self.tracker = self.create_client(Trigger, 'tracking_state')
                self.create_timer(.01, self.publish_pose)

            def publish_pose(self):
                stamp = self.get_clock().now()
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                message.header.stamp = stamp.to_msg()
                message.pose.pose = pose(self.position)
                # Each artificial observation describes a stationary chassis.
                # No velocity derived from commanded motion is fed back.
                self.odom.publish(message)
                self.odometry.append((time.monotonic() - started, self.phase,
                                      stamp.nanoseconds, *self.position))
                return stamp.nanoseconds

            def command(self, message):
                values = (message.linear.x, message.linear.y, message.linear.z,
                          message.angular.x, message.angular.y, message.angular.z)
                require(all(math.isfinite(v) for v in values), 'Nonfinite cmd_vel')
                require(abs(values[0]) <= .500001 and abs(values[5]) <= .500001,
                        f'Command exceeded configured limits: {values}')
                require(all(abs(values[i]) < 1.e-10 for i in (1, 2, 3, 4)),
                        f'Nonplanar cmd_vel: {values}')
                self.commands.append((time.monotonic() - started, self.phase,
                                      values[0], values[5]))

            def carrot(self, message):
                stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                self.carrots.append((time.monotonic() - started, stamp,
                                     message.pose.position.x, message.pose.position.y,
                                     message.header.frame_id))

            def until(self, predicate, label, seconds=2.):
                stop_at = min(deadline, time.monotonic() + seconds)
                while not predicate():
                    if process.poll() is not None:
                        raise RuntimeError(f'Tracker exited {process.returncode}; see {log_path}')
                    if time.monotonic() >= stop_at:
                        raise TimeoutError(f'{self.phase}: {label}')
                    rclpy.spin_once(self, timeout_sec=.01)

            def spin_for(self, seconds):
                stop_at = time.monotonic() + seconds
                self.until(lambda: time.monotonic() >= stop_at,
                           'Observation interval', seconds + .1)

            def state(self):
                future = self.tracker.call_async(Trigger.Request())
                self.until(future.done, 'Tracker state response')
                reply = future.result()
                require(reply.success, reply.message)
                return json.loads(reply.message)

            def send(self):
                request = PathFollower.Goal()
                request.vel_max, request.bound_mode, request.manual_path = .4, 3, True
                endpoint = PoseStamped()
                endpoint.header.frame_id = 'map'
                endpoint.header.stamp = self.get_clock().now().to_msg()
                endpoint.pose = pose(goal_pose)
                request.path = [endpoint]
                future = self.action.send_goal_async(request)
                self.until(future.done, 'Goal acceptance')
                handle = future.result()
                require(handle.accepted, 'Native action rejected the fixture goal')
                self.current_result = handle.get_result_async()
                return handle, self.current_result

            def observe(self, phase, position, state, direction, turn=None, linear=None):
                self.phase = phase
                self.position[:] = position
                stamp = self.publish_pose()
                c, s = math.cos(position[2]), math.sin(position[2])
                dx, dy = goal_pose[0] - position[0], goal_pose[1] - position[1]
                body_x, body_y = c * dx + s * dy, -s * dx + c * dy
                # The tracker stamps its carrot with the odometry sample it
                # actually consumed. This avoids checking the previous pose's
                # command after merely publishing a new odometry message.
                self.until(lambda: any(row[1] >= stamp and row[4] == 'base_link'
                                       and abs(row[2] - body_x) < 1.e-8
                                       and abs(row[3] - body_y) < 1.e-8
                                       for row in self.carrots[-20:]),
                           'Control tick consuming the new source pose')
                # Let one complete timer period pass before the command window;
                # separate DDS topics need not arrive in publication order.
                self.spin_for(.06)
                before = len(self.commands)
                odom_before = len(self.odometry)
                self.spin_for(.15)
                observed = self.state()
                commands = self.commands[before:]
                require(len(commands) >= 2, 'Fewer than two fresh control commands')
                require(len(self.odometry) > odom_before, 'No fresh odometry during observation')
                require(not self.current_result.done(), 'Goal ended before strict terminal observation')
                require(observed['active'] and observed['state'] == state, observed)
                require('position_correction_direction' in observed, 'Missing direction observability')
                require(observed['position_correction_direction'] == direction, observed)
                require(abs(observed['dist_to_goal'] - math.hypot(dx, dy)) < 1.e-7
                        and abs(wrap(observed['yaw_error'] - wrap(goal_pose[2] - position[2]))) < 1.e-7,
                        f'State does not match the controlled pose: {observed}')
                require(observed['position_tolerance'] == .04
                        and observed['yaw_tolerance'] == .1
                        and abs(observed['alignment_exit_distance'] - .06) < 1.e-12,
                        'Terminal tolerances changed')
                for _, _, v, w in commands:
                    if turn is not None:
                        require(turn * w > .49, f'{phase}: turn changed sign: v={v}, w={w}')
                    if linear == 0:
                        require(abs(v) < 1.e-10, f'{phase}: expected the initial turn before translation')
                    elif linear is not None:
                        require(linear * v > 1.e-4, f'{phase}: no requested translation in locked direction')
                last = observed['last_command']
                if turn is not None:
                    require(turn * last['angular_z'] > .49, observed)
                if linear == 0:
                    require(abs(last['linear_x']) < 1.e-10, observed)
                elif linear is not None:
                    require(linear * last['linear_x'] > 1.e-4, observed)
                return {'supplied_pose': list(position), 'body_goal_xy_m': [body_x, body_y],
                        'first_source_stamp_ns': stamp, 'fresh_odom_samples': len(self.odometry) - odom_before,
                        'raw_commands': [[v, w] for _, _, v, w in commands], 'tracker': observed}

            def cancel(self, handle, result):
                self.phase = 'cancel_locked_recovery'
                before = len(self.commands)
                future = handle.cancel_goal_async()
                self.until(future.done, 'Cancellation acknowledgement')
                require(bool(future.result().goals_canceling), 'Cancellation rejected')
                self.until(result.done, 'Cancelled native action result')
                reply = result.result()
                require(reply.status == GoalStatus.STATUS_CANCELED and not reply.result.success,
                        f'Unexpected cancellation result: {reply.status}')
                self.until(lambda: any(abs(v) < 1.e-10 and abs(w) < 1.e-10
                                       for _, _, v, w in self.commands[before:]),
                           'Raw zero command after cancellation')
                state = self.state()
                require(state['state'] == 'CANCELED' and not state['active']
                        and state['position_correction_direction'] is None, state)
                require(state['last_command'] == {'linear_x': 0, 'angular_z': 0}, state)
                return {'native_action_status': reply.status, 'tracker': state,
                        'raw_zero_command': True}

        rclpy.init(args=[])
        node = Probe()
        parameters = {
            'fixed_frame': 'map', 'robot_frame': 'base_link', 'dt': .05,
            'desired_vel': .5, 'max_lin_vel': .5, 'max_ang_vel': .5,
            'look_ahead_distance': .15, 'goal_reaching_threshold': .04,
            'goal_yaw_threshold': .1, 'odom_timeout': .5,
        }
        command = ['ros2', 'run', 'gbplanner_tracking', 'pure_pursuit_node', '--ros-args',
                   '-r', 'cmd_vel:=tracking_cmd_vel', '-r', 'odometry:=tracking_odometry',
                   '-r', 'carrot_point:=tracking_carrot']
        for name, value in parameters.items():
            command += ['-p', f'{name}:={value}']
        summary['launch'] = command
        with log_path.open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=dict(os.environ))
            node.until(lambda: node.action.server_is_ready() and node.tracker.service_is_ready()
                       and node.odom.get_subscription_count() > 0, 'ROS discovery', 7.)
            node.spin_for(.1)
            handle, result = node.send()
            initial = node.observe('initial_forward_choice', (-.001, .061, 0.),
                                   'TRACKING', None, turn=-1, linear=0)
            initial_reverse = node.observe('initial_reverse_choice', (.001, .061, 0.),
                                           'TRACKING', None, turn=1, linear=0)
            summary['checks']['ordinary_tracking_unlocked'] = [initial, initial_reverse]
            summary['checks']['enter_alignment'] = node.observe(
                'first_alignment', (-.039, 0., 0.), 'ALIGNING', None)
            summary['checks']['enter_forward_recovery'] = node.observe(
                'forward_recovery', (-.001, .061, 0.), 'TRACKING', 1, turn=-1, linear=0)
            jitter = []
            for index in range(6):
                # Goal body-x alternates -1/+1 mm, while the endpoint stays
                # outside 0.04 m and inside the previous 0.06 m exit band.
                x = .001 if index % 2 == 0 else -.001
                jitter.append(node.observe(f'locked_jitter_{index + 1}', (x, .05, 0.),
                                           'TRACKING', 1, turn=-1, linear=0))
            summary['checks']['six_fresh_lateral_jitter_observations'] = jitter
            summary['checks']['requested_forward_translation'] = node.observe(
                'forward_after_rotation', (0., .05, -1.1), 'TRACKING', 1, linear=1)
            summary['checks']['alignment_clears_direction'] = node.observe(
                'alignment_again', (-.039, 0., 0.), 'ALIGNING', None)
            # A new episode may choose the opposite direction. Cancel while
            # that lock is nonempty, so cancellation is not a vacuous reset.
            summary['checks']['new_episode_can_choose_reverse'] = node.observe(
                'reverse_recovery', (.001, .061, 0.), 'TRACKING', -1, turn=1, linear=0)
            summary['checks']['cancel_clears_direction'] = node.cancel(handle, result)
            node.phase = 'new_goal'
            node.position[:] = [-.001, .05, 0.]
            node.publish_pose()
            node.spin_for(.1)
            _, new_result = node.send()
            fresh = node.observe('new_goal_unlocked', (-.001, .05, 0.),
                                 'TRACKING', None, turn=-1, linear=0)
            require(fresh['tracker']['goal_id'] != initial['tracker']['goal_id'],
                    'New goal reused the old identity')
            summary['checks']['new_goal_does_not_inherit_lock'] = fresh
            # Complete only after one supplied pose satisfies both unchanged
            # tolerances. This proves Action semantics, not physical arrival.
            node.phase = 'strict_terminal_pose'
            node.position[:] = [-.039, 0., goal_pose[2]]
            before = len(node.commands)
            node.publish_pose()
            node.until(new_result.done, 'Success at a pose satisfying both tolerances')
            reply = new_result.result()
            require(reply.status == GoalStatus.STATUS_SUCCEEDED and reply.result.success,
                    f'Unexpected strict terminal result: {reply.status}')
            node.until(lambda: any(abs(v) < 1.e-10 and abs(w) < 1.e-10
                                   for _, _, v, w in node.commands[before:]),
                       'Raw zero command after success')
            final = node.state()
            require(final['state'] == 'SUCCEEDED' and not final['active']
                    and final['position_correction_direction'] is None
                    and final['dist_to_goal'] <= .04 and abs(final['yaw_error']) <= .1,
                    final)
            summary['checks']['strict_terminal_action_success'] = {
                'native_action_status': reply.status, 'supplied_pose': list(node.position),
                'physical_arrival_observed': False, 'tracker': final, 'raw_zero_command': True}
            summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
    finally:
        if process is not None:
            for requested_signal, seconds in ((signal.SIGINT, .8), (signal.SIGTERM, .4),
                                               (signal.SIGKILL, .4)):
                if process.poll() is not None:
                    break
                try:
                    os.killpg(process.pid, requested_signal)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=seconds)
                except subprocess.TimeoutExpired:
                    continue
        if node is not None:
            with odometry_path.open('w', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(['time_s', 'phase', 'source_stamp_ns', 'x_m', 'y_m', 'yaw_rad'])
                writer.writerows(node.odometry)
            with command_path.open('w', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(['time_s', 'phase', 'linear_x_mps', 'angular_z_radps'])
                writer.writerows(node.commands)
            summary['odometry_samples'] = len(node.odometry)
            summary['command_samples'] = len(node.commands)
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        summary['elapsed_s'] = round(time.monotonic() - started, 3)
        summary['processes_stopped'] = process is None or process.poll() is not None
        if not summary['processes_stopped']:
            summary['status'] = 'failed'
            summary['cleanup_error'] = 'Tracker process did not exit after bounded cleanup'
        summary['artifacts'] = {'odometry': str(odometry_path), 'commands': str(command_path),
                                'log': str(log_path)}
        args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(summary, ensure_ascii=False))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
