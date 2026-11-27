#!/usr/bin/env python3
"""Short ROS action/cmd_vel check using independent planar kinematic odometry."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))

    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import Pose, PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from planner_msgs.action import PathFollower
    from planner_msgs.srv import ValidatePath
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    def pose(state):
        value = Pose()
        value.position.x, value.position.y = float(state[0]), float(state[1])
        value.orientation.z = math.sin(state[2] / 2.)
        value.orientation.w = math.cos(state[2] / 2.)
        return value

    started = time.monotonic()
    processes = []
    node = None
    summary = {
        'status': 'failed', 'domain': os.environ['ROS_DOMAIN_ID'],
        'scope': 'real ROS actions and cmd_vel with independent differential-drive '
                 'kinematic odometry; no wheel/contact dynamics',
        'checks': {},
    }
    trace_path = args.output.with_suffix('.csv')
    log_path = args.output.with_suffix('.log')
    command_path = args.output.with_name(args.output.stem + '_commands.csv')

    class Probe(Node):
        def __init__(self):
            super().__init__('pure_pursuit_probe')
            self.position = [0., 0., 0.]
            self.command = (0., 0.)
            self.last_update = time.monotonic()
            self.phase = 'discovery'
            self.publish_odometry = True
            self.commands, self.trace, self.feedback = [], [], []
            self.execution = {}
            self.validation_count = 0
            self.odom = self.create_publisher(Odometry, 'tracking_odometry', 10)
            self.create_subscription(Twist, 'tracking_cmd_vel', self.receive_command, 10)
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.maps = self.create_publisher(String, 'map_updates', durable)
            self.create_subscription(String, 'execution_status', self.receive_execution, durable)
            self.create_service(ValidatePath, 'validate_path', self.validate)
            self.direct = ActionClient(self, PathFollower, 'pci_output_path')
            self.bridge = ActionClient(self, PathFollower, 'path_follower')
            self.tracker_state = self.create_client(Trigger, 'tracking_state')
            self.execution_state = self.create_client(Trigger, 'execution_state')
            self.create_timer(.01, self.tick)

        def advance(self, now):
            # Exact constant-Twist unicycle integration, independent of the
            # tracker implementation and of the requested goal coordinates.
            dt = now - self.last_update
            x, y, yaw = self.position
            velocity, omega = self.command
            next_yaw = yaw + omega * dt
            if abs(omega) > 1.e-10:
                x += velocity / omega * (math.sin(next_yaw) - math.sin(yaw))
                y += velocity / omega * (math.cos(yaw) - math.cos(next_yaw))
            else:
                x += velocity * math.cos(yaw) * dt
                y += velocity * math.sin(yaw) * dt
            self.position[:] = [x, y, wrap(next_yaw)]
            self.last_update = now

        def receive_command(self, message):
            values = (message.linear.x, message.linear.y, message.linear.z,
                      message.angular.x, message.angular.y, message.angular.z)
            require(all(math.isfinite(value) for value in values), 'Non-finite cmd_vel')
            require(abs(values[0]) <= .500001 and abs(values[5]) <= .500001,
                    f'Command exceeded configured limits: {values}')
            require(all(abs(values[index]) < 1.e-10 for index in (1, 2, 3, 4)),
                    f'Non-planar cmd_vel: {values}')
            now = time.monotonic()
            self.advance(now)
            self.command = (values[0], values[5])
            self.commands.append((now - started, self.phase, *self.command))

        def receive_execution(self, message):
            self.execution = json.loads(message.data)

        def validate(self, request, response):
            self.validation_count += 1
            response.valid = True
            response.reason = 'free_kinematic_test_fixture'
            response.map_revision = 1
            return response

        def tick(self):
            now = time.monotonic()
            self.advance(now)
            self.trace.append((now - started, self.phase, *self.position,
                               *self.command, int(self.publish_odometry)))
            if self.publish_odometry:
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                message.header.stamp = self.get_clock().now().to_msg()
                message.pose.pose = pose(self.position)
                message.twist.twist.linear.x, message.twist.twist.angular.z = self.command
                self.odom.publish(message)
            update = String()
            update.data = '{"map_revision":1,"known_cells":100}'
            self.maps.publish(update)

        def until(self, predicate, label, seconds=5.):
            deadline = min(time.monotonic() + seconds, started + 45.)
            while not predicate():
                for process in processes:
                    if process.poll() is not None:
                        raise RuntimeError(f'ROS node exited {process.returncode}; see {log_path}')
                if time.monotonic() > deadline:
                    raise TimeoutError(f'{self.phase}: {label}; pose={self.position}; '
                                       f'cmd_vel={self.command}; bridge={self.execution}')
                rclpy.spin_once(self, timeout_sec=.01)

        def spin_for(self, seconds):
            end = time.monotonic() + seconds
            self.until(lambda: time.monotonic() >= end, 'Observation interval', seconds + .5)

        def state(self, client):
            self.until(client.service_is_ready, 'State service discovery')
            future = client.call_async(Trigger.Request())
            self.until(future.done, 'State service response')
            response = future.result()
            require(response.success, response.message)
            return json.loads(response.message)

        def reset(self, phase):
            # Resets occur only between terminal goals after raw zero output.
            require(max(map(abs, self.command)) < 1.e-10, 'Previous goal has not stopped')
            self.phase = phase
            self.position[:] = [0., 0., 0.]
            self.last_update = time.monotonic()
            self.feedback.clear()
            self.publish_odometry = True
            self.spin_for(.12)

        def send(self, client, points, speed=.4):
            goal = PathFollower.Goal()
            goal.manual_path, goal.bound_mode, goal.vel_max = True, 3, speed
            for state in points:
                stamped = PoseStamped()
                stamped.header.frame_id = 'map'
                stamped.header.stamp = self.get_clock().now().to_msg()
                stamped.pose = pose(state)
                goal.path.append(stamped)
            self.feedback.clear()
            future = client.send_goal_async(
                goal, feedback_callback=lambda msg: self.feedback.append(msg.feedback))
            self.until(future.done, 'Goal acceptance')
            handle = future.result()
            require(handle.accepted, f'{self.phase} goal rejected')
            return handle, handle.get_result_async()

        def stopped(self, command_start):
            self.until(lambda: any(abs(v) < 1.e-10 and abs(w) < 1.e-10
                                   for _, _, v, w in self.commands[command_start:])
                       and max(map(abs, self.command)) < 1.e-10,
                       'Raw zero cmd_vel after terminal/cancel', 1.)
            still = tuple(self.position)
            self.spin_for(.15)
            require(math.dist(still[:2], self.position[:2]) < .001
                    and abs(wrap(still[2] - self.position[2])) < .001,
                    'Motion resumed after zero cmd_vel')

        def complete(self, result, endpoint, command_start):
            self.until(result.done, 'Goal completion', 12.)
            response = result.result()
            require(response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success,
                    f'{self.phase} result: {response}')
            self.stopped(command_start)
            distance = math.dist(self.position[:2], endpoint[:2])
            yaw_error = abs(wrap(self.position[2] - endpoint[2]))
            require(distance <= .055 and yaw_error <= .115,
                    f'Observed arrival outside tolerance: {distance}, {yaw_error}')
            require(bool(self.feedback), 'No native action feedback received')
            return {'endpoint': list(endpoint), 'observed_pose': list(self.position),
                    'position_error_m': distance, 'yaw_error_rad': yaw_error,
                    'feedback_messages': len(self.feedback),
                    'raw_zero_command_and_stationary_odometry': True}

    def launch(log, package, executable, parameters):
        command = ['ros2', 'run', package, executable, '--ros-args',
                   '-r', 'cmd_vel:=tracking_cmd_vel', '-r', 'odometry:=tracking_odometry']
        for name, value in parameters.items():
            command += ['-p', f'{name}:={value}']
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=dict(os.environ))
        processes.append(process)

    def stop(process):
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=1.)

    try:
        rclpy.init(args=[])
        node = Probe()
        with log_path.open('w') as log:
            launch(log, 'gbplanner_tracking', 'pure_pursuit_node', {
                'fixed_frame': 'map', 'robot_frame': 'base_link', 'dt': .05,
                'desired_vel': .5, 'max_lin_vel': .5, 'max_ang_vel': .5,
                'look_ahead_distance': .15, 'goal_reaching_threshold': .04,
                'goal_yaw_threshold': .1, 'odom_timeout': .5,
            })
            node.until(lambda: node.direct.server_is_ready()
                       and node.tracker_state.service_is_ready()
                       and node.odom.get_subscription_count() > 0, 'Tracker discovery', 10.)
            node.spin_for(.15)
            initial = node.state(node.tracker_state)
            require({'state', 'goal_id', 'remaining_waypoints', 'dist_to_goal',
                     'yaw_error', 'odom_age', 'last_command'} <= initial.keys(), initial)
            summary['tracking_state_fields'] = sorted(initial)

            node.reset('curve_and_final_yaw')
            route = [(0., 0., 0.), (.18, 0., 0.), (.35, .08, .4), (.5, .2, 1.1)]
            before = len(node.commands)
            _, result = node.send(node.direct, route)
            record = node.complete(result, route[-1], before)
            commands = node.commands[before:]
            require(any(v > .01 and abs(w) > .01 for _, _, v, w in commands),
                    'Curved route did not produce simultaneous translation and turn')
            require(any(abs(v) < 1.e-10 and abs(w) > .01 for _, _, v, w in commands),
                    'Final yaw did not produce an in-place turn')
            record['route'] = route
            record['tracker'] = node.state(node.tracker_state)
            summary['checks']['curve_and_final_yaw'] = record

            node.reset('reverse')
            route = [(0., 0., 0.), (-.3, 0., 0.)]
            before = len(node.commands)
            _, result = node.send(node.direct, route)
            record = node.complete(result, route[-1], before)
            reverse_commands = [v for _, _, v, _ in node.commands[before:] if v < -.01]
            require(bool(reverse_commands), 'Reverse route did not produce negative linear velocity')
            record['reverse_commands'] = len(reverse_commands)
            summary['checks']['reverse'] = record

            node.reset('cancel')
            before = len(node.commands)
            handle, result = node.send(node.direct, [(0., 0., 0.), (1.5, 0., 0.)])
            node.until(lambda: any(abs(v) > .05 for _, _, v, _ in node.commands[before:]),
                       'Motion before cancellation')
            cancel_at = time.monotonic()
            cancel_commands = len(node.commands)
            cancellation = handle.cancel_goal_async()
            node.until(cancellation.done, 'Cancel acknowledgement')
            require(bool(cancellation.result().goals_canceling), 'Cancel request rejected')
            node.until(result.done, 'Cancelled terminal action')
            require(result.result().status == GoalStatus.STATUS_CANCELED
                    and not result.result().result.success, result.result())
            node.stopped(cancel_commands)
            summary['checks']['cancel'] = {
                'terminal_status': result.result().status,
                'elapsed_s': time.monotonic() - cancel_at,
                'raw_zero_command_and_stationary_odometry': True,
            }

            node.reset('odometry_dropout')
            before = len(node.commands)
            _, result = node.send(node.direct, [(0., 0., 0.), (1.5, 0., 0.)])
            node.until(lambda: any(abs(v) > .05 for _, _, v, _ in node.commands[before:]),
                       'Motion before odometry dropout')
            dropout_at = time.monotonic()
            dropout_commands = len(node.commands)
            node.publish_odometry = False
            # Continue integrating/capturing commands while withholding only
            # odometry; no consumer watchdog can hide a missing stop command.
            node.until(result.done, 'Abort after stale odometry', 2.)
            require(result.result().status == GoalStatus.STATUS_ABORTED
                    and not result.result().result.success, result.result())
            abort_after = time.monotonic() - dropout_at
            node.stopped(dropout_commands)
            summary['checks']['odometry_dropout'] = {
                'terminal_status': result.result().status, 'abort_after_s': abort_after,
                'configured_timeout_s': .5,
                'raw_zero_command_and_stationary_odometry': True,
                'tracker': node.state(node.tracker_state),
            }
            node.publish_odometry = True

            node.reset('path_follower_bridge')
            launch(log, 'gbplanner_execution', 'path_follower_node.py', {
                'output_type': 'kAction', 'robot_type': 'kGround',
                'sample_period': .05, 'speed': .4, 'execution_grace': 3.,
            })
            node.until(lambda: node.bridge.server_is_ready()
                       and node.execution_state.service_is_ready()
                       and node.execution.get('output_ready')
                       and node.odom.get_subscription_count() >= 2,
                       'Ground action bridge discovery', 8.)
            node.spin_for(.15)
            before = len(node.commands)
            validations = node.validation_count
            start = tuple(node.position)
            route = [start, (start[0] + .3, start[1], start[2])]
            _, result = node.send(node.bridge, route)
            record = node.complete(result, route[-1], before)
            state = node.state(node.execution_state)
            require(node.validation_count > validations, 'Bridge bypassed path validation')
            require(state['action_commands'] == 1 and state['succeeded'] == 1
                    and state['trajectory_commands'] == 0, state)
            record['execution'] = state
            record['validation_calls'] = node.validation_count - validations
            record['validation_scope'] = 'all-free fixture, not a map/collision test'
            summary['checks']['path_follower_bridge'] = record
            summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
    finally:
        for process in reversed(processes):
            stop(process)
        if node is not None:
            with trace_path.open('w', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(['time_s', 'phase', 'x_m', 'y_m', 'yaw_rad',
                                 'linear_x_mps', 'angular_z_radps', 'odometry_published'])
                writer.writerows(node.trace)
            with command_path.open('w', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(['time_s', 'phase', 'linear_x_mps', 'angular_z_radps'])
                writer.writerows(node.commands)
            summary['command_count'] = len(node.commands)
            summary['trace_samples'] = len(node.trace)
            summary['max_abs_linear_x_mps'] = max((abs(row[2]) for row in node.commands), default=0.)
            summary['max_abs_angular_z_radps'] = max((abs(row[3]) for row in node.commands), default=0.)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        summary['elapsed_s'] = round(time.monotonic() - started, 3)
        summary['processes_stopped'] = all(process.poll() is not None for process in processes)
        summary['artifacts'] = {'trace': str(trace_path), 'commands': str(command_path), 'log': str(log_path)}
        args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(summary, ensure_ascii=False))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
