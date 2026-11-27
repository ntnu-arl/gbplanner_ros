#!/usr/bin/env python3
"""Native SMB/Bullet route tracking, or the recorded short terminal-turn case."""
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
    parser.add_argument('--terminal-turn', action='store_true',
                        help='Exercise the run8 short reverse route and final yaw only')
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))

    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from planner_msgs.action import PathFollower
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    started = time.monotonic()
    processes = []
    node = None
    summary = {
        'status': 'failed', 'domain': os.environ['ROS_DOMAIN_ID'],
        'scope': 'pci_output_path → pure_pursuit_node → cmd_vel → SMB Bullet wheel '
                 'motors/contact dynamics → actual odometry; no synthetic odometry',
        'checks': {},
        'case': 'run8_terminal_turn' if args.terminal_turn else 'bent_route_and_timeout',
    }
    log_path = args.output.with_suffix('.log')
    trace_path = args.output.with_suffix('.csv')
    commands_path = args.output.with_name(args.output.stem + '_commands.csv')
    states_path = args.output.with_name(args.output.stem + '_states.json')

    class Probe(Node):
        def __init__(self):
            super().__init__('smb_tracking_probe')
            self.phase = 'settling'
            self.position = None
            self.speed = math.inf
            self.yaw_rate = math.inf
            self.odom_received = 0.
            self.status = {}
            self.states, self.trace, self.commands, self.feedback = [], [], [], []
            self.tracking_states = []
            self.last_command = None
            self.create_subscription(Odometry, 'odometry', self.odometry, qos_profile_sensor_data)
            self.create_subscription(Twist, 'cmd_vel', self.command, 10)
            self.create_subscription(String, 'smb_status', self.receive_status, 10)
            self.create_subscription(String, 'tracking_status', self.receive_tracking, 10)
            self.pulse = self.create_publisher(Twist, 'cmd_vel', 10)
            self.action = ActionClient(self, PathFollower, 'pci_output_path')
            self.smb_state = self.create_client(Trigger, 'smb_state')
            self.tracking_state = self.create_client(Trigger, 'tracking_state')

        def odometry(self, message):
            require(message.header.frame_id == 'map' and message.child_frame_id == 'base_link',
                    f'Unexpected odometry frames: {message.header.frame_id}/{message.child_frame_id}')
            p, q = message.pose.pose.position, message.pose.pose.orientation
            yaw = math.atan2(2. * (q.w*q.z + q.x*q.y), 1. - 2. * (q.y*q.y + q.z*q.z))
            linear, angular = message.twist.twist.linear, message.twist.twist.angular
            values = [p.x, p.y, p.z, yaw, linear.x, linear.y, linear.z,
                      angular.x, angular.y, angular.z]
            require(all(math.isfinite(v) for v in values), 'Non-finite Bullet odometry')
            self.position = (p.x, p.y, p.z, yaw)
            self.speed = math.sqrt(linear.x**2 + linear.y**2 + linear.z**2)
            self.yaw_rate = angular.z
            self.odom_received = time.monotonic()
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1.e-9
            self.trace.append((self.odom_received-started, self.phase, stamp, *values))

        def command(self, message):
            values = (message.linear.x, message.linear.y, message.linear.z,
                      message.angular.x, message.angular.y, message.angular.z)
            require(all(math.isfinite(v) for v in values), 'Non-finite cmd_vel')
            require(abs(values[0]) <= .500001 and abs(values[5]) <= .500001,
                    f'Tracking command exceeds configured limit: {values}')
            require(all(abs(values[i]) <= 1.e-10 for i in (1, 2, 3, 4)),
                    f'Unexpected non-planar command: {values}')
            self.last_command = (values[0], values[5])
            self.commands.append((time.monotonic()-started, self.phase, *self.last_command))

        def receive_status(self, message):
            value = json.loads(message.data)
            require(value['physics'] == 'Bullet', value)
            require(value['chassis_contacts'] == 0 and value['chassis_contact_steps'] == 0,
                    f'Unexpected chassis/ground contact: {value}')
            require(len(value['wheel_speed']) == len(value['wheel_target']) == 4, value)
            motor_values = [value['command_linear'], value['command_angular'],
                            *value['wheel_speed'], *value['wheel_target']]
            require(all(math.isfinite(v) for v in motor_values), 'Non-finite wheel state')
            require(abs(value['command_linear']) <= .500001
                    and abs(value['command_angular']) <= .500001,
                    f'Motor command exceeds tracking limit: {value}')
            # SMB converts each bounded chassis command to a left/right wheel
            # target using its retained 0.6604 m separation and 0.19 m radius.
            wheel_bound = (.5 + .5 * .6604 / 2.) / .19
            require(max(map(abs, value['wheel_target'])) <= wheel_bound + 1.e-6,
                    f'Wheel target exceeds differential-drive limit: {value}')
            self.status = value
            self.states.append({'time_s': time.monotonic()-started, 'phase': self.phase,
                                **value})

        def receive_tracking(self, message):
            self.tracking_states.append({'time_s': time.monotonic()-started,
                                         **json.loads(message.data)})

        def until(self, predicate, label, seconds=5.):
            # Leave four seconds of the 45 s wall budget for process cleanup.
            deadline = min(time.monotonic() + seconds, started + 40.)
            while not predicate():
                for process in processes:
                    if process.poll() is not None:
                        raise RuntimeError(f'ROS node exited {process.returncode}; see {log_path}')
                if time.monotonic() > deadline:
                    raise TimeoutError(f'{self.phase}: {label}; pose={self.position}; state={self.status}')
                rclpy.spin_once(self, timeout_sec=.01)

        def spin_for(self, seconds):
            end = time.monotonic() + seconds
            self.until(lambda: time.monotonic() >= end, 'Observation interval', seconds + .25)

        def state(self, client):
            self.until(client.service_is_ready, 'State service discovery')
            future = client.call_async(Trigger.Request())
            self.until(future.done, 'State service response')
            reply = future.result()
            require(reply.success, reply.message)
            return json.loads(reply.message)

        def stationary(self):
            return (self.position is not None and time.monotonic()-self.odom_received < .2
                    and self.speed < .03 and abs(self.yaw_rate) < .05
                    and bool(self.status) and max(map(abs, self.status['wheel_target'])) < 1.e-6)

        def wait_stationary(self):
            self.until(self.stationary, 'Physical vehicle stop', 3.)
            position = self.position
            self.spin_for(.25)
            require(self.stationary(), 'Vehicle resumed motion after stopping')
            require(math.dist(position[:3], self.position[:3]) < .015
                    and abs(wrap(position[3]-self.position[3])) < .025,
                    'Excessive settled-pose drift')

    def launch(log, package, executable, parameters):
        command = ['ros2', 'run', package, executable, '--ros-args']
        for key, value in parameters.items():
            command += ['-p', f'{key}:={value}']
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=dict(os.environ))
        processes.append(process)

    def stop(process):
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=1.5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=.5)

    try:
        rclpy.init(args=[])
        node = Probe()
        with log_path.open('w') as log:
            # This is the actual first local segment from smb_autonomy/run8.
            # Only the initial height is raised for Bullet to settle onto its
            # wheels; XY/yaw and the requested path are the recorded values.
            terminal_route = [
                (-.8389034867286682, .0016545632388442755, .2189998217821122,
                 -.0506260673246966),
                (-1.0097428275770157, .1046982295880039, .18682535042036869,
                 2.5988519635603526),
            ]
            initial_position = (f'[{terminal_route[0][0]}, {terminal_route[0][1]}, 0.26]'
                                if args.terminal_turn else '[0.0, 0.0, 0.26]')
            launch(log, 'gbplanner_smb', 'smb_node', {
                'initial_position': initial_position, 'room': 'false',
                'initial_yaw': terminal_route[0][3] if args.terminal_turn else 0.,
                # Terminal-only tracking has no map/laser consumer. Keep its
                # physical and odometry clocks unchanged, skip recurring scans.
                'lidar_period': 3600. if args.terminal_turn else .2, 'command_timeout': .3,
            })
            launch(log, 'gbplanner_tracking', 'pure_pursuit_node', {
                'look_ahead_distance': .5 if args.terminal_turn else .25, 'desired_vel': .3,
                'max_lin_vel': .5, 'max_ang_vel': .5,
                'goal_reaching_threshold': .04, 'goal_yaw_threshold': .1,
            })
            node.until(lambda: node.action.server_is_ready() and node.smb_state.service_is_ready()
                       and node.tracking_state.service_is_ready() and node.position is not None
                       and node.status.get('wheel_contacts', 0) >= 2,
                       'SMB/tracker discovery and wheel-ground contact', 10.)
            node.wait_stationary()
            initial = node.state(node.smb_state)
            require(initial['physics'] == 'Bullet', initial)
            summary['initial_vehicle'] = initial

            node.phase = 'tracking'
            route = (terminal_route if args.terminal_turn else
                     [(0., 0., .24, 0.), (.5, 0., .24, 0.), (.9, .15, .24, .4)])
            goal = PathFollower.Goal()
            goal.manual_path, goal.vel_max = True, .3
            for x, y, z, yaw in route:
                point = PoseStamped()
                point.header.frame_id = 'map'
                point.header.stamp = node.get_clock().now().to_msg()
                point.pose.position.x, point.pose.position.y, point.pose.position.z = x, y, z
                point.pose.orientation.z, point.pose.orientation.w = math.sin(yaw/2.), math.cos(yaw/2.)
                goal.path.append(point)
            before = len(node.commands)
            future = node.action.send_goal_async(goal, feedback_callback=lambda msg: node.feedback.append(msg.feedback))
            node.until(future.done, 'Native tracking action acceptance')
            handle = future.result()
            require(handle.accepted, node.state(node.tracking_state))
            result = handle.get_result_async()
            node.until(result.done, 'SMB follows the requested route and terminal heading', 30.)
            reply = result.result()
            require(reply.status == GoalStatus.STATUS_SUCCEEDED and reply.result.success, reply)
            terminal_state = node.state(node.tracking_state)
            require(terminal_state['state'] == 'SUCCEEDED'
                    and terminal_state['dist_to_goal'] <= .04
                    and abs(terminal_state['yaw_error']) <= .1,
                    f'Native action did not meet its strict terminal tolerances: {terminal_state}')
            node.until(lambda: node.last_command == (0., 0.)
                       and any(abs(v) > .02 for _, _, v, _ in node.commands[before:]),
                       'Tracker terminal zero velocity command', 1.)
            node.wait_stationary()
            final = node.state(node.smb_state)
            position_error = math.dist(node.position[:2], route[-1][:2])
            yaw_error = abs(wrap(node.position[3]-route[-1][3]))
            require(position_error <= .07 and yaw_error <= .15,
                    f'Settled endpoint error: {position_error} m, {yaw_error} rad')
            require(node.feedback, 'No action feedback received')
            require(any(record['phase'] == 'tracking' and record['wheel_contacts'] >= 2
                        and max(map(abs, record['wheel_speed'])) > .1 for record in node.states),
                    'No observed driven wheel-ground contacts')
            minimum_travel = .6 * math.dist(route[0][:2], route[-1][:2]) if args.terminal_turn else .7
            require(final['distance_m']-initial['distance_m'] > minimum_travel,
                    'Insufficient measured chassis travel')
            if args.terminal_turn:
                require(any(record['state'] == 'ALIGNING' for record in node.tracking_states),
                        'Recorded terminal-turn case never entered heading alignment')
            summary['checks']['tracking'] = {
                'route': route, 'status': reply.status, 'observed_pose': node.position,
                'position_error_m': position_error, 'yaw_error_rad': yaw_error,
                'feedback_messages': len(node.feedback), 'final_vehicle': final,
                'tracker': terminal_state,
                'measured_travel_m': final['distance_m']-initial['distance_m'],
                'tracking_transitions': node.tracking_states,
                'terminal_zero_command_and_physical_stop': True,
            }

            if args.terminal_turn:
                summary['status'] = 'passed'
                return 0

            # The completed tracker is idle and sends no periodic commands.
            # Apply a short motor pulse, then publish nothing: only the SMB
            # receiver's own timeout may remove this nonzero wheel target.
            node.phase = 'command_timeout'
            node.until(lambda: node.pulse.get_subscription_count() >= 2, 'Pulse publisher discovery')
            pulse = Twist()
            pulse.linear.x = .2
            last_pulse = 0.
            pulse_end = time.monotonic() + .2
            while time.monotonic() < pulse_end:
                node.pulse.publish(pulse)
                last_pulse = time.monotonic()
                node.spin_for(.04)
            node.until(lambda: not node.status.get('timed_out', True)
                       and max(map(abs, node.status.get('wheel_target', [0.]))) > .1,
                       'Nonzero motor targets before stopping publications', .2)
            node.spin_for(.08)
            commands_after_pulse = node.status['commands']
            node.until(lambda: node.status.get('timed_out', False)
                       and max(map(abs, node.status.get('wheel_target', [1.]))) < 1.e-6,
                       'SMB command timeout resets all four wheel targets', 1.5)
            elapsed = time.monotonic()-last_pulse
            node.wait_stationary()
            timed_out = node.state(node.smb_state)
            require(timed_out['commands'] == commands_after_pulse,
                    'A later command, rather than receiver timeout, may have stopped the motors')
            require(node.last_command == (.2, 0.),
                    'Zero Twist was published during the independent receiver-timeout check')
            require(timed_out['timed_out'] and max(map(abs, timed_out['wheel_target'])) < 1.e-6,
                    timed_out)
            summary['checks']['command_timeout'] = {
                'configured_timeout_s': .3, 'zero_targets_observed_after_s': elapsed,
                'last_published_command': [.2, 0.], 'no_subsequent_stop_command': True,
                'final_vehicle': timed_out,
            }
            summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
        if node is not None:
            for name, client in [('tracking', node.tracking_state), ('vehicle', node.smb_state)]:
                try:
                    summary['failure_' + name] = node.state(client)
                except Exception as state_error:
                    summary['failure_' + name + '_error'] = str(state_error)
    finally:
        for process in reversed(processes):
            stop(process)
        if node is not None:
            with trace_path.open('w', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(['time_s', 'phase', 'odom_stamp_s', 'x_m', 'y_m', 'z_m', 'yaw_rad',
                                 'vx_mps', 'vy_mps', 'vz_mps', 'wx_radps', 'wy_radps', 'wz_radps'])
                writer.writerows(node.trace)
            with commands_path.open('w', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(['time_s', 'phase', 'linear_x_mps', 'angular_z_radps'])
                writer.writerows(node.commands)
            states_path.write_text(json.dumps(node.states, ensure_ascii=False, indent=2)+'\n')
            summary['odometry_samples'] = len(node.trace)
            summary['command_samples'] = len(node.commands)
            summary['max_abs_linear_command_mps'] = max((abs(row[2]) for row in node.commands), default=0.)
            summary['max_abs_angular_command_radps'] = max((abs(row[3]) for row in node.commands), default=0.)
            summary['max_abs_wheel_speed_radps'] = max(
                (abs(v) for record in node.states for v in record['wheel_speed']), default=0.)
            summary['max_wheel_contacts'] = max((v['wheel_contacts'] for v in node.states), default=0)
            summary['chassis_contact_steps'] = max((v['chassis_contact_steps'] for v in node.states), default=0)
            summary['tracking_transitions'] = node.tracking_states
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        summary['elapsed_s'] = round(time.monotonic()-started, 3)
        summary['processes_stopped'] = all(process.poll() is not None for process in processes)
        summary['artifacts'] = {'odometry': str(trace_path), 'commands': str(commands_path),
                                'vehicle_states': str(states_path), 'log': str(log_path)}
        args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(summary, ensure_ascii=False))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
