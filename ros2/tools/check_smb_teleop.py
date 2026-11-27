#!/usr/bin/env python3
"""Short native ROS mux/joystick contract check with explicit PCI state fixtures."""
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))

    import rclpy
    from geometry_msgs.msg import Twist
    from planner_msgs.srv import PciStop
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Bool, Empty, String
    from std_srvs.srv import Trigger

    started = time.monotonic()
    deadline = started+12.
    processes, node = [], None
    log_path = args.output.with_suffix('.log')
    trace_path = args.output.with_suffix('.csv')
    history_path = args.output.with_name(args.output.stem+'_history.json')
    summary = {
        'status': 'failed', 'domain': os.environ['ROS_DOMAIN_ID'],
        'scope': 'real velocity_mux_node and joy_teleop_node with a minimal PCI stop '
                 'service and execution/tracking/control status fixture; no robot physics or planner run',
        'timeouts_s': {'source': .18, 'joystick': .12}, 'checks': {},
    }

    def twist(velocity):
        value = Twist()
        value.linear.x = float(velocity)
        return value

    class Fixture(Node):
        def __init__(self):
            super().__init__('smb_teleop_contract_fixture')
            self.phase = 'discovery'
            self.output = None
            self.mux = {}
            self.outputs, self.mux_history, self.joy_outputs = [], [], []
            self.lock_messages, self.stop_messages = [], []
            self.stop_calls = []
            self.external_value, self.marker_value = None, None
            self.joy_enabled, self.joy_velocity = False, 0.
            self.joy_buttons = [0]*6
            self.execution = {'active': True, 'state': 'EXECUTING', 'output_in_flight': True,
                              'output_unconfirmed': False, 'continuation_pending': False,
                              'goal_id': [1]*16, 'accepted': 1}
            self.tracking = {'active': True, 'state': 'TRACKING', 'goal_id': '01'*16, 'accepted': 1,
                             'last_command': {'linear_x': .11, 'angular_z': 0.}}
            self.control = {'state': 'EXECUTING', 'auto': False}
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.status_publishers = {name: self.create_publisher(String, name+'_status', durable)
                                      for name in ('execution', 'tracking', 'control')}
            for name in self.status_publishers:
                self.create_service(Trigger, name+'_state',
                    lambda request, response, key=name: self.state_service(response, key))
            self.create_service(PciStop, 'pci_stop', self.stop)
            self.external = self.create_publisher(Twist, 'external/cmd_vel', 10)
            self.marker = self.create_publisher(Twist, 'twist_marker_server/cmd_vel', 10)
            self.joystick = self.create_publisher(Joy, 'joy', 10)
            self.lock = self.create_publisher(Bool, 'cmd_vel_e_stop', 10)
            self.emergency = self.create_publisher(Empty, 'soft_emergency_stop', 10)
            self.create_subscription(Twist, 'cmd_vel', self.receive_output, 10)
            self.create_subscription(Twist, 'joy_teleop/cmd_vel', self.receive_joy, 10)
            self.create_subscription(Bool, 'cmd_vel_e_stop',
                lambda m: self.lock_messages.append((time.monotonic()-started, m.data)), 10)
            self.create_subscription(Empty, 'soft_emergency_stop',
                lambda m: self.stop_messages.append(time.monotonic()-started), 10)
            self.create_subscription(String, 'velocity_mux_status', self.receive_mux, durable)
            self.mux_state = self.create_client(Trigger, 'velocity_mux_state')
            self.create_timer(.02, self.tick)

        def state_service(self, response, name):
            response.success = True
            response.message = json.dumps(getattr(self, name))
            return response

        def stop(self, request, response):
            self.stop_calls.append({'time_s': time.monotonic()-started, 'phase': self.phase})
            # Model a completed PCI cancellation. State services return these
            # live values, so the mux cannot rely on an old durable idle event.
            self.execution.update(active=False, state='DONE', output_in_flight=False,
                                  output_unconfirmed=False, continuation_pending=False, goal_id=None)
            self.tracking.update(active=False, state='CANCELED', last_command={'linear_x': 0., 'angular_z': 0.})
            self.control.update(state='STOPPED', auto=False)
            self.publish_states()
            response.success = True
            return response

        def publish_states(self):
            for name, publisher in self.status_publishers.items():
                message = String()
                message.data = json.dumps(getattr(self, name))
                publisher.publish(message)

        def tick(self):
            self.publish_states()
            if self.external_value is not None:
                self.external.publish(twist(self.external_value))
            if self.marker_value is not None:
                self.marker.publish(twist(self.marker_value))
            if self.joy_enabled:
                message = Joy()
                message.header.stamp = self.get_clock().now().to_msg()
                message.axes = [0., 0., 0., float(self.joy_velocity)]
                message.buttons = list(self.joy_buttons)
                self.joystick.publish(message)

        def receive_output(self, message):
            values = [message.linear.x, message.linear.y, message.linear.z,
                      message.angular.x, message.angular.y, message.angular.z]
            require(all(math.isfinite(v) for v in values), 'Non-finite mux output')
            self.output = (message.linear.x, message.angular.z)
            self.outputs.append((time.monotonic()-started, self.phase, *self.output))

        def receive_joy(self, message):
            self.joy_outputs.append((time.monotonic()-started, self.phase,
                                     message.linear.x, message.angular.z))

        def receive_mux(self, message):
            self.mux = json.loads(message.data)
            self.mux_history.append({'time_s': time.monotonic()-started, 'phase': self.phase, **self.mux})

        def until(self, predicate, label, seconds=1.2):
            end = min(time.monotonic()+seconds, deadline)
            while not predicate():
                if any(process.poll() is not None for process in processes):
                    raise RuntimeError(f'Teleop node exited; see {log_path}')
                if time.monotonic() > end:
                    raise TimeoutError(f'{self.phase}: {label}; mux={self.mux}; output={self.output}')
                rclpy.spin_once(self, timeout_sec=.005)

        def spin_for(self, seconds):
            end = time.monotonic()+seconds
            self.until(lambda: time.monotonic() >= end, 'Observation interval', seconds+.2)

        def has_output(self, velocity):
            return self.output is not None and abs(self.output[0]-velocity) < 1.e-6 and abs(self.output[1]) < 1.e-6

        def select(self, source, velocity):
            self.until(lambda: self.mux.get('selected_source') == source and self.has_output(velocity),
                       f'{source} selected at {velocity} m/s')

        def status(self):
            future = self.mux_state.call_async(Trigger.Request())
            self.until(future.done, 'Mux state response')
            response = future.result()
            require(response.success, response.message)
            return json.loads(response.message)

        def require_blocked(self, label):
            self.spin_for(.07)
            require(self.has_output(0.) and self.mux.get('gate') != 'open',
                    f'{label}: {self.mux}, {self.output}')

    def launch(log, executable, parameters):
        command = ['ros2', 'run', 'gbplanner_smb', executable, '--ros-args']
        for key, value in parameters.items():
            command += ['-p', f'{key}:={value}']
        processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                          start_new_session=True, env=dict(os.environ)))

    def stop(process):
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=1.)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=.3)

    try:
        rclpy.init(args=[])
        node = Fixture()
        with log_path.open('w') as log:
            launch(log, 'velocity_mux_node.py', {'source_timeout': .18, 'publish_period': .02})
            launch(log, 'joy_teleop_node.py', {'joy_timeout': .12, 'publish_period': .02,
                                              'max_velocity_linear': 1., 'max_velocity_angular': 1.})
            node.until(lambda: node.mux_state.service_is_ready()
                       and node.external.get_subscription_count() > 0
                       and node.marker.get_subscription_count() > 0
                       and node.joystick.get_subscription_count() > 0 and bool(node.mux),
                       'Native mux/joystick discovery', 5.)
            node.phase = 'priority_and_expiration'
            node.external_value = .11
            node.select('external', .11)
            node.external_value = None
            node.select('none', 0.)
            summary['checks']['external_expiration'] = 'Expired external stream produced zero output'

            node.external_value = .11
            node.select('external', .11)
            node.marker_value = .22
            node.select('interactive_marker', .22)
            node.until(lambda: bool(node.stop_calls) and node.mux.get('stop_acknowledged'),
                       'Manual takeover requested and acknowledged pci_stop')
            node.joy_velocity, node.joy_enabled = .33, True
            node.select('joy', .33)
            joy_before = len(node.joy_outputs)
            node.joy_enabled = False
            node.until(lambda: any(abs(row[2]) < 1.e-10 and abs(row[3]) < 1.e-10
                                  for row in node.joy_outputs[joy_before:]),
                       'Stale joystick input produced a zero Twist')
            node.select('interactive_marker', .22)
            summary['checks']['manual_priority'] = {
                'order': ['joy', 'interactive_marker', 'external'],
                'joystick_timeout_zero_observed': True,
                'expired_joystick_released_marker': True,
            }
            node.marker_value = None
            node.until(lambda: node.mux.get('gate') == 'waiting_for_goal' and node.has_output(0.),
                       'Expired manual stream does not resume old autonomous commands')
            node.require_blocked('Old external stream survived manual takeover')
            summary['checks']['takeover'] = {'pci_stop_calls': len(node.stop_calls),
                                            'old_external_stream_blocked': True,
                                            'state': node.status()}

            node.phase = 'new_goal_gate'
            # Replaying both old active states must not defeat the post-stop
            # accepted-counter baseline established through the state services.
            node.execution.update(active=True, state='EXECUTING', output_in_flight=True, goal_id=[1]*16)
            node.tracking.update(active=True, state='TRACKING', last_command={'linear_x': .11, 'angular_z': 0.})
            node.control['state'] = 'EXECUTING'
            node.require_blocked('Replayed old goal states reopened the gate')
            node.execution.update(goal_id=[2]*16, accepted=2)
            node.require_blocked('Execution-only new goal reopened the gate')
            node.external_value = None
            node.external.publish(twist(.44))
            node.spin_for(.03)
            node.tracking.update(goal_id='03'*16, accepted=2)
            node.until(lambda: node.mux.get('gate') == 'open', 'Both new goal identities reopen gate')
            node.spin_for(.04)
            require(node.has_output(0.), 'Cached external command replayed when the gate opened')
            node.external.publish(twist(.13))
            node.select('external', .13)
            summary['checks']['new_goal_gate'] = {
                'old_active_status_replay_blocked': True,
                'execution_only_restart_blocked': True,
                'both_new_goal_ids_and_accepted_counts_required': True,
                'old_cached_twist_cleared': True, 'new_external_twist_forwarded': True,
            }

            node.phase = 'locks_and_button_edges'
            node.lock.publish(Bool(data=True))
            node.until(lambda: node.mux.get('locked') is True and node.has_output(0.), 'Bool lock stops output')
            node.lock.publish(Bool(data=False))
            node.until(lambda: node.mux.get('locked') is False, 'Bool unlock')
            node.require_blocked('Unlock resumed a pre-stop autonomous goal')
            lock_before = len(node.lock_messages)
            node.joy_velocity, node.joy_enabled = 0., True
            node.joy_buttons[4] = 1
            node.until(lambda: node.mux.get('locked') is True, 'Joystick lock rising edge')
            node.spin_for(.16)
            emitted = [value for _, value in node.lock_messages[lock_before:]]
            require(emitted == [True] and node.mux.get('locked') is True,
                    f'Held joystick button retriggered lock: {emitted}')
            node.joy_buttons[4] = 0
            node.spin_for(.04)
            node.joy_buttons[4] = 1
            node.until(lambda: node.mux.get('locked') is False, 'Second rising edge unlocks')
            node.joy_buttons[4] = 0
            node.spin_for(.04)
            stop_before = len(node.stop_messages)
            node.joy_buttons[5] = 1
            node.until(lambda: len(node.stop_messages) > stop_before and node.mux.get('locked') is True
                       and node.has_output(0.), 'Joystick soft emergency stop')
            node.spin_for(.16)
            require(len(node.stop_messages) == stop_before+1,
                    'Held emergency-stop button republished stop events')
            node.joy_enabled = False
            node.lock.publish(Bool(data=False))
            node.until(lambda: node.mux.get('locked') is False, 'Unlock before direct soft stop')
            node.emergency.publish(Empty())
            node.until(lambda: node.mux.get('locked') is True and node.has_output(0.),
                       'Direct soft_emergency_stop locks zero output')
            summary['checks']['locks_and_button_edges'] = {
                'bool_lock_zero': True, 'unlock_does_not_resume_old_goal': True,
                'held_lock_button_events': emitted, 'second_lock_edge_unlocks': True,
                'held_stop_button_events': 1, 'direct_soft_emergency_stop_zero': True,
            }
            summary['final_mux_state'] = node.status()
            summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
        if node is not None:
            summary['last_mux_state'] = node.mux
            summary['last_output'] = node.output
    finally:
        for process in reversed(processes):
            stop(process)
        if node is not None:
            with trace_path.open('w', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(['time_s', 'phase', 'linear_x_mps', 'angular_z_radps'])
                writer.writerows(node.outputs)
            history_path.write_text(json.dumps({'mux': node.mux_history, 'joystick_output': node.joy_outputs,
                                               'lock_messages': node.lock_messages, 'stop_messages': node.stop_messages,
                                               'pci_stop_calls': node.stop_calls}, ensure_ascii=False, indent=2)+'\n')
            summary['pci_stop_calls'] = len(node.stop_calls)
            summary['output_messages'] = len(node.outputs)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        summary['elapsed_s'] = round(time.monotonic()-started, 3)
        summary['processes_stopped'] = all(process.poll() is not None for process in processes)
        summary['artifacts'] = {'output': str(trace_path), 'history': str(history_path), 'log': str(log_path)}
        args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(summary, ensure_ascii=False))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
