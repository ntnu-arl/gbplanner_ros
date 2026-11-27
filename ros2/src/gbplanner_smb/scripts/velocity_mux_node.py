#!/usr/bin/env python3
"""SMB velocity-source arbitration, adapted from the retained twist_mux config.

Unlike the ROS 1 launch, autonomous tracking enters the external input rather
than bypassing the mux. PCI owns Action cancellation; this node only selects
Twist commands and gates autonomous output after manual takeover.
"""
import json
import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger
from planner_msgs.srv import PciStop
from planner_msgs.action import PathFollower


def finite_positive(value, name):
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(name + ' must be finite and positive')
    return result


def goal_id(data):
    value = data.get('goal_id')
    if isinstance(value, list) and len(value) == 16 and all(
            isinstance(part, int) and not isinstance(part, bool) and 0 <= part <= 255
            for part in value):
        return bytes(value).hex()
    if isinstance(value, str) and len(value) == 32:
        try:
            bytes.fromhex(value)
            return value.lower()
        except ValueError:
            pass
    return None


class VelocityMux(Node):
    SOURCES = {'joy': ('joy_teleop/cmd_vel', 10),
               'interactive_marker': ('twist_marker_server/cmd_vel', 8),
               'external': ('external/cmd_vel', 1)}

    def __init__(self):
        super().__init__('gbplanner_velocity_mux')
        self.timeout = finite_positive(self.declare_parameter('source_timeout', 0.5).value, 'source_timeout')
        self.period = finite_positive(self.declare_parameter('publish_period', 0.02).value, 'publish_period')
        self.poll_period = finite_positive(self.declare_parameter('state_poll_period', 0.1).value, 'state_poll_period')
        self.reply_timeout = finite_positive(self.declare_parameter('service_timeout', 2.0).value, 'service_timeout')
        self.sources = {}
        self.locked = False
        self.gate = 'open'
        self.reason = 'ready'
        self.selected = 'none'
        self.manual_episode = False
        self.last_command = (0.0, 0.0)
        self.epoch = 0
        self.stop_acknowledged = False
        self.stop_future = None
        self.stop_started = self.next_stop = self.next_poll = 0.0
        self.query = None
        self.status = {}
        self.baseline = {'execution': 0, 'tracking': 0}
        self.retired = {'execution': set(), 'tracking': set()}
        self.stop_covered = {'execution': None, 'tracking': None}
        self.stop_requests = self.forwarded = self.blocked_external = self.rejected = 0
        self.last_status = None
        self.commands = self.create_publisher(Twist, 'cmd_vel', 1)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'velocity_mux_status', durable)
        for name, (topic, _) in self.SOURCES.items():
            self.create_subscription(Twist, topic, lambda message, source=name:
                                     self.receive_command(source, message), 1)
        self.create_subscription(Bool, 'cmd_vel_e_stop', self.receive_lock, 10)
        self.create_subscription(Empty, 'soft_emergency_stop', self.soft_stop, 10)
        for name in ('execution', 'tracking'):
            self.create_subscription(String, name + '_status',
                                     lambda message, kind=name: self.receive_status(kind, message), durable)
        self.stop_client = self.create_client(PciStop, 'pci_stop')
        self.state_clients = {name: self.create_client(Trigger, name + '_state')
                              for name in ('execution', 'tracking', 'control')}
        self.create_service(Trigger, 'velocity_mux_state', self.get_state)
        # Command expiry and stop acknowledgements use wall time, including
        # when a simulator pauses its /clock. Continue forwarding zero then.
        self.deadline_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(self.period, self.tick, clock=self.deadline_clock)
        self.publish_status()

    def fresh_manual(self):
        now = time.monotonic()
        return [name for name in ('joy', 'interactive_marker') if name in self.sources
                and now - self.sources[name][0] <= self.timeout]

    def emit(self, v=0.0, w=0.0, source='none'):
        message = Twist()
        message.linear.x, message.angular.z = float(v), float(w)
        self.commands.publish(message)
        self.last_command = (v, w)
        self.selected = source

    def retire_current(self):
        for kind in self.retired:
            identifier = goal_id(self.status.get(kind, {}))
            if identifier:
                self.retired[kind].add(identifier)

    def takeover(self, reason):
        # Publish zero before requesting cancellation, not after its reply.
        # No new Action server/state machine is created here.
        self.emit()
        self.sources.pop('external', None)
        self.retire_current()
        self.epoch += 1
        self.gate, self.reason = 'waiting_for_idle', reason
        self.stop_acknowledged = False
        self.query = None
        self.next_stop = self.next_poll = 0.0
        self.request_stop()
        self.publish_status()

    def receive_command(self, source, message):
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        if not all(math.isfinite(value) for value in values) or any(
                abs(value) > 1e-8 for value in (values[1], values[2], values[3], values[4])):
            self.rejected += 1
            self.sources.pop(source, None)
            if self.selected == source:
                self.emit()
            return
        if source == 'external' and (self.locked or self.gate != 'open'):
            self.blocked_external += 1
            return
        if source != 'external':
            if not self.manual_episode or not self.fresh_manual():
                self.manual_episode = True
                self.takeover('manual_takeover')
        self.sources[source] = (time.monotonic(), float(values[0]), float(values[5]))

    def receive_lock(self, message):
        if message.data == self.locked:
            return
        self.locked = bool(message.data)
        if self.locked:
            self.takeover('emergency_stop')
        else:
            # Releasing the lock is not permission to replay an old command.
            self.sources.clear()
            self.manual_episode = False
            self.emit()
            self.reason = 'lock_released_waiting_for_new_goal'
            self.publish_status()

    def soft_stop(self, _message):
        if not self.locked:
            self.locked = True
            self.takeover('soft_emergency_stop')
        else:
            self.emit()

    def request_stop(self):
        now = time.monotonic()
        if self.stop_acknowledged or self.stop_future is not None or now < self.next_stop:
            return
        if not self.stop_client.service_is_ready():
            self.next_stop = now + self.poll_period
            return
        epoch = self.epoch
        self.stop_covered = {kind: goal_id(self.status.get(kind, {}))
                             if self.status.get(kind, {}).get('active') else None
                             for kind in self.baseline}
        self.stop_started = now
        self.stop_requests += 1
        future = self.stop_client.call_async(PciStop.Request())
        self.stop_future = future

        def completed(result):
            if self.stop_future is not result:
                return
            self.stop_future = None
            if epoch != self.epoch:
                return
            try:
                self.stop_acknowledged = bool(result.result().success)
            except Exception:
                self.stop_acknowledged = False
            self.next_stop = time.monotonic() + self.poll_period
            self.next_poll = 0.0
            self.cancel_uncovered_goal()

        future.add_done_callback(completed)

    def cancel_uncovered_goal(self):
        if not self.stop_acknowledged or self.gate != 'waiting_for_idle':
            return
        for kind in self.baseline:
            data = self.status.get(kind, {})
            identifier = goal_id(data)
            if data.get('active') is True and identifier and identifier != self.stop_covered[kind]:
                # A new goal can arrive after the stop service has run but
                # before its response reaches us. Retired IDs prevent replay;
                # this separate snapshot identifies what that stop covered.
                self.takeover('autonomous_goal_before_idle')
                return

    @staticmethod
    def accepted(data):
        value = data.get('accepted')
        return value if isinstance(value, int) and not isinstance(value, bool) and value >= 0 else -1

    def receive_status(self, kind, message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict) or not isinstance(data.get('active'), bool):
                return
        except (ValueError, TypeError):
            return
        self.status[kind] = data
        identifier = goal_id(data)
        if self.gate == 'waiting_for_idle':
            if identifier:
                self.retired[kind].add(identifier)
            self.cancel_uncovered_goal()
        elif self.gate == 'waiting_for_goal':
            if self.locked or self.fresh_manual():
                if identifier:
                    self.retired[kind].add(identifier)
                return
            self.maybe_reopen()

    @staticmethod
    def layers_idle(snapshots):
        execution, tracking, control = (snapshots.get(name, {}) for name in
                                         ('execution', 'tracking', 'control'))
        command = tracking.get('last_command', {})
        return (execution.get('active') is False and execution.get('output_in_flight') is False
                and execution.get('continuation_pending') is False
                and execution.get('goal_id') is None and tracking.get('active') is False
                and command.get('linear_x') == 0.0 and command.get('angular_z') == 0.0
                and control.get('auto') is False
                and control.get('state') in {'IDLE', 'STOPPED', 'DONE', 'ERROR', 'PREVIEW'})

    def poll_idle(self):
        # Status topics publish transitions, not heartbeats. In particular,
        # a late downstream result can clear output_in_flight without publishing
        # another transition. Query the existing state services after pci_stop.
        now = time.monotonic()
        if self.query is None:
            if now < self.next_poll or not all(client.service_is_ready() for client in self.state_clients.values()):
                return
            self.query = (now, {name: client.call_async(Trigger.Request())
                               for name, client in self.state_clients.items()})
            return
        started, futures = self.query
        if not all(future.done() for future in futures.values()):
            if now - started > self.reply_timeout:
                for future in futures.values():
                    future.cancel()
                self.query = None
                self.next_poll = now + self.poll_period
            return
        self.query = None
        self.next_poll = now + self.poll_period
        try:
            responses = {name: future.result() for name, future in futures.items()}
            if not all(response.success for response in responses.values()):
                return
            snapshots = {name: json.loads(response.message) for name, response in responses.items()}
            if not all(isinstance(data, dict) for data in snapshots.values()):
                return
            if not self.layers_idle(snapshots):
                for kind in self.baseline:
                    if self.accepted(snapshots[kind]) >= self.accepted(self.status.get(kind, {})):
                        self.status[kind] = snapshots[kind]
                self.cancel_uncovered_goal()
                return
            for kind in self.baseline:
                count = self.accepted(snapshots[kind])
                if count < 0 or self.accepted(self.status.get(kind, {})) > count:
                    return
            if self.locked or self.fresh_manual():
                return
            for kind in self.baseline:
                self.status[kind] = snapshots[kind]
                self.baseline[kind] = self.accepted(snapshots[kind])
            self.retire_current()
            self.sources.pop('external', None)
            self.gate, self.reason = 'waiting_for_goal', 'idle_confirmed'
            self.publish_status()
        except (ValueError, TypeError, AttributeError, RuntimeError):
            return

    def maybe_reopen(self):
        execution, tracking = (self.status.get(kind, {}) for kind in ('execution', 'tracking'))
        tracking_ready = tracking.get('state') in {'TRACKING', 'ALIGNING'}
        if PathFollower.Goal.GROUND_MANEUVER in (execution.get('execution_mode'), tracking.get('execution_mode')):
            # A newly admitted stopped maneuver owns the same downstream
            # output as PATH. Braking/terminal states cannot reopen the gate.
            tracking_ready = (execution.get('execution_mode') == PathFollower.Goal.GROUND_MANEUVER
                and tracking.get('execution_mode') == PathFollower.Goal.GROUND_MANEUVER
                and tracking.get('ground_maneuver', {}).get('active') is True
                and tracking.get('state') in {'MANEUVER_RUNNING', 'MANEUVER_SETTLING'})
        if (execution.get('state') != 'EXECUTING' or execution.get('output_in_flight') is not True
                or execution.get('output_unconfirmed') is True
                or not tracking_ready):
            return
        for kind, baseline in self.baseline.items():
            data = self.status.get(kind, {})
            identifier = goal_id(data)
            if (data.get('active') is not True or identifier is None or
                    identifier in self.retired[kind] or self.accepted(data) <= baseline):
                return
        # UUIDs belong to different Action servers; never compare them to each
        # other. A new top-level goal and a new downstream goal must each exist.
        self.sources.pop('external', None)
        self.emit()
        self.gate, self.reason = 'open', 'new_autonomous_goal'
        self.publish_status()

    def tick(self):
        now = time.monotonic()
        for name, command in list(self.sources.items()):
            if now - command[0] > self.timeout:
                del self.sources[name]
        manual = self.fresh_manual()
        if not manual:
            self.manual_episode = False
        if self.gate == 'waiting_for_idle':
            if self.stop_future is not None and now - self.stop_started > self.reply_timeout:
                self.stop_future.cancel()
                self.stop_future = None
                self.next_stop = now + self.poll_period
            self.request_stop()
            if self.stop_acknowledged:
                self.poll_idle()
        if self.locked:
            self.emit()
        else:
            candidates = [name for name in self.sources if name != 'external' or self.gate == 'open']
            if candidates:
                selected = max(candidates, key=lambda name: self.SOURCES[name][1])
                _, v, w = self.sources[selected]
                self.emit(v, w, selected)
                self.forwarded += 1
            else:
                self.emit()
        self.publish_status()

    def payload(self):
        return {'locked': self.locked, 'gate': self.gate, 'reason': self.reason,
                'selected_source': self.selected, 'manual_active': bool(self.fresh_manual()),
                'stop_requests': self.stop_requests, 'stop_acknowledged': self.stop_acknowledged,
                'last_command': {'linear_x': self.last_command[0], 'angular_z': self.last_command[1]},
                'blocked_external': self.blocked_external, 'forwarded': self.forwarded,
                'rejected_commands': self.rejected,
                'execution_goal_id': goal_id(self.status.get('execution', {})),
                'tracking_goal_id': goal_id(self.status.get('tracking', {}))}

    def publish_status(self):
        # Counts are available on demand; avoid a 50 Hz status stream solely
        # because another command was forwarded.
        data = self.payload()
        state = (data['locked'], data['gate'], data['reason'], data['selected_source'],
                 data['manual_active'], data['stop_requests'], data['stop_acknowledged'],
                 self.last_command)
        if state != self.last_status:
            message = String()
            message.data = json.dumps(data)
            self.status_pub.publish(message)
            self.last_status = state

    def get_state(self, _request, response):
        response.success = True
        response.message = json.dumps(self.payload())
        return response


def main(args=None):
    # Keep the context alive until synchronous message conversion and the
    # active callback finish. Humble's asynchronous signal shutdown can race
    # take_message(), including while converting high-rate status messages.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = VelocityMux()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while not stop_requested and rclpy.ok():
            executor.spin_once(timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A launch process may forward SIGINT after the process group already
        # received it. Finish destruction without a second interruption.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        executor.remove_node(node)
        executor.shutdown()
        if rclpy.ok():
            try:
                node.emit()
            except rclpy._rclpy_pybind11.RCLError:
                # Context shutdown can race the preceding ok() check. Only
                # suppress that shutdown race; runtime publish errors remain.
                if rclpy.ok():
                    raise
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
