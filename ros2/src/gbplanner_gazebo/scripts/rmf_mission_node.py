#!/usr/bin/env python3
"""Optional native RMF mission entry; distinct from the original PCI init path.

The flight controller owns takeoff/landing and contact confirmation. This node
only sequences their services with the existing planner and PCI interfaces.
"""
import copy
import json
import math
import signal
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from planner_msgs.srv import PciSetHomingPos, PciStop, PciTrigger, PlannerSrv, ValidatePath
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger


class RmfMission(Node):
    TERMINAL = {'IDLE', 'COMPLETE', 'FAILED'}

    def __init__(self):
        super().__init__('rmf_mission')
        self.frame = str(self.declare_parameter('frame_id', 'map').value)
        self.auto_start = bool(self.declare_parameter('auto_start', False).value)
        self.limits = {name: float(self.declare_parameter(name+'_timeout_s', value).value)
                      for name, value in [('communication', 30.), ('status', 10.),
                                          ('ground', 30.), ('takeoff', 40.), ('map', 90.),
                                          ('mission', 450.), ('landing', 45.)]}
        self.odom_timeout = float(self.declare_parameter('odometry_timeout_s', 2.).value)
        self.recheck_interval = float(self.declare_parameter('root_recheck_interval_s', 2.).value)
        self.max_checks = int(self.declare_parameter('max_root_checks', 12).value)
        if (not self.frame or self.max_checks < 1 or
                any(not math.isfinite(v) or v <= 0 for v in
                    [*self.limits.values(), self.odom_timeout, self.recheck_interval])):
            raise ValueError('Invalid RMF mission parameters')
        self.state, self.reason = 'IDLE', 'await_start'
        self.started = False
        self.started_wall = self.phase_wall = time.monotonic()
        self.started_ros = self.phase_ros = self.clock_high_water = None
        self.clock_advanced_wall = self.started_wall
        self.phase_limit = None
        self.pending = None
        self.failure = None
        self.stop_result = None
        self.hold_result = None
        self.execution_release = None
        self.takeoff_accepted = False
        self.events, self.root_checks = [], []
        self.requests = dict(takeoff=0, validate=0, set_home=0, exploration=0, landing=0,
                             stop=0, execution_state=0, hold=0)
        self.values, self.receipts = {}, {}
        self.odom = None
        self.odom_receipt = 0.
        self.control_count = self.exploration_control_count = 0
        self.map_begin_ros = self.airborne_ros = None
        self.airborne_observation = self.last_checked_observation = -1
        self.last_check_ros = -math.inf
        self.home_position = None
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, 'rmf_mission_state', durable)
        self.create_subscription(String, 'rmf_controller_state',
                                 lambda msg: self.status('controller', msg), 10)
        for topic, key in [('map_updates', 'map'), ('control_status', 'control'),
                           ('execution_status', 'execution'), ('mission_status', 'budget')]:
            self.create_subscription(String, topic, lambda msg, name=key: self.status(name, msg), durable)
        self.create_subscription(Odometry, 'odometry', self.odometry, 10)
        self.service_clients = {'takeoff': self.create_client(Trigger, 'rmf/takeoff'),
                        'validate': self.create_client(ValidatePath, 'validate_path'),
                        'set_home': self.create_client(PciSetHomingPos, 'pci_set_homing_pos'),
                        'exploration': self.create_client(PciTrigger, 'pci_trigger'),
                        'landing': self.create_client(Empty, 'land_srv'),
                        'stop': self.create_client(PciStop, 'pci_stop'),
                        'execution_state': self.create_client(Trigger, 'execution_state'),
                        'hold': self.create_client(Trigger, 'rmf/hold')}
        self.create_service(Trigger, 'rmf/start_mission', self.start)
        self.create_service(Trigger, 'rmf/mission_state', self.get_state)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.1, self.tick, clock=self.steady_clock)
        self.transition('IDLE', 'await_start')

    def ros_now(self):
        return self.get_clock().now().nanoseconds*1.e-9

    def status(self, key, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError('object required')
        except (ValueError, TypeError) as error:
            if self.started and self.state not in self.TERMINAL:
                self.fail('invalid_'+key+'_status:'+str(error))
            return
        self.values[key], self.receipts[key] = value, time.monotonic()
        if key == 'control':
            self.control_count += 1

    def odometry(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1.e-9
        if (message.header.frame_id != self.frame or stamp <= 0 or
                not all(math.isfinite(x) for x in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or
                math.hypot(q.x, q.y, q.z, q.w) < 1.e-8):
            return
        if self.odom is not None:
            prior = self.odom.header.stamp.sec+self.odom.header.stamp.nanosec*1.e-9
            if stamp <= prior:
                return
        self.odom, self.odom_receipt = message, time.monotonic()

    def fresh_odom(self):
        if self.odom is None:
            return False
        stamp = self.odom.header.stamp.sec+self.odom.header.stamp.nanosec*1.e-9
        age = self.ros_now()-stamp
        return (-.1 <= age <= self.odom_timeout and
                time.monotonic()-self.odom_receipt <= self.limits['communication'])

    def position(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        return [p.x, p.y, p.z]

    def transition(self, state, reason, timeout=None):
        self.state, self.reason = state, reason
        self.phase_wall = time.monotonic()
        self.phase_ros = self.ros_now() or None
        self.phase_limit = timeout
        self.events.append({'state': state, 'reason': reason, 'ros_s': self.ros_now(),
                            'steady_s': self.phase_wall, 'position': self.position()})
        self.get_logger().info(state+': '+reason)
        self.publish()

    def payload(self):
        return {'state': self.state, 'reason': self.reason, 'started': self.started,
                'stamp_s': self.ros_now(), 'auto_start': self.auto_start,
                'elapsed_ros_s': self.ros_now()-self.started_ros if self.started_ros is not None else None,
                'requests': self.requests, 'failure': self.failure, 'stop_result': self.stop_result,
                'hold_result': self.hold_result, 'takeoff_accepted': self.takeoff_accepted,
                'execution_release': self.execution_release,
                'home_position': self.home_position, 'position': self.position(),
                'root_checks': self.root_checks, 'events': self.events,
                'flight': self.values.get('controller', {}).get('flight'),
                'control': {key: self.values.get('control', {}).get(key)
                            for key in ('state', 'reason', 'cycles', 'completed_actions')},
                'map': self.values.get('map'),
                'semantics': 'Native takeoff and landing services; original PCI initialization is unchanged'}

    def publish(self):
        self.publisher.publish(String(data=json.dumps(self.payload(), allow_nan=False)))

    def get_state(self, request, response):
        response.success = True
        response.message = json.dumps(self.payload(), allow_nan=False)
        return response

    def start(self, request, response):
        if self.started:
            response.success, response.message = False, 'mission_already_started:'+self.state
            return response
        self.started = True
        self.started_wall = time.monotonic()
        self.started_ros = self.ros_now() or None
        self.transition('WAIT_GROUND', 'await_native_ground_contact', self.limits['ground'])
        response.success, response.message = True, 'mission_accepted'
        return response

    def send(self, kind, request, state):
        if not self.service_clients[kind].service_is_ready():
            if time.monotonic()-self.phase_wall > self.limits['communication']:
                self.fail(kind+'_service_unavailable')
            return False
        self.requests[kind] += 1
        try:
            future = self.service_clients[kind].call_async(request)
        except Exception as error:
            self.fail(kind+'_transport_error:'+str(error))
            return False
        self.pending = {'kind': kind, 'future': future,
                        'deadline': time.monotonic()+self.limits['communication']}
        self.transition(state, kind+'_requested')
        return True

    def finish_request(self, entry):
        kind = entry['kind']
        try:
            response = entry['future'].result()
        except Exception as error:
            self.fail(kind+'_transport_error:'+str(error))
            return
        if kind == 'validate':
            checked = {'valid': bool(response.valid), 'reason': response.reason,
                       'map_revision': int(response.map_revision), 'ros_s': self.ros_now(),
                       'observation_revision': self.last_checked_observation}
            self.root_checks.append(checked)
            if response.valid:
                self.transition('SET_HOME', 'extended_root_validated')
            elif response.reason in {'map_missing', 'current_pose_not_free', 'path_not_free'}:
                # These reasons do not distinguish unknown cells from occupied
                # cells. Wait for a new observation; never clear or override it.
                if len(self.root_checks) >= self.max_checks:
                    self.fail('root_validation_attempts_exhausted:'+response.reason)
                else:
                    self.transition('WAIT_MAP', 'root_not_free_wait_new_observation:'+response.reason)
            else:
                self.fail('root_validation_rejected:'+response.reason)
        elif kind == 'landing':
            self.transition('WAIT_LANDED', 'landing_response_received', self.limits['landing'])
        elif not response.success:
            self.fail(kind+'_rejected:'+str(getattr(response, 'message', 'false')))
        elif kind == 'takeoff':
            self.takeoff_accepted = True
            self.transition('WAIT_AIRBORNE', 'takeoff_accepted', self.limits['takeoff'])
        elif kind == 'set_home':
            self.home_position = self.position()
            self.transition('START_EXPLORATION', 'home_set')
        elif kind == 'exploration':
            self.transition('EXPLORING', 'automatic_exploration_accepted')

    def fail(self, reason):
        if self.state in {'STOPPING', 'FAILURE_HOLD', 'FAILED', 'COMPLETE'}:
            return
        self.failure = reason
        # Cancel only our local response future. A request already delivered
        # may still execute; the failure remains visible and is never retried.
        if self.pending is not None:
            self.pending['future'].cancel()
            self.pending = None
        self.transition('STOPPING', reason)

    def stopped_after_failure(self):
        self.pending = None
        flight = self.values.get('controller', {}).get('flight', {}).get('state')
        if self.requests['landing'] or flight in {'LANDING', 'LANDED', 'GROUNDED'}:
            self.hold_result = 'not_requested_landing_or_ground_state'
        elif (self.requests['takeoff'] and flight in {'TAKING_OFF', 'AIRBORNE'} and
              self.fresh_odom() and
              time.monotonic()-self.receipts.get('controller', 0.) <= self.limits['status']):
            # PCI may not own the controller's takeoff trajectory. Its stop
            # response alone cannot halt that climb; request one native hold.
            self.transition('FAILURE_HOLD', 'pci_stop_finished_wait_output_release')
            return
        else:
            self.hold_result = 'not_requested_flight_state_unconfirmed'
        self.transition('FAILED', self.failure)

    def hold_after_failure(self):
        flight = self.values.get('controller', {}).get('flight', {}).get('state')
        if self.pending is None:
            if flight not in {'TAKING_OFF', 'AIRBORNE'} or self.requests['landing']:
                self.hold_result = 'not_requested_flight_state:'+str(flight)
                self.transition('FAILED', self.failure)
                return
            fresh = (self.fresh_odom() and
                     time.monotonic()-self.receipts.get('controller', 0.) <= self.limits['status'])
            if not fresh:
                self.hold_result = 'not_requested_stale_flight_observation'
                self.transition('FAILED', self.failure)
                return
            health = self.values.get('execution', {})
            released = (self.receipts.get('execution', 0.) >= self.phase_wall and
                        time.monotonic()-self.receipts.get('execution', 0.) <= self.limits['status'] and
                        all(health.get(key) is False for key in
                            ('active', 'output_in_flight', 'output_unconfirmed')) and
                        not str(health.get('reason', '')).endswith(
                            (':hold_unconfirmed', ':external_stop_unconfirmed')))
            if not released:
                if time.monotonic()-self.phase_wall > self.limits['communication']:
                    self.execution_release = 'unconfirmed_timeout'
                    self.hold_result = 'not_requested_output_release_unconfirmed'
                    self.transition('FAILED', self.failure)
                elif not self.requests['execution_state'] and self.service_clients['execution_state'].service_is_ready():
                    # execution_status is event-driven. One explicit snapshot
                    # confirms an idle takeoff executor without treating its
                    # old startup event as a current release acknowledgement.
                    self.requests['execution_state'] = 1
                    try:
                        self.pending = {'kind': 'execution_state',
                            'future': self.service_clients['execution_state'].call_async(Trigger.Request()),
                            'deadline': self.phase_wall+self.limits['communication']}
                    except Exception as error:
                        self.execution_release = 'transport_error:'+str(error)
                        self.hold_result = 'not_requested_output_release_unconfirmed'
                        self.transition('FAILED', self.failure)
                return
            self.execution_release = 'confirmed'
            if not self.service_clients['hold'].service_is_ready():
                if time.monotonic()-self.phase_wall > self.limits['communication']:
                    self.hold_result = 'service_unavailable'
                    self.transition('FAILED', self.failure)
                return
            self.requests['hold'] = 1
            try:
                self.pending = {'kind': 'hold', 'future': self.service_clients['hold'].call_async(Trigger.Request()),
                                'deadline': time.monotonic()+self.limits['communication']}
            except Exception as error:
                self.hold_result = 'transport_error:'+str(error)
                self.transition('FAILED', self.failure)
        elif self.pending['future'].done():
            if self.pending['kind'] == 'execution_state':
                try:
                    response = self.pending['future'].result()
                    health = json.loads(response.message)
                    if not response.success or not isinstance(health, dict):
                        raise ValueError('execution_state_rejected')
                    # An event can overtake this service response. Preserve the
                    # newer producer snapshot using its steady-clock timestamp.
                    produced = health.get('odometry_freshness', {}).get('tick_monotonic_s', 0.)
                    existing = self.values.get('execution', {}).get('odometry_freshness', {}).get('tick_monotonic_s', 0.)
                    if produced >= existing:
                        self.values['execution'], self.receipts['execution'] = health, time.monotonic()
                    self.execution_release = 'snapshot_received'
                except Exception as error:
                    self.execution_release = 'response_error:'+str(error)
                    self.hold_result = 'not_requested_output_release_unconfirmed'
                    self.transition('FAILED', self.failure)
                self.pending = None
                return
            try:
                response = self.pending['future'].result()
                self.hold_result = ('acknowledged:' if response.success else 'rejected:')+response.message
            except Exception as error:
                self.hold_result = 'transport_error:'+str(error)
            self.pending = None
            self.transition('FAILED', self.failure)
        elif time.monotonic() > self.pending['deadline']:
            kind = self.pending['kind']
            self.pending['future'].cancel()
            self.pending = None
            self.hold_result = kind+'_response_timeout'
            if kind == 'execution_state':
                self.execution_release = 'unconfirmed_timeout'
            self.transition('FAILED', self.failure)

    def stop_failed_mission(self):
        if self.pending is None:
            if self.requests['stop']:
                return
            if not self.service_clients['stop'].service_is_ready():
                if time.monotonic()-self.phase_wall > self.limits['communication']:
                    self.stop_result = 'service_unavailable'
                    self.stopped_after_failure()
                return
            self.requests['stop'] = 1
            try:
                self.pending = {'kind': 'stop', 'future': self.service_clients['stop'].call_async(PciStop.Request()),
                                'deadline': time.monotonic()+self.limits['communication']}
            except Exception as error:
                self.stop_result = 'transport_error:'+str(error)
                self.stopped_after_failure()
        elif self.pending['future'].done():
            try:
                self.stop_result = 'acknowledged' if self.pending['future'].result().success else 'rejected'
            except Exception as error:
                self.stop_result = 'transport_error:'+str(error)
            self.stopped_after_failure()
        elif time.monotonic() > self.pending['deadline']:
            self.pending['future'].cancel()
            self.stop_result = 'response_timeout'
            self.stopped_after_failure()

    def tick(self):
        if self.auto_start and not self.started:
            self.start(Trigger.Request(), Trigger.Response())
        if self.state in {'STOPPING', 'FAILURE_HOLD'}:
            if self.state == 'STOPPING':
                self.stop_failed_mission()
            else:
                self.hold_after_failure()
            self.publish()
            return
        if self.state in self.TERMINAL:
            self.publish()
            return
        now, wall = self.ros_now(), time.monotonic()
        if self.clock_high_water is not None and now < self.clock_high_water-1.e-6:
            self.fail('ros_clock_regressed')
            return
        if self.clock_high_water is None or now > self.clock_high_water:
            self.clock_high_water, self.clock_advanced_wall = now, wall
        if now <= 0 or wall-self.clock_advanced_wall > self.limits['communication']:
            if wall-self.started_wall > self.limits['communication']:
                self.fail('ros_clock_unavailable_or_stalled')
            return
        if self.started_ros is None:
            self.started_ros = now
        if self.phase_ros is None:
            self.phase_ros = now
        if now-self.started_ros > self.limits['mission']:
            self.fail('mission_ros_timeout')
            return
        if self.phase_limit is not None and now-self.phase_ros > self.phase_limit:
            self.fail(self.state.lower()+'_ros_timeout')
            return
        if self.state != 'WAIT_GROUND':
            if wall-self.receipts.get('controller', self.started_wall) > self.limits['status']:
                self.fail('controller_status_timeout')
                return
            if not self.fresh_odom():
                self.fail('odometry_stale')
                return
        flight = self.values.get('controller', {}).get('flight', {})
        if flight.get('state') == 'FAULT':
            self.fail('flight_fault:'+str(flight.get('reason')))
            return
        if self.state in {'WAIT_MAP', 'VALIDATING_ROOT', 'SET_HOME', 'SETTING_HOME',
                          'START_EXPLORATION', 'EXPLORATION_REQUESTED', 'EXPLORING', 'REQUEST_LANDING'}:
            if flight.get('state') != 'AIRBORNE':
                self.fail('unexpected_flight_state:'+str(flight.get('state')))
                return
        if self.pending is not None:
            entry = self.pending
            if entry['future'].done():
                self.pending = None
                self.finish_request(entry)
            elif wall > entry['deadline']:
                self.fail(entry['kind']+'_response_timeout')
            self.publish()
            return
        if self.state == 'WAIT_GROUND':
            if (flight.get('state') == 'GROUNDED' and self.fresh_odom() and
                    wall-self.receipts.get('controller', 0.) <= self.limits['status']):
                self.send('takeoff', Trigger.Request(), 'TAKEOFF_REQUESTED')
        elif self.state == 'WAIT_AIRBORNE':
            if flight.get('state') == 'AIRBORNE':
                self.airborne_ros = self.map_begin_ros = now
                self.airborne_observation = int(self.values.get('map', {}).get('observation_revision', -1))
                self.transition('WAIT_MAP', 'await_observed_extended_root')
        elif self.state in {'WAIT_MAP', 'VALIDATING_ROOT'}:
            if now-self.map_begin_ros > self.limits['map']:
                self.fail('map_root_ros_timeout')
                return
            update = self.values.get('map', {})
            observation = int(update.get('observation_revision', -1))
            stamp = float(update.get('stamp_sec', 0))+float(update.get('stamp_nanosec', 0))*1.e-9
            if (int(update.get('known_cells', 0)) > 0 and stamp >= self.airborne_ros and
                    observation > max(self.airborne_observation, self.last_checked_observation) and
                    now-self.last_check_ros >= self.recheck_interval):
                request = ValidatePath.Request()
                request.header.frame_id = self.frame
                request.header.stamp = self.get_clock().now().to_msg()
                request.path = [copy.deepcopy(self.odom.pose.pose)]
                request.bound_mode = PlannerSrv.Request.K_EXTENDED_BOUND
                request.first_pose_is_observation = True
                if self.send('validate', request, 'VALIDATING_ROOT'):
                    self.last_checked_observation, self.last_check_ros = observation, now
        elif self.state == 'SET_HOME':
            self.send('set_home', PciSetHomingPos.Request(), 'SETTING_HOME')
        elif self.state == 'START_EXPLORATION':
            request = PciTrigger.Request()
            request.set_auto, request.not_exe_path = True, False
            request.bound_mode, request.vel_max = PlannerSrv.Request.K_EXTENDED_BOUND, 0.
            self.exploration_control_count = self.control_count
            self.send('exploration', request, 'EXPLORATION_REQUESTED')
        elif self.state == 'EXPLORING':
            # control_status is an event stream, not a heartbeat. The planner's
            # mission_status and the flight controller supply communication health.
            if wall-self.receipts.get('budget', self.phase_wall) > self.limits['communication']:
                self.fail('planner_mission_status_timeout')
                return
            control = self.values.get('control', {})
            if self.control_count > self.exploration_control_count:
                if control.get('state') in {'ERROR', 'STOPPED'}:
                    self.fail('pci_'+control['state'].lower()+':'+str(control.get('reason')))
                elif control.get('state') == 'DONE' and control.get('reason') in {'returned_home', 'already_home'}:
                    self.transition('REQUEST_LANDING', control['reason'])
        elif self.state == 'REQUEST_LANDING':
            self.send('landing', Empty.Request(), 'LANDING_REQUESTED')
        elif self.state == 'WAIT_LANDED':
            speeds = self.values.get('controller', {}).get('motor_speed_rad_s', [])
            if (flight.get('state') == 'LANDED' and flight.get('touchdown_confirmed') is True and
                    len(speeds) == 4 and all(math.isfinite(x) and abs(x) <= 1.e-9 for x in speeds)):
                self.transition('COMPLETE', 'landed_contact_confirmed_motors_zero')
        self.publish()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = False
    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = RmfMission()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not stopping:
            executor.spin_once(timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if rclpy.ok() and node.started and node.state not in node.TERMINAL:
            node.fail('shutdown_requested')
            # A shutdown request must never issue an airborne disarm command.
            end = time.monotonic()+min(2., node.limits['communication'])
            while node.state in {'STOPPING', 'FAILURE_HOLD'} and time.monotonic() < end:
                executor.spin_once(timeout_sec=.05)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
