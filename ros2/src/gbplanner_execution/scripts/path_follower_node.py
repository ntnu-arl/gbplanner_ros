#!/usr/bin/env python3
"""Validated PathFollower bridge to a trajectory topic or external action.

Publishes commands and observes external odometry; never simulates a vehicle.
Trajectory geometry is adapted from pci_general; ROS 2 action/lifecycle logic
is new. Topic drivers replace their trajectory on every message; action drivers
implement PathFollower on pci_output_path and stop when their goal is canceled.
"""
import json
import copy
import bisect
import math
import signal
import time
import sys
from pathlib import Path
from ament_index_python.packages import get_package_prefix
sys.path.insert(0,str(Path(get_package_prefix('gbplanner_local'))/'lib/gbplanner_local'))
from path_progress import PathProgress

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from action_msgs.msg import GoalStatus
from rclpy.executors import ExternalShutdownException
from rclpy.task import Future
from rclpy.signals import SignalHandlerOptions
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Pose, PoseStamped, Transform, Twist
from nav_msgs.msg import Odometry, Path
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from std_msgs.msg import String
from std_srvs.srv import Trigger
from planner_msgs.action import PathFollower
from planner_msgs.srv import ValidatePath, ContinuePath
from trajectory_tools import (TimedSamples, continuation_within_checked, manual_validation_route,
                              prepare_continuation, prepare, remaining_validation_routes,
                              resolve_velocity, retime_polyline, state_from_pose, check_parameters, wrap)
from timed_continuation import prepare_timed_continuation
from ground_maneuver_transport import TransportLimits, decode_maneuver, validate_start


def pose(state):
    result = Pose()
    result.position.x, result.position.y, result.position.z = state[:3]
    result.orientation.z, result.orientation.w = math.sin(state[3]/2), math.cos(state[3]/2)
    return result


class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('gbplanner_path_follower')
        self.frame = self.declare_parameter('frame_id', 'map').value
        self.joint = self.declare_parameter('joint_name', 'base_link').value
        self.speed = float(self.declare_parameter('speed', 1.).value)
        self.yaw_rate = float(self.declare_parameter('yaw_rate', .15).value)
        self.dt = float(self.declare_parameter('sample_period', .05).value)
        self.smooth = bool(self.declare_parameter('smooth_yaw', True).value)
        self.robot_type = self.declare_parameter('robot_type', 'kAerial').value
        self.output_type = self.declare_parameter('output_type', 'kTopic').value
        self.trajectory_profile = self.declare_parameter('trajectory_profile', 'upstream').value
        self.continuation_strategy = self.declare_parameter('continuation_strategy', 'reanchor').value
        if self.continuation_strategy not in ('reanchor', 'append_timed'):
            raise ValueError('continuation_strategy must be reanchor or append_timed')
        if self.continuation_strategy == 'append_timed' and (self.robot_type != 'kAerial' or self.output_type != 'kTopic'):
            raise ValueError('append_timed requires an aerial topic consumer honoring header timestamps')
        self.max_acceleration = float(self.declare_parameter('max_acceleration', 1.).value)
        self.max_yaw_acceleration = float(self.declare_parameter('max_yaw_acceleration', .3).value)
        if self.trajectory_profile not in ('upstream', 'quintic', 'quintic_collinear'):
            raise ValueError('trajectory_profile must be upstream, quintic or quintic_collinear')
        if self.trajectory_profile != 'upstream' and (self.robot_type != 'kAerial' or self.output_type != 'kTopic'):
            raise ValueError('quintic profile requires an aerial trajectory topic')
        if any(not math.isfinite(v) or v <= 0 for v in (self.max_acceleration, self.max_yaw_acceleration)):
            raise ValueError('trajectory acceleration limits must be finite and positive')
        self.output_action_name = self.declare_parameter('output_action', 'pci_output_path').value
        self.action_timeout = float(self.declare_parameter('output_action_timeout', 0.).value)
        if not math.isfinite(self.action_timeout) or self.action_timeout < 0.:
            raise ValueError('output_action_timeout must be finite and non-negative')
        if self.output_type not in ('kTopic', 'kAction'):
            raise ValueError('output_type must be kTopic or kAction')
        if (not self.output_action_name or self.resolve_topic_name(self.output_action_name) ==
                self.resolve_topic_name('path_follower')):
            raise ValueError('output_action must differ from the input path_follower action')
        if self.robot_type not in ('kAerial', 'kGround'):
            raise ValueError('robot_type must be kAerial or kGround')
        # PCIGeneral allocates smoothed yaw only for aerial robots. Ground
        # paths keep their planner-assigned segment headings and terrain Z.
        if self.robot_type == 'kGround':
            self.smooth = False
        self.odom_timeout = float(self.declare_parameter('odom_timeout', 2.).value)
        self.tolerance = float(self.declare_parameter('endpoint_tolerance', .05).value)
        self.yaw_tolerance = float(self.declare_parameter('yaw_tolerance', .1).value)
        self.grace = float(self.declare_parameter('execution_grace', 10.).value)
        self.validation_timeout = float(self.declare_parameter('validation_timeout', 10.).value)
        self.max_points = int(self.declare_parameter('max_points', 20000).value)
        self.allow_ground_maneuver = bool(self.declare_parameter('allow_ground_maneuver', False).value)
        self.ground_caps = TransportLimits(**{name: float(self.declare_parameter(
            'ground_maneuver_'+name, default).value) for name, default in (
                ('max_speed', .3), ('max_yaw_rate', .4), ('max_acceleration', .3),
                ('max_yaw_acceleration', .6), ('max_duration', 30.), ('max_path_length', 5.))},
            max_points=self.max_points)
        if self.allow_ground_maneuver and (self.robot_type != 'kGround' or self.output_type != 'kAction'):
            raise ValueError('ground_maneuver_requires_ground_action')
        check_parameters(self.speed, self.yaw_rate, self.dt)
        if (not self.frame or not self.joint or self.dt < .001 or self.max_points < 2 or self.max_points > 20000 or
                any(not math.isfinite(v) or v <= 0 for v in (self.odom_timeout, self.tolerance,
                    self.yaw_tolerance, self.grace, self.validation_timeout))):
            raise ValueError('Invalid executor parameters')
        self.current = None
        self.last_valid = None
        self.odom_received = 0.
        self.odom_ros_received = 0.
        self.odom_stamp = 0.
        self.odom_clock_high_water = None
        self.odom_clock_invalidated = False
        self.measured_speed = math.inf
        self.measured_yaw_rate = math.inf
        self.measured_velocity = None
        self.known = 0
        self.revision = 0
        self.active = None
        self.validation_failure = None
        self.terminal_outcome = None
        self.reserved = False
        self.reserved_ground = None
        self.reserved_pre_execution_deadline = None
        self.sequence = 0
        self.continuation = None
        self.last_continuation = None
        self.last_continuation_rejection = None
        self.last_goal_rejection = None
        # Kept after an unconfirmed stop to prevent dispatching a second motion
        # while a delayed downstream goal or result can still be outstanding.
        self.output_in_flight = None
        self.shutting_down = False
        self.mission = None; self.mission_received = 0.
        self.mission_ros_received = 0.
        self.mission_source_stamp = None; self.mission_source_valid = False
        self.mission_source_diagnostic = 'missing_stamp_s'
        self.mission_source_rejections = 0
        self.mission_ros_clock_high_water = None; self.mission_ros_clock_invalidated = False
        self.state = 'IDLE'
        self.reason = 'ready'
        self.stats = {'accepted': 0, 'rejected': 0, 'succeeded': 0, 'cancelled': 0, 'aborted': 0,
                      'trajectory_commands': 0, 'hold_commands': 0, 'feedback': 0,
                      'action_commands': 0, 'action_cancels': 0,
                      'continuations': 0, 'continuation_rejections': 0}
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Volatile command stream prevents replaying old motion to a late driver.
        self.commands = self.create_publisher(MultiDOFJointTrajectory, 'command/trajectory', 10)
        self.output_action = (ActionClient(self, PathFollower, self.output_action_name)
                              if self.output_type == 'kAction' else None)
        self.path_pub = self.create_publisher(Path, 'execution_path', durable)
        self.status_pub = self.create_publisher(String, 'execution_status', durable)
        self.create_subscription(Odometry, 'odometry', self.odometry, 10)
        self.create_subscription(String, 'map_updates', self.map_update, durable)
        self.create_subscription(String, 'mission_status', self.receive_mission, durable)
        self.validator = self.create_client(ValidatePath, 'validate_path')
        self.continuation_group = ReentrantCallbackGroup()
        self.continuation_validator = self.create_client(ValidatePath, 'validate_path', callback_group=self.continuation_group)
        self.create_service(ContinuePath, 'continue_path', self.continue_path, callback_group=self.continuation_group)
        self.create_service(Trigger, 'execution_state', self.status)
        self.server = ActionServer(self, PathFollower, 'path_follower', execute_callback=self.execute,
                                   goal_callback=self.accept, cancel_callback=self.cancel)
        # A paused simulation must not freeze action/validation/stop deadlines.
        self.deadline_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02, self.tick, clock=self.deadline_clock)
        self.transition('IDLE', 'ready')

    def payload(self):
        owns_validation=(self.state=='EXECUTING' and self.active is not None and
                         self.active['start'] is not None and not self.active['stop'] and
                         self.active['validated']>0)
        return dict(self.stats, state=self.state, reason=self.reason, map_revision=self.revision,
                    known_cells=self.known, active=self.active is not None,
                    command_subscribers=self.commands.get_subscription_count(),
                    output_type=self.output_type, output_action=self.output_action_name,
                    output_ready=self.output_ready(),
                    odometry_freshness=self.odometry_freshness(),
                    mission_freshness=self.mission_freshness(),
                    output_in_flight=self.output_in_flight is not None,
                    output_unconfirmed=self.output_in_flight is not None and self.output_in_flight is not self.active,
                    generation=self.active['generation'] if self.active else None,
                    bound_mode=self.active['bound_mode'] if self.active else None,
                    manual_path=self.active['manual_path'] if self.active else None,
                    execution_mode=self.active.get('execution_mode', 0) if self.active else None,
                    allow_ground_maneuver=self.allow_ground_maneuver,
                    ground_maneuver_duration=self.active['duration'] if self.active and self.active.get('ground') else None,
                    speed_mps=self.speed,
                    active_speed_mps=self.active['speed'] if self.active else None,
                    trajectory_profile=self.trajectory_profile,
                    continuation_strategy=self.continuation_strategy,
                    topic_timing=self.topic_timing(),
                    goal_id=[int(v) for v in self.active['handle'].goal_id.uuid] if self.active else None,
                    validates_remaining_path=owns_validation,
                    validated_revision=self.active['validated'] if owns_validation else 0,
                    validation_failure=self.validation_failure,
                    terminal_outcome=self.terminal_outcome,
                    endpoint=list(self.active['connected'][-1]) if self.active and 'connected' in self.active else None,
                    continuation_pending=self.continuation is not None,
                    last_continuation_rejection=self.last_continuation_rejection,
                    last_goal_rejection=self.last_goal_rejection,
                    last_continuation=self.last_continuation)

    def transition(self, state, reason):
        self.state, self.reason = state, reason
        message = String(); message.data = json.dumps(self.payload()); self.status_pub.publish(message)

    def status(self, request, response):
        response.success = True; response.message = json.dumps(self.payload()); return response

    def odometry(self, message):
        try:
            current = state_from_pose(message.pose.pose)
            stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
            received, now, _, clock_regressed = self.odometry_clock()
            if message.header.frame_id != self.frame or stamp <= 0 or now-stamp > self.odom_timeout or stamp-now > .1:
                raise ValueError('stale_or_wrong_frame_odometry')
            if clock_regressed:
                raise ValueError('ros_clock_regressed')
            if stamp <= self.odom_stamp:
                return  # Duplicate/replayed samples do not refresh liveness.
            self.measured_speed = (math.dist(self.current[:3], current[:3])/(stamp-self.odom_stamp)
                                   if self.current is not None else math.inf)
            self.measured_yaw_rate = (abs(wrap(current[3]-self.current[3]))/(stamp-self.odom_stamp)
                                      if self.current is not None else math.inf)
            self.measured_velocity = (tuple((b-a)/(stamp-self.odom_stamp)
                                      for a,b in zip(self.current[:3],current[:3]))+
                                      (wrap(current[3]-self.current[3])/(stamp-self.odom_stamp),)
                                      if self.current is not None else None)
            self.current, self.odom_stamp, self.odom_received = current, stamp, received
            self.odom_ros_received = now
            self.odom_clock_invalidated = False
            self.last_valid = current
            if self.active and self.active.get('ground') and self.active['output_terminal']:
                self.observe_ground_stop(self.active)
            elif self.output_in_flight and self.output_in_flight.get('ground') and self.output_in_flight['output_terminal']:
                self.observe_ground_stop(self.output_in_flight)
            if self.active and self.active.get('start') is not None and not self.active['stop']:
                try:
                    if self.active['manual_path']:
                        manual_validation_route(self.active['connected'],current)
                    else:self.active['progress'].update(current[:3])
                except ValueError as error:self.stop(str(error),False)
        except (ValueError, TypeError):
            self.current = None

    def odometry_clock(self):
        received = time.monotonic()
        now = self.get_clock().now().nanoseconds*1e-9
        simulated = bool(self.get_parameter('use_sim_time').value)
        regressed = (simulated and self.odom_clock_high_water is not None and now < self.odom_clock_high_water)
        if simulated:
            if regressed:
                self.odom_clock_invalidated = True
            else:
                self.odom_clock_high_water = now
        return received, now, simulated, regressed

    def odometry_freshness(self):
        received, now, simulated, _ = self.odometry_clock()
        present = self.current is not None
        accepted = self.odom_stamp > 0.
        steady_age = received-self.odom_received if accepted else None
        ros_age = now-self.odom_ros_received if accepted else None
        source_age = now-self.odom_stamp if accepted else None
        receipt_age = ros_age if simulated else steady_age
        # External trackers own safe motor output while the simulation pauses.
        # Source stamps must still advance before any receipt time is renewed.
        fresh = (present and not self.odom_clock_invalidated and
                 0. <= receipt_age <= self.odom_timeout and -.1 <= source_age <= self.odom_timeout)
        return dict(fresh=fresh, odometry_present=present, use_sim_time=simulated,
                    freshness_clock='ros' if simulated else 'steady', deadline_clock='steady',
                    receipt_steady_age_s=steady_age, receipt_ros_age_s=ros_age, source_age_s=source_age,
                    clock_regressed=self.odom_clock_invalidated, odometry_timeout_s=self.odom_timeout,
                    last_accepted_source_stamp_s=self.odom_stamp if accepted else None,
                    tick_monotonic_s=received, tick_ros_s=now)

    def fresh(self):
        return self.odometry_freshness()['fresh']

    def trajectory_elapsed(self, active, steady_now=None):
        # Topic consumers interpret time_from_start against this exact header,
        # including after a continuation replaces the running trajectory.
        # fresh() owns clock-regression rejection before any motion decision.
        if active['trajectory_uses_ros_time']:
            return max(0., (self.get_clock().now().nanoseconds-active['trajectory_start_ros_ns'])*1e-9)
        return max(0., (time.monotonic() if steady_now is None else steady_now)-active['start'])

    def topic_timing(self):
        active = self.active
        if self.output_action is not None or not active or active['start'] is None:
            return None
        return dict(clock='ros' if active['trajectory_uses_ros_time'] else 'steady',
                    header_stamp_ns=active['trajectory_start_ros_ns'],
                    elapsed_s=self.trajectory_elapsed(active), duration_s=active['duration'],
                    endpoint_grace_s=self.grace)

    def output_ready(self):
        return (self.output_action.server_is_ready() if self.output_action is not None
                else bool(self.commands.get_subscription_count()))

    def map_update(self, message):
        try:
            data = json.loads(message.data)
            revision, known = int(data['map_revision']), int(data['known_cells'])
            if revision < 1 or known < 0:
                return
        except (ValueError, TypeError, KeyError):
            return
        reset = known == 0 or revision < self.revision
        self.revision, self.known = revision, known
        if reset and self.active:
            self.stop('map_reset', False)

    def receive_mission(self, message):
        try:
            status=json.loads(message.data)
            if not isinstance(status['enabled'],bool) or not isinstance(status['valid'],bool):return
            status['remaining_s']=float(status['remaining_s'])
            if not math.isfinite(status['remaining_s']):return
            now_ros=self.get_clock().now().nanoseconds*1e-9
            if not self.observe_mission_clock(now_ros):
                self.mission_source_rejections+=1
                self.mission_source_diagnostic='ros_clock_regressed';return
            source_stamp=None;source_valid=False;diagnostic='legacy_missing_stamp_s'
            previous=self.mission_source_stamp
            if 'stamp_s' in status:
                try:
                    raw_stamp=status['stamp_s']
                    if isinstance(raw_stamp,bool):raise ValueError('boolean_stamp')
                    source_stamp=float(raw_stamp)
                    if not math.isfinite(source_stamp) or source_stamp<0.:raise ValueError('invalid_stamp')
                except (ValueError,TypeError,OverflowError):
                    self.mission_source_rejections+=1;self.mission_source_diagnostic='invalid_stamp_s';return
                if source_stamp-now_ros>.1:
                    self.mission_source_rejections+=1;self.mission_source_diagnostic='future_stamp_s';return
                if previous is not None and source_stamp<=previous:
                    self.mission_source_rejections+=1
                    self.mission_source_diagnostic=('duplicate_stamp_s' if source_stamp==previous else 'out_of_order_stamp_s')
                    return
                # Paused repeated stamps never overwrite the stored budget or
                # refresh receipt; they remain fresh through the ROS clock.
                source_valid=True;diagnostic='ok'
            elif previous is not None:
                self.mission_source_rejections+=1
                self.mission_source_diagnostic='missing_stamp_s_after_stamped';return
            self.mission=status;self.mission_received=time.monotonic()
            self.mission_ros_received=now_ros
            self.mission_source_stamp=source_stamp;self.mission_source_valid=source_valid
            self.mission_source_diagnostic=diagnostic
            self.mission_ros_clock_invalidated=False
        except (ValueError,TypeError,KeyError,OverflowError):return

    def observe_mission_clock(self,now_ros):
        if self.mission_ros_clock_high_water is not None and now_ros<self.mission_ros_clock_high_water:
            self.mission_ros_clock_invalidated=True
            return False
        self.mission_ros_clock_high_water=now_ros
        # Clock catch-up alone cannot restore the previous budget snapshot.
        return True

    def mission_freshness(self,now_steady=None,now_ros=None):
        if self.mission is None:return None
        steady=time.monotonic() if now_steady is None else now_steady
        ros=self.get_clock().now().nanoseconds*1e-9 if now_ros is None else now_ros
        self.observe_mission_clock(ros)
        receipt_steady=steady-self.mission_received
        receipt_ros=ros-self.mission_ros_received
        source_age=(ros-self.mission_source_stamp if self.mission_source_valid else None)
        simulated=bool(self.get_parameter('use_sim_time').value)
        receipt_age=receipt_ros if simulated else receipt_steady
        clock_regressed=(self.mission_ros_clock_invalidated or receipt_ros<0. or receipt_steady<0.)
        fresh=(not clock_regressed and 0.<=receipt_age<=2. and
               (source_age is None or -.1<=source_age<=2.))
        age=receipt_age if source_age is None else max(receipt_age,source_age)
        return {'fresh':fresh,'age_s':age,'receipt_steady_age_s':receipt_steady,
                'receipt_ros_age_s':receipt_ros,'source_age_s':source_age,
                'age_clock':'ros' if simulated else 'steady+ros',
                'clock_regressed':clock_regressed,
                'source_valid':self.mission_source_valid,
                'source_diagnostic':self.mission_source_diagnostic}

    def continuation_budget(self, request, active, replacement_duration=None):
        enabled=request.budget_enabled or bool(self.mission and self.mission['enabled'])
        if not enabled:return
        if (not request.budget_enabled or not math.isfinite(request.required_time_after_root) or
                request.required_time_after_root<20.):raise ValueError('continuation_budget_missing')
        now=time.monotonic();now_ros=self.get_clock().now().nanoseconds*1e-9
        freshness=self.mission_freshness(now,now_ros)
        if (not self.mission or not self.mission['enabled'] or not self.mission['valid'] or
                freshness is None or not freshness['fresh']):
            raise ValueError('continuation_budget_unavailable')
        age=freshness['age_s']
        eta=max(0.,active['duration']-self.trajectory_elapsed(active,now))
        if replacement_duration is not None:
            if not math.isfinite(replacement_duration) or replacement_duration < 0:
                raise ValueError('invalid_replacement_duration')
            # ContinuePath's cost combines new travel, return and reserve. It
            # has no separate outbound cost to subtract. Retain that complete
            # reserve and charge the retimed command, rather than reusing an
            # ETA from the old, faster schedule. This may reserve travel twice.
            eta=max(eta,replacement_duration)
        # Charge remaining budget and travel ETA in the same trajectory clock.
        # Frozen /clock does not consume simulation mission seconds.
        if self.mission['remaining_s']-age<eta+request.required_time_after_root:
            raise ValueError('continuation_time_insufficient')

    def profile_start_velocity(self, continuation=False):
        if self.trajectory_profile == 'upstream':
            return None
        if continuation:
            if self.measured_velocity is None or any(not math.isfinite(v) for v in self.measured_velocity):
                raise ValueError('retiming_initial_velocity_unavailable')
            return self.measured_velocity
        if any(not math.isfinite(v) or v > .05 for v in (self.measured_speed, self.measured_yaw_rate)):
            raise ValueError('retiming_start_not_stationary')
        return None

    def profile_samples(self, connected, samples, speed, continuation=False):
        if self.trajectory_profile == 'upstream':
            return samples
        initial = self.profile_start_velocity(continuation)
        parameters = dict(speed=speed, yaw_rate=self.yaw_rate, dt=self.dt,
                          max_acceleration=self.max_acceleration,
                          max_yaw_acceleration=self.max_yaw_acceleration,
                          max_points=self.max_points,
                          initial_velocity=tuple(initial) if initial is not None else None,
                          merge_collinear=self.trajectory_profile == 'quintic_collinear')
        # Use this frozen input for the actual call, not odometry reconstructed
        # later by an observer. No asynchronous work occurs inside retiming.
        path = tuple(tuple(state) for state in connected) if continuation else connected
        try:
            return retime_polyline(path, **parameters)
        except ValueError as error:
            if continuation:
                error.retiming_input = {'schema_version': 1,
                    'trajectory_profile': self.trajectory_profile,
                    'path': [list(state) for state in path], 'parameters': parameters}
            raise

    def capture_retiming_rejection(self, error, active, candidate, map_revision):
        if not hasattr(error, 'retiming_input'):
            return
        self.last_continuation_rejection = {
            'stage': 'retiming', 'reason': str(error),
            'retiming_input': error.retiming_input,
            'retiming_details': getattr(error, 'details', None),
            'map_revision': map_revision, 'frame_id': self.frame,
            'validation_stamp_ns': candidate['validation_stamp_ns'],
            'validation_attempts': candidate['validation_attempts'],
            'checked_path': [list(p) for p in candidate['checked_path']],
            'extension': [list(p) for p in candidate['extension']],
            'previous_path': [list(p) for p in active['connected']],
            'position': list(self.current), 'progress_m': active['progress'].progress,
            'measured_velocity': list(self.measured_velocity), 'odom_stamp_s': self.odom_stamp,
            'generation': active['generation'],
            'goal_id': [int(v) for v in active['handle'].goal_id.uuid],
            'ros_s': self.get_clock().now().nanoseconds*1.e-9}

    def sample_duration(self, samples):
        # Inserted original vertices need not fall on the regular sample grid.
        # Their count must not be used as a substitute for trajectory duration.
        return samples.time_offsets[-1] if isinstance(samples, TimedSamples) else len(samples)*self.dt

    def accept(self, request):
        received_at = time.monotonic()
        try:
            if (self.shutting_down or self.reserved or self.active or not self.fresh() or self.known == 0 or
                    not self.validator.service_is_ready() or not self.output_ready() or
                    self.output_in_flight is not None):
                raise ValueError('executor_not_ready')
            if not 1 <= len(request.path) <= self.max_points:
                raise ValueError('invalid_path_size')
            if request.bound_mode not in range(5):
                raise ValueError('invalid_bound_mode')
            if any(p.header.frame_id != self.frame for p in request.path):
                raise ValueError('wrong_frame')
            mode = int(request.execution_mode)
            if mode not in (0, 1):
                raise ValueError('invalid_execution_mode')
            timeout = float(request.pre_execution_timeout)
            if not math.isfinite(timeout) or timeout < 0. or (mode == 1 and timeout <= 0.):
                raise ValueError('invalid_pre_execution_timeout')
            deadline = received_at+timeout if timeout > 0. else None
            ground = None
            if mode == 1:
                if not self.allow_ground_maneuver or self.robot_type != 'kGround' or self.output_type != 'kAction':
                    raise ValueError('ground_maneuver_disabled')
                if request.manual_path:
                    raise ValueError('ground_maneuver_cannot_be_manual')
                if any(abs(p.pose.orientation.x) > 1.e-8 or abs(p.pose.orientation.y) > 1.e-8 for p in request.path):
                    raise ValueError('ground_reference_not_planar')
                reference = decode_maneuver([state_from_pose(p.pose) for p in request.path],
                                           request.maneuver, caps=self.ground_caps)
                if (not math.isfinite(request.vel_max) or
                        (request.vel_max != 0. and abs(request.vel_max-reference.max_speed) > 1.e-9)):
                    raise ValueError('ground_velocity_contract_mismatch')
                validate_start(reference, self.current, self.measured_speed, self.measured_yaw_rate)
                speed = reference.max_speed
                ground = (reference, copy.deepcopy(request.maneuver), copy.deepcopy(request.path))
            else:
                speed = resolve_velocity(request.vel_max, self.speed)
                connected, samples = prepare([state_from_pose(p.pose) for p in request.path], self.current,
                        speed, self.yaw_rate, self.dt, self.smooth, self.max_points,
                        manual_path=request.manual_path)
                self.profile_samples(connected, samples, speed)
            if deadline is not None and time.monotonic() >= deadline:
                raise ValueError('pre_execution_timeout')
        except (ValueError, TypeError, OverflowError) as error:
            self.stats['rejected'] += 1
            self.last_goal_rejection = {
                'reason': str(error), 'ros_s': self.get_clock().now().nanoseconds*1.e-9,
                'odom_stamp_s': self.odom_stamp, 'state': self.state,
                'position': list(self.current) if self.current is not None else None,
                'measured_speed_mps': self.measured_speed if math.isfinite(self.measured_speed) else None,
                'measured_yaw_rate_rps': self.measured_yaw_rate if math.isfinite(self.measured_yaw_rate) else None,
                'retiming_details': getattr(error, 'details', None)}
            self.transition(self.state, self.reason)
            return GoalResponse.REJECT
        # Reserve synchronously, before the execute coroutine starts, so two
        # back-to-back goals cannot both pass the busy check.
        if ground is None:
            self.speed = speed
        self.reserved_ground = ground
        self.reserved_pre_execution_deadline = deadline
        self.reserved = True
        return GoalResponse.ACCEPT

    def cancel(self, handle):
        return CancelResponse.ACCEPT

    async def execute(self, handle):
        self.sequence += 1
        self.validation_failure = None
        self.terminal_outcome = None
        done = Future()
        ground = self.reserved_ground
        deadline = self.reserved_pre_execution_deadline
        self.reserved_ground = None; self.reserved_pre_execution_deadline = None
        self.active = dict(handle=handle, done=done, sequence=self.sequence, pending=False,
                           validated=0, start=None, stop=None, feedback_at=0., generation=0,
                           trajectory_start_ros_ns=None, trajectory_uses_ros_time=False,
                           output_handle=None, output_terminal=False, output_success=False,
                           output_cancel_sent=False,output_goal_id=None,output_status=None,
                           output_rejected=False,dispatched=False,validation_failure=None,
                           bound_mode=int(handle.request.bound_mode),
                           manual_path=bool(handle.request.manual_path),
                           speed=ground[0].max_speed if ground else self.speed,
                           execution_mode=int(handle.request.execution_mode),ground=ground[0] if ground else None,
                           maneuver=ground[1] if ground else None,approved_path=ground[2] if ground else None,
                           approved_vel_max=float(handle.request.vel_max),
                           pre_execution_deadline=deadline,ground_settled=0,ground_settle_since=None,
                           ground_settle_stamp=self.odom_stamp,
                           observed_prefix=None,validation_group=None)
        self.stats['accepted'] += 1
        try:
            if not self.fresh():
                raise ValueError('odometry_lost_before_dispatch')
            if handle.is_cancel_requested:
                self.stop('cancel_requested', True)
                success = await done
                result = PathFollower.Result(); result.success = success; return result
            if deadline is not None and time.monotonic() >= deadline:
                raise ValueError('pre_execution_timeout')
            if ground:
                validate_start(ground[0], self.current, self.measured_speed, self.measured_yaw_rate)
                connected = list(ground[0].path); samples = list(ground[0].path)
                duration = ground[0].max_duration
            else:
                connected, samples = prepare([state_from_pose(p.pose) for p in handle.request.path], self.current,
                    self.active['speed'], self.yaw_rate, self.dt, self.smooth, self.max_points,
                    manual_path=self.active['manual_path'])
                samples = self.profile_samples(connected, samples, self.active['speed'])
                duration = self.sample_duration(samples)
            self.active.update(connected=connected, samples=samples, duration=duration,
                               progress=PathProgress([state[:3] for state in connected]))
            if (not ground and self.robot_type=='kGround' and not self.active['manual_path'] and len(connected)>1
                    and tuple(connected[0][:3])==tuple(self.current[:3])
                    and math.dist(connected[0][:3],connected[1][:3])>1.e-9):
                self.active['observed_prefix']=tuple(tuple(point) for point in connected)
            self.transition('VALIDATING', 'validate_modified_path')
            self.validate()
        except (ValueError, TypeError) as error:
            not_dispatched=not self.active['dispatched']
            self.finish(False, str(error), stop_confirmed=not_dispatched,
                        confirmation='not_dispatched' if not_dispatched else 'unconfirmed')
        success = await done
        result = PathFollower.Result(); result.success = success; return result

    def validate(self, deadline=None):
        active = self.active
        if not active or active['pending'] or active['stop']:
            return
        if not self.validator.service_is_ready():
            self.stop('validation_service_unavailable', False); return
        deadline = time.monotonic()+self.validation_timeout if deadline is None else deadline
        if active['start'] is None and active.get('pre_execution_deadline') is not None:
            deadline = min(deadline, active['pre_execution_deadline'])
        if time.monotonic()>deadline:
            self.stop('validation_timeout', False); return
        request = ValidatePath.Request(); request.header.frame_id = self.frame
        request.bound_mode = active['bound_mode']
        request.path = [pose(p) for p in active['connected']]
        if active.get('ground') and active['start'] is None:
            if not self.fresh():
                self.stop('odometry_lost_before_validation', False); return
            # Validate the actual observed connector; never insert it into or
            # otherwise change the approved command/reference/progress route.
            request.path.insert(0, pose(self.current))
            request.first_pose_is_observation = True
        elif not active['manual_path'] and active['start'] is None:
            # prepare() explicitly prepends this follower's measured current
            # pose. Only its first connector has observation semantics; every
            # incoming command vertex stays fixed, including any PCI prefix.
            request.first_pose_is_observation = True
        if active['manual_path'] and active['start'] is None:
            # Manual command vertices stay unchanged, but validation still
            # covers the observed pose and its connector to the first vertex.
            if not self.fresh():
                self.stop('odometry_lost_before_validation', False); return
            request.path.insert(0, pose(self.current))
        routes = None
        if active['start'] is not None and self.current is not None:
            if active['manual_path']:
                request.path=[pose(p) for p in manual_validation_route(active['connected'],self.current)]
            else:
                routes=remaining_validation_routes(active['progress'], self.current, active['observed_prefix'])
                request.first_pose_is_observation=True
        requests = [request]
        if routes is not None:
            requests = []
            for route in routes:
                part = ValidatePath.Request(); part.header.frame_id = self.frame
                part.bound_mode = active['bound_mode']; part.first_pose_is_observation = True
                part.path = [pose(point) for point in route]
                requests.append(part)
        group = {'sequence': active['sequence'], 'generation': active['generation'], 'deadline': deadline,
                 'requests': requests, 'revisions': [None]*len(requests)}
        active['pending'] = True; active['validation_deadline'] = deadline
        active['validation_group'] = group
        for index, part in enumerate(requests):
            self.validator.call_async(part).add_done_callback(
                lambda future, i=index: self.validated(future, group, i))

    def validated(self, future, group, index):
        active = self.active
        if (not active or active['sequence'] != group['sequence'] or active['stop']
                or active['generation'] != group['generation'] or active['validation_group'] is not group):
            return
        try:
            if time.monotonic()>group['deadline']:
                self.stop('validation_timeout', False); return
            result = future.result()
            if not result.valid:
                rejected = group['requests'][index]
                self.validation_failure = {
                    'map_revision': result.map_revision, 'reason': result.reason,
                    'first_pose_is_observation': rejected.first_pose_is_observation,
                    'path': [list(state_from_pose(point)) for point in rejected.path[:256]],
                    'path_points': len(rejected.path), 'path_truncated': len(rejected.path)>256}
                active['validation_failure'] = self.validation_failure
                self.stop('path_invalidated:'+result.reason, False); return
            group['revisions'][index] = result.map_revision
            if any(revision is None for revision in group['revisions']):
                return
            active['pending'] = False; active['validation_group'] = None
            active['validated'] = min(group['revisions'])
            active['validation_latest_revision'] = max(group['revisions'])
            if active['validated'] < self.revision:
                self.validate(deadline=group['deadline']); return
            if active['start'] is None:
                if active['handle'].is_cancel_requested:
                    self.stop('cancel_requested', True); return
                if not self.fresh() or math.dist(self.current[:3], active['samples'][0][:3]) > self.tolerance:
                    self.stop('start_moved_during_validation', False); return
                if active.get('ground'):
                    validate_start(active['ground'], self.current, self.measured_speed, self.measured_yaw_rate)
                else:
                    self.profile_start_velocity()
                if active['manual_path']:
                    # Reject start drift while the asynchronous validation was
                    # running; preserve every manual waypoint instead of
                    # reconnecting the command to a new pose.
                    prepare(active['connected'], self.current, active['speed'],
                            self.yaw_rate, self.dt, self.smooth, self.max_points,
                            manual_path=True)
                if not self.output_ready():
                    self.stop('output_consumer_missing', False); return
                if self.output_action is not None:
                    self.dispatch_action(active)
                else:
                    active['dispatched'] = True
                    active['trajectory_start_ros_ns'] = self.publish_trajectory(active['samples'])
                    active['trajectory_uses_ros_time'] = bool(self.get_parameter('use_sim_time').value)
                    active['start'] = time.monotonic()
                    self.transition('EXECUTING', 'trajectory_published')
            candidate = self.continuation
            if candidate is not None and candidate['sequence'] == active['sequence']:
                waiting = candidate.pop('waiting_active_validation', None)
                if waiting is not None:
                    self.continuation_validated(waiting, candidate)
        except Exception as error:
            self.stop('validation_error:'+str(error), False)

    async def continue_path(self, request, response):
        """Commit an extension inside the running action; never cancel/restart it.

        UUID + generation binds the request to one exact route. Validation is
        asynchronous, so odometry, obstacle rechecks and cancellation continue.
        The caller receives the actual stitched route only after publication.
        """
        active = self.active
        timed_extension = None
        try:
            # ROS action goals do not define in-place route replacement. The
            # PCI keeps the current goal and replans after its result instead.
            if self.output_action is not None:
                raise ValueError('external_action_cannot_continue')
            if (not active or active['start'] is None or active['stop'] or
                    active['handle'].is_cancel_requested or not self.fresh()):
                raise ValueError('no_running_trajectory')
            if active['manual_path']:
                raise ValueError('manual_path_cannot_continue')
            if list(request.goal_id) != list(active['handle'].goal_id.uuid):
                raise ValueError('stale_goal')
            if request.expected_generation != active['generation']:
                raise ValueError('stale_generation')
            if self.continuation is not None:
                raise ValueError('continuation_busy')
            if request.header.frame_id != self.frame:
                raise ValueError('wrong_frame')
            if not 2 <= len(request.path) <= self.max_points:
                raise ValueError('invalid_path_size')
            self.continuation_budget(request, active)
            extension = [state_from_pose(p) for p in request.path]
            if self.continuation_strategy == 'append_timed' and self.trajectory_profile != 'upstream':
                full_path, combined, extension = prepare_timed_continuation(
                    active['connected'], active['samples'], extension,
                    active['speed'], self.yaw_rate, self.dt,
                    self.max_acceleration, self.max_yaw_acceleration, self.max_points,
                    self.smooth, self.trajectory_profile == 'quintic_collinear')
                timed_extension = (full_path, combined)
            prepare_continuation(active['connected'], active['progress'].progress, extension,
                                 self.current, active['speed'], self.yaw_rate, self.dt,
                                 False if timed_extension is not None else self.smooth, self.max_points)
            if not self.continuation_validator.service_is_ready():
                raise ValueError('validation_service_unavailable')
        except (ValueError, TypeError, OverflowError) as error:
            response.reason = str(error); self.stats['continuation_rejections'] += 1
            return response
        done = Future()
        self.continuation = dict(done=done, response=response, sequence=active['sequence'],
                                 extension=extension, request=request, deadline=time.monotonic()+self.validation_timeout,
                                 timed_extension=timed_extension,
                                 started_wall=time.monotonic(), started_ros=self.get_clock().now().nanoseconds*1.e-9,
                                 validation_attempts=0)
        self.validate_continuation()
        return await done

    def reject_continuation(self, reason):
        candidate = self.continuation
        if candidate is None:
            return
        self.continuation = None
        response = candidate['response']; response.reason = reason
        self.stats['continuation_rejections'] += 1
        candidate['done'].set_result(response)

    def validate_continuation(self):
        active, candidate = self.active, self.continuation
        if candidate is None:
            return
        if (not active or active['sequence'] != candidate['sequence'] or active['stop'] or
                active['handle'].is_cancel_requested or not self.fresh()):
            self.reject_continuation('trajectory_changed'); return
        request = ValidatePath.Request(); request.header.frame_id = self.frame
        # ContinuePath extends this active action, including its collision
        # model. A second request cannot change the robot body mid-flight.
        request.header.stamp = self.get_clock().now().to_msg()
        request.bound_mode = active['bound_mode']
        try:
            connected, _ = prepare_continuation(active['connected'], active['progress'].progress,
                candidate['extension'], self.current, active['speed'], self.yaw_rate, self.dt,
                False if candidate.get('timed_extension') is not None else self.smooth, self.max_points)
        except ValueError as error:
            self.reject_continuation(str(error));return
        # Check the actual modified route. Only its moving first connector gets
        # a small tube; every later segment is fixed and checked at robot size.
        # Commit must stay inside this exact connector or validation restarts.
        request.clearance_margin = .02;request.clearance_segments = 1
        request.path = [pose(p) for p in connected]
        candidate['validation_attempts'] += 1
        candidate['checked_path'] = connected
        candidate['validation_stamp_ns'] = request.header.stamp.sec*1000000000+request.header.stamp.nanosec
        self.continuation_validator.call_async(request).add_done_callback(
            lambda future: self.continuation_validated(future, candidate))

    def continuation_validated(self, future, candidate):
        if candidate is not self.continuation:
            return
        active = self.active
        if (not active or active['sequence'] != candidate['sequence'] or active['stop'] or
                active['handle'].is_cancel_requested or not self.fresh()):
            self.reject_continuation('trajectory_changed'); return
        try:
            result = future.result()
            if not result.valid:
                self.last_continuation_rejection = {
                    'reason':result.reason,'map_revision':result.map_revision,
                    'failure_segment':result.failure_segment,
                    'validation_stamp_ns':candidate['validation_stamp_ns'],
                    'checked_path':candidate['checked_path'],
                    'extension':candidate['extension'],
                    'previous_path':active['connected'],
                    'position':list(self.current),'progress_m':active['progress'].progress,
                    'generation':active['generation'],'goal_id':[int(v) for v in active['handle'].goal_id.uuid],
                    'ros_s':self.get_clock().now().nanoseconds*1.e-9}
                self.reject_continuation('extension_invalid:'+result.reason); return
            if time.monotonic() > candidate['deadline']:
                self.reject_continuation('continuation_timeout'); return
            preserve_timing = candidate.get('timed_extension') is not None
            known_revision = self.revision
            if preserve_timing:
                # The appended command retains the old progress guard as well
                # as its forward suffix. Require that old-route check and the
                # extension check at one common newest observed map revision.
                # A service reply may precede the map_updates notification.
                known_revision = max(known_revision, active['validated'],
                    active.get('validation_latest_revision', active['validated']), result.map_revision)
                if active['pending']:
                    candidate['waiting_active_validation'] = future
                    return
                if active['validated'] < known_revision:
                    candidate['waiting_active_validation'] = future
                    self.validate(deadline=candidate['deadline'])
                    return
            if result.map_revision < known_revision:
                self.validate_continuation(); return
            self.continuation_budget(candidate['request'], active)
            # Re-cut within the checked tube at measured progress at commit.
            connected, samples = prepare_continuation(active['connected'], active['progress'].progress,
                candidate['extension'], self.current, active['speed'], self.yaw_rate, self.dt,
                False if candidate.get('timed_extension') is not None else self.smooth, self.max_points)
            if not continuation_within_checked(candidate['checked_path'], connected, .02):
                self.validate_continuation(); return
            if preserve_timing:
                # The checked suffix follows the same XYZ vertices. Preserve
                # every old command and its clock; append only after its known
                # rest endpoint rather than braking at a short measured splice.
                connected, samples = candidate['timed_extension']
                if self.trajectory_elapsed(active) >= active['duration']-self.dt:
                    raise ValueError('continuation_root_too_close')
                old_progress = active['progress']
                progress = PathProgress([p[:3] for p in connected],
                    old_progress.tolerance, old_progress.guard, old_progress.max_step)
                if (progress.points[:len(old_progress.points)] != old_progress.points or
                        progress.arc[:len(old_progress.arc)] != old_progress.arc):
                    raise ValueError('continuation_prefix_changed')
                progress.progress, progress.previous = old_progress.progress, old_progress.previous
            else:
                try:
                    samples = self.profile_samples(connected, samples, active['speed'], continuation=True)
                except ValueError as error:
                    self.capture_retiming_rejection(error, active, candidate, result.map_revision)
                    raise
                progress = PathProgress([p[:3] for p in connected])
            if self.trajectory_profile != 'upstream':
                replacement_remaining = self.sample_duration(samples)
                if preserve_timing:
                    replacement_remaining = max(0., replacement_remaining-self.trajectory_elapsed(active))
                self.continuation_budget(candidate['request'], active, replacement_remaining)
            if not self.commands.get_subscription_count():
                self.reject_continuation('trajectory_consumer_missing'); return
            reference_at_commit = None
            if preserve_timing:
                # RMF selects the last sample at or before the acquisition
                # stamp, not the first sample in a republished message.
                elapsed_ns = max(0, round(self.odom_stamp*1.e9)-active['trajectory_start_ros_ns'])
                offsets_ns = [round(offset*1.e9) for offset in active['samples'].time_offsets]
                reference_index = max(0, bisect.bisect_right(offsets_ns, elapsed_ns)-1)
                reference_at_commit = {
                    'clock': 'odometry_acquisition', 'sample_index': reference_index,
                    'odom_stamp_s': self.odom_stamp,
                    'state': list(samples[reference_index]),
                    'velocity': list(samples.velocities[reference_index]),
                    'acceleration': list(samples.accelerations[reference_index]),
                    'offset_s': samples.time_offsets[reference_index],
                    'old_and_new_identical': (
                        samples[reference_index] == active['samples'][reference_index] and
                        samples.velocities[reference_index] == active['samples'].velocities[reference_index] and
                        samples.accelerations[reference_index] == active['samples'].accelerations[reference_index] and
                        samples.time_offsets[reference_index] == active['samples'].time_offsets[reference_index])}
            self.last_continuation = {
                'strategy':'append_timed' if preserve_timing else 'reanchor',
                'commit_ros_s':self.get_clock().now().nanoseconds*1.e-9,
                'old_sample_count':len(active['samples']), 'new_sample_count':len(samples),
                'old_trajectory_command_count':self.stats['trajectory_commands'],
                'new_trajectory_command_count':self.stats['trajectory_commands']+1,
                'old_generation':active['generation'],'new_generation':active['generation']+1,
                'goal_id':[int(v) for v in active['handle'].goal_id.uuid],
                'old_trajectory_stamp_ns':active['trajectory_start_ros_ns'],
                'old_duration_s':active['duration'],'old_elapsed_s':self.trajectory_elapsed(active),
                'hold_commands':self.stats['hold_commands'],
                'old_remaining_time_s':max(0.,active['duration']-self.trajectory_elapsed(active)),
                'old_endpoint_distance_m':math.dist(self.current[:3],active['connected'][-1][:3]),
                'measured_speed_mps':self.measured_speed,'odom_stamp_s':self.odom_stamp,
                'new_duration_s':self.sample_duration(samples),'trajectory_profile':self.trajectory_profile,
                'measured_velocity':self.measured_velocity,
                'initial_reference_velocity':samples.velocities[0] if isinstance(samples,TimedSamples) else None,
                'initial_reference_velocity_scope':'trajectory_origin',
                'reference_at_commit':reference_at_commit,
                'old_route_validated_revision':active['validated'],
                'position':list(self.current),'validation_attempts':candidate['validation_attempts'],
                'validation_wall_s':time.monotonic()-candidate['started_wall'],
                'validation_ros_s':self.get_clock().now().nanoseconds*1.e-9-candidate['started_ros'],
                'map_revision':result.map_revision}
            self.sequence += 1
            header_stamp_ns = self.publish_trajectory(samples,
                origin_stamp_ns=active['trajectory_start_ros_ns'] if preserve_timing else None)
            self.last_continuation['new_trajectory_stamp_ns'] = header_stamp_ns
            active.update(sequence=self.sequence, generation=active['generation']+1, connected=connected,
                          samples=samples, duration=self.sample_duration(samples),
                          start=active['start'] if preserve_timing else time.monotonic(),
                          trajectory_start_ros_ns=header_stamp_ns,
                          trajectory_uses_ros_time=bool(self.get_parameter('use_sim_time').value),
                          progress=progress, pending=False,
                          validated=result.map_revision, feedback_at=0., observed_prefix=None,
                          validation_latest_revision=result.map_revision,
                          validation_group=None)
            self.stats['continuations'] += 1
            response = candidate['response']; response.accepted = True; response.reason = 'trajectory_replaced'
            response.generation = active['generation']; response.map_revision = result.map_revision
            response.path = [pose(p) for p in connected]
            self.continuation = None
            self.transition('EXECUTING', 'continuation_committed')
            candidate['done'].set_result(response)
        except ValueError as error:
            self.reject_continuation(str(error))
            if hasattr(error, 'retiming_input'):
                # Publish the rejection while the original action continues;
                # observers must not wait until its eventual endpoint to see it.
                self.transition(self.state, self.reason)
        except Exception as error:
            self.reject_continuation('continuation_error:'+str(error))

    def publish_trajectory(self, samples, hold=False, origin_stamp_ns=None):
        message = MultiDOFJointTrajectory()
        message.header.frame_id = self.frame
        if origin_stamp_ns is None:
            message.header.stamp = self.get_clock().now().to_msg()
        else:
            if hold or not isinstance(samples, TimedSamples) or type(origin_stamp_ns) is not int or origin_stamp_ns <= 0:
                raise ValueError('invalid_preserved_trajectory_origin')
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(origin_stamp_ns, 1000000000)
        message.joint_names = [self.joint]
        timed = isinstance(samples, TimedSamples) and not hold
        if timed and not (len(samples) == len(samples.time_offsets) ==
                          len(samples.velocities) == len(samples.accelerations)):
            raise ValueError('trajectory_sample_shape')
        previous_ns = 0
        for index, state in enumerate(samples):
            point = MultiDOFJointTrajectoryPoint()
            transform = Transform(); transform.translation.x, transform.translation.y, transform.translation.z = state[:3]
            transform.rotation.z, transform.rotation.w = math.sin(state[3]/2), math.cos(state[3]/2)
            point.transforms = [transform]
            velocity, acceleration = Twist(), Twist()
            if timed:
                velocity.linear.x, velocity.linear.y, velocity.linear.z = samples.velocities[index][:3]
                velocity.angular.z = samples.velocities[index][3]
                acceleration.linear.x, acceleration.linear.y, acceleration.linear.z = samples.accelerations[index][:3]
                acceleration.angular.z = samples.accelerations[index][3]
            # The upstream profile and stop commands retain mav_msgs' zero
            # derivatives. The explicit quintic profile publishes its own
            # analytic derivatives with the same sampled time law.
            point.velocities = [velocity]; point.accelerations = [acceleration]
            offset = samples.time_offsets[index] if timed else (index+1)*self.dt
            if not math.isfinite(offset) or offset <= 0.:
                raise ValueError('trajectory_sample_time')
            nanoseconds = round(offset*1e9)
            if nanoseconds <= previous_ns:
                raise ValueError('trajectory_sample_time')
            previous_ns = nanoseconds
            point.time_from_start.sec = nanoseconds//1000000000
            point.time_from_start.nanosec = nanoseconds%1000000000
            message.points.append(point)
        self.commands.publish(message)
        self.stats['hold_commands' if hold else 'trajectory_commands'] += 1
        path = Path(); path.header = message.header
        if not hold:
            for state in samples:
                stamped = PoseStamped(); stamped.header = path.header; stamped.pose = pose(state); path.poses.append(stamped)
        self.path_pub.publish(path)
        return message.header.stamp.sec*1000000000+message.header.stamp.nanosec

    def dispatch_action(self, active):
        goal = PathFollower.Goal()
        goal.bound_mode = active['bound_mode']; goal.manual_path = active['manual_path']
        goal.vel_max = active['speed']
        goal.execution_mode = active.get('execution_mode', 0)
        deadline = active.get('pre_execution_deadline')
        if deadline is not None:
            remaining = deadline-time.monotonic()
            if remaining <= 0.:
                self.stop('pre_execution_timeout', False); return
            goal.pre_execution_timeout = remaining
        path = Path(); path.header.frame_id = self.frame
        path.header.stamp = self.get_clock().now().to_msg()
        if active.get('ground'):
            validate_start(active['ground'], self.current, self.measured_speed, self.measured_yaw_rate)
            goal.vel_max = active['approved_vel_max']
            goal.maneuver = copy.deepcopy(active['maneuver'])
            path.poses = copy.deepcopy(active['approved_path'])
            active['trajectory_start_ros_ns'] = self.get_clock().now().nanoseconds
            active['trajectory_uses_ros_time'] = bool(self.get_parameter('use_sim_time').value)
        else:
            for state in active['samples']:
                stamped = PoseStamped(); stamped.header = path.header; stamped.pose = pose(state)
                path.poses.append(stamped)
        goal.path = path.poses
        if active['handle'].is_cancel_requested:
            self.stop('cancel_requested', True); return
        if deadline is not None:
            goal.pre_execution_timeout = deadline-time.monotonic()
            if goal.pre_execution_timeout <= 0.:
                self.stop('pre_execution_timeout', False); return
        active['start'] = time.monotonic()
        active['output_deadline'] = active['start']+self.validation_timeout
        if deadline is not None:
            active['output_deadline'] = min(active['output_deadline'], deadline)
        active['dispatched'] = True
        self.output_in_flight = active
        try:
            future = self.output_action.send_goal_async(goal,
                feedback_callback=lambda message: self.action_feedback(message, active))
        except Exception:
            # A transport exception does not prove that the downstream goal
            # was never received. Retain ownership until a real terminal reply.
            raise
        future.add_done_callback(lambda reply: self.action_accepted(reply, active))
        self.path_pub.publish(path)
        self.stats['action_commands'] += 1
        self.transition('DISPATCHING', 'external_action_sent')

    def action_accepted(self, future, active):
        try:
            handle = future.result()
            active['output_goal_id'] = [int(v) for v in handle.goal_id.uuid]
            if not handle.accepted:
                active['output_rejected'] = True
                self.action_terminal(active, False, 'external_action_rejected'); return
            active['output_handle'] = handle
            handle.get_result_async().add_done_callback(lambda reply: self.action_result(reply, active))
            # A cancel can arrive before the send-goal reply. Never allow that
            # delayed acceptance to resurrect motion after the outer stop.
            if self.active is not active or active['stop']:
                self.cancel_output(active)
            else:
                self.transition('EXECUTING', 'external_action_accepted')
        except Exception as error:
            if self.active is active:
                self.stop('external_action_reply_error:'+str(error), False)

    def action_feedback(self, message, active):
        if self.active is not active or active['stop'] or active['output_terminal']:
            return
        feedback = message.feedback
        if (feedback.remaining_waypoints < 0 or any(not math.isfinite(v) or v < 0 for v in
                (feedback.dist_to_goal, feedback.estimated_time_remaining))):
            return
        if active.get('ground'):
            # PCI admission reserves the approved total maneuver budget. Do
            # not silently replace it with path-length/speed or a shorter ETA.
            feedback.estimated_time_remaining = max(0., active['duration']-self.trajectory_elapsed(active))
        # Ordinary trackers continue to own their independent ETA contract.
        active['handle'].publish_feedback(feedback)
        active['feedback_at'] = time.monotonic(); self.stats['feedback'] += 1

    def action_result(self, future, active):
        try:
            result = future.result()
            if result.status not in (GoalStatus.STATUS_SUCCEEDED,GoalStatus.STATUS_CANCELED,
                                      GoalStatus.STATUS_ABORTED):
                if self.active is active:
                    self.stop('external_action_invalid_result_status:'+str(result.status), False)
                return
            active['output_status'] = int(result.status)
            success = result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success
            self.action_terminal(active, success, 'external_action_result:'+str(result.status))
        except Exception as error:
            if self.active is active:
                self.stop('external_action_result_error:'+str(error), False)

    def action_terminal(self, active, success, reason):
        if (not active.get('output_rejected',False) and active.get('output_status') not in
                (GoalStatus.STATUS_SUCCEEDED,GoalStatus.STATUS_CANCELED,GoalStatus.STATUS_ABORTED)):
            return
        active['output_terminal'] = True; active['output_success'] = success
        if active.get('ground'):
            active['ground_settled'] = 0; active['ground_settle_since'] = None
            active['ground_settle_stamp'] = self.odom_stamp
        elif self.output_in_flight is active:
            self.output_in_flight = None
        if self.active is active and not active['stop']:
            if success:
                active['result_at'] = time.monotonic()
                self.transition('EXECUTING', 'external_success_waiting_for_odometry')
            else:
                self.stop(reason, False)

    def observe_ground_stop(self, active):
        """Count new stationary observations after the downstream terminal."""
        if self.odom_stamp <= active['ground_settle_stamp']:
            return
        if self.odom_stamp-active['ground_settle_stamp'] > self.odom_timeout:
            active['ground_settled'] = 0; active['ground_settle_since'] = None
        active['ground_settle_stamp'] = self.odom_stamp
        if self.measured_speed <= .02 and self.measured_yaw_rate <= .03:
            if active['ground_settle_since'] is None:
                active['ground_settle_since'] = self.odom_stamp
            active['ground_settled'] += 1
        else:
            active['ground_settled'] = 0; active['ground_settle_since'] = None

    def ground_stop_confirmed(self, active):
        if not self.fresh():
            active['ground_settled'] = 0; active['ground_settle_since'] = None
            return False
        since = active['ground_settle_since']
        return (active['output_terminal'] and active['ground_settled'] >= 3
                and since is not None and self.odom_stamp-since >= .1-1.e-9)

    def cancel_output(self, active):
        if active['output_terminal'] or active['output_cancel_sent'] or active['output_handle'] is None:
            return
        active['output_cancel_sent'] = True
        self.stats['action_cancels'] += 1
        try:
            # Acceptance of a cancel is not proof that the robot has stopped.
            # tick waits for both a terminal action result and stationary odom.
            active['output_handle'].cancel_goal_async()
        except Exception as error:
            self.get_logger().error('External cancel failed: '+str(error))

    def stop(self, reason, cancelled):
        active = self.active
        if not active or active['stop']:
            return
        self.reject_continuation('trajectory_stopping')
        active['pending'] = False
        if not active['dispatched']:
            self.finish(False, reason, cancelled, stop_confirmed=True,
                        confirmation='not_dispatched'); return
        hold = self.current if self.current is not None else self.last_valid
        active['stop'] = dict(reason=reason, cancelled=cancelled, at=time.monotonic(),
                              stamp=self.odom_stamp, pose=hold, settled=0, last_stamp=self.odom_stamp,
                              hold_sent=False, timeout=self.odom_timeout)
        if active.get('ground'):
            reference = active['ground']
            forward, yaw_rate = reference.max_speed, reference.max_yaw_rate
            if self.measured_velocity is not None and self.current is not None:
                vx, vy, _, omega = self.measured_velocity
                measured_forward = math.cos(self.current[3])*vx+math.sin(self.current[3])*vy
                if math.isfinite(measured_forward):
                    forward = max(forward, abs(measured_forward))
                if math.isfinite(omega):
                    yaw_rate = max(yaw_rate, abs(omega))
            # Freeze once; a late cancel ACK or terminal cannot renew this
            # deadline. Actual translation/yaw observations still prove rest.
            active['stop']['timeout'] += max(forward/reference.max_acceleration,
                yaw_rate/reference.max_yaw_acceleration) + .15
        if self.output_action is not None:
            self.cancel_output(active)
            path = Path(); path.header.frame_id = self.frame; self.path_pub.publish(path)
        elif hold is not None:
            try:
                self.publish_trajectory([hold], hold=True)
                active['stop']['hold_sent'] = True
            except Exception as error:
                self.get_logger().error('Hold publication failed: '+str(error))
        self.transition('STOPPING', reason)

    def finish(self, success, reason, cancelled=False, *, stop_confirmed=False,
               confirmation='unconfirmed'):
        active = self.active
        if not active:
            return
        self.reject_continuation('trajectory_finished')
        handle = active['handle']
        action_status=(GoalStatus.STATUS_SUCCEEDED if success else
                       GoalStatus.STATUS_CANCELED if cancelled and handle.is_cancel_requested else
                       GoalStatus.STATUS_ABORTED)
        failure=active.get('validation_failure')
        # Capture this input goal before clearing active. The outcome is never
        # upgraded by a late downstream reply or associated with a later goal.
        self.terminal_outcome = {
            'goal_id':[int(v) for v in handle.goal_id.uuid],
            'generation':int(active['generation']), 'action_status':int(action_status),
            'success':bool(success), 'reason':reason,
            'failure_map_revision':int(failure['map_revision']) if failure else None,
            'validated_revision':int(active['validated']), 'dispatched':bool(active['dispatched']),
            'stop_confirmed':bool(stop_confirmed),
            'confirmation':confirmation if stop_confirmed else 'unconfirmed',
            'stop_stamp_s':float(self.odom_stamp) if stop_confirmed else None,
            'finished_ros_s':self.get_clock().now().nanoseconds*1.e-9,
            'output_type':self.output_type,
            'output_goal_id':list(active['output_goal_id']) if active['output_goal_id'] is not None else None,
            'output_status':active['output_status']}
        if success:
            handle.succeed(); self.stats['succeeded'] += 1
        elif cancelled and handle.is_cancel_requested:
            handle.canceled(); self.stats['cancelled'] += 1
        else:
            handle.abort(); self.stats['aborted'] += 1
        if active.get('ground') and stop_confirmed and self.output_in_flight is active:
            self.output_in_flight = None
        self.active = None; self.reserved = False
        self.transition('DONE' if success else ('CANCELLED' if cancelled else 'ERROR'), reason)
        active['done'].set_result(success)

    def tick(self):
        active = self.active
        if not active:
            pending = self.output_in_flight
            if pending and pending.get('ground') and self.ground_stop_confirmed(pending):
                # Release only the motion ownership latch. The earlier input
                # action's unconfirmed terminal outcome remains immutable.
                self.output_in_flight = None
            return
        if active['handle'].is_cancel_requested and not active['stop']:
            self.stop('cancel_requested', True)
        if self.active is None:
            return
        now = time.monotonic()
        if self.continuation and now > self.continuation['deadline']:
            self.reject_continuation('continuation_timeout')
        stop = active['stop']
        if stop:
            fresh=self.fresh()
            if not fresh:
                stop['settled'] = 0
            if (fresh and stop['pose'] is not None and self.odom_stamp > stop['last_stamp']):
                stop['last_stamp'] = self.odom_stamp
                stationary = (self.measured_speed <= .05 and self.measured_yaw_rate <= .05)
                if self.output_action is None:
                    stationary = (stationary and math.dist(self.current[:3], stop['pose'][:3]) <= self.tolerance
                                  and abs(wrap(self.current[3]-stop['pose'][3])) <= self.yaw_tolerance)
                stop['settled'] = stop['settled']+1 if stationary else 0
            terminal=(active['output_terminal'] if self.output_action is not None else stop['hold_sent'])
            ground_confirmed = not active.get('ground') or self.ground_stop_confirmed(active)
            if fresh and stop['settled'] >= 3 and terminal and ground_confirmed:
                confirmation=('action_terminal_and_stationary_odometry' if self.output_action is not None
                              else 'topic_hold_and_stationary_odometry')
                self.finish(False, stop['reason'], stop['cancelled'], stop_confirmed=True,
                            confirmation=confirmation)
            elif now-stop['at'] > stop.get('timeout', self.odom_timeout):
                suffix = ':hold_unconfirmed' if self.output_action is None else ':external_stop_unconfirmed'
                self.finish(False, stop['reason']+suffix, False)
            return
        if not self.fresh():
            self.stop('odometry_lost', False); return
        if not self.output_ready() and not active['output_terminal']:
            self.stop('output_consumer_lost', False); return
        if active['pending'] and now > active['validation_deadline']:
            self.stop('validation_timeout', False); return
        if active['start'] is None:
            if active.get('pre_execution_deadline') is not None and now >= active['pre_execution_deadline']:
                self.stop('pre_execution_timeout', False)
            return
        if not active['pending'] and self.revision > active['validated']:
            self.validate()
        if self.active is not active or active['stop']:
            return
        if self.output_action is not None:
            self.tick_action(active, now)
            return
        elapsed = self.trajectory_elapsed(active, now)
        samples = active['samples']
        passed = (bisect.bisect_right(samples.time_offsets, elapsed)
                  if isinstance(samples, TimedSamples) else max(0, int(elapsed/self.dt)))
        index = min(len(samples)-1, passed)
        if now-active['feedback_at'] >= .1:
            feedback = PathFollower.Feedback()
            feedback.remaining_waypoints = len(samples)-index
            feedback.dist_to_goal = float(math.dist(self.current[:3], samples[-1][:3]))
            feedback.estimated_time_remaining = max(0., active['duration']-elapsed)
            active['handle'].publish_feedback(feedback); active['feedback_at'] = now; self.stats['feedback'] += 1
        if elapsed >= active['duration']:
            endpoint = math.dist(self.current[:3], samples[-1][:3]) <= self.tolerance
            heading = abs(wrap(self.current[3]-samples[-1][3])) <= self.yaw_tolerance
            if endpoint and heading and self.measured_speed <= .05 and self.measured_yaw_rate <= .05 and not active['pending']:
                self.finish(True, 'endpoint_observed', stop_confirmed=True,
                            confirmation='endpoint_observed'); return
        if elapsed > active['duration']+self.grace:
            self.stop('endpoint_timeout', False)

    def tick_action(self, active, now):
        if active['output_handle'] is None and now > active['output_deadline']:
            self.stop('external_action_reply_timeout', False); return
        if active.get('ground') and self.trajectory_elapsed(active, now) >= active['duration']:
            self.stop('ground_maneuver_timeout', False); return
        if active['output_success']:
            target = active['samples'][-1]
            if (math.dist(self.current[:3], target[:3]) <= self.tolerance and
                    abs(wrap(self.current[3]-target[3])) <= self.yaw_tolerance and
                    self.measured_speed <= .05 and self.measured_yaw_rate <= .05 and not active['pending'] and
                    (not active.get('ground') or self.ground_stop_confirmed(active))):
                self.finish(True, 'external_action_endpoint_observed', stop_confirmed=True,
                            confirmation='endpoint_observed'); return
            if now-active['result_at'] > self.grace:
                self.stop('external_action_endpoint_unconfirmed', False)
        # An external tracker owns its timing. Do not apply the topic sample
        # duration to pure pursuit or other independently paced controllers.
        elif active.get('ground'):
            return  # Ground's approved motion-clock budget was checked above.
        elif self.action_timeout > 0. and now-active['start'] > self.action_timeout:
            self.stop('external_action_timeout', False)

    def shutdown_output(self):
        self.shutting_down = True
        if self.output_action is None:
            if self.active and self.last_valid is not None:
                self.publish_trajectory([self.last_valid], hold=True)
            return
        active = self.active or self.output_in_flight
        if active:
            self.stop('shutdown', False)
            self.cancel_output(active)
            # Give the action cancellation/result exchange a bounded chance
            # to finish before destroying DDS entities.
            deadline = (active['stop']['at']+active['stop'].get('timeout', self.odom_timeout)
                        if active.get('ground') and active.get('stop') else time.monotonic()+self.odom_timeout)
            while (rclpy.ok() and time.monotonic() < deadline and
                   (not self.ground_stop_confirmed(active) if active.get('ground') else not active['output_terminal'])):
                rclpy.spin_once(self, timeout_sec=.02)


def main():
    # Keep DDS alive while finally forwards the stop; rclpy's default signal
    # handler would shut the context down before a cancel could be delivered.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(signum, frame):
        # Raising from a signal handler can interrupt a DDS message conversion.
        # Let the current callback finish before forwarding the stop below.
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = TrajectoryFollower()
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if rclpy.ok():
            node.shutdown_output()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
