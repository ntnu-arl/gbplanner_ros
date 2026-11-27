#!/usr/bin/env python3
"""ROS 2 adaptation of upstream PlannerControlInterface orchestration.

Preserves planning/execution separation, manual/auto triggering, explicit homing
and stop. Action transport and the bounded asynchronous state machine are new.
No motor controller or simulated vehicle is implemented in this node.
"""
import copy
import json
import math
import signal
import threading
import time
import traceback
from collections import deque
from dataclasses import asdict
from path_progress import PathProgress
from landing_policy import LandingPolicy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.duration import Duration
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from planner_msgs.action import PathFollower
from planner_msgs.srv import PlannerSrv, PlannerGlobal, PlannerHoming, PlannerSearch, PlannerRequestPath, PlannerGoToWaypoint, PlannerSetHomingPos, PciSearch, PciTrigger, PciGlobal, PciStop, PciHomingTrigger, PciSetHomingPos, PciToWaypoint, PciInitialization, ValidatePath,PlannerGeofence,PciGeofence,PlanAhead,ContinuePath

def position(pose):return (pose.position.x,pose.position.y,pose.position.z)

def valid_pose(pose):
    q=pose.orientation
    return all(math.isfinite(v) for v in (*position(pose),q.x,q.y,q.z,q.w)) and q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w>=1e-12

def unit_quaternion(q):
    values=(q.x,q.y,q.z,q.w);norm=math.hypot(*values)
    if not math.isfinite(norm) or norm<1e-12:raise ValueError('invalid_waypoint_quaternion')
    return tuple(value/norm for value in values)

def quaternion_product(a,b):
    ax,ay,az,aw=a;bx,by,bz,bw=b
    return (aw*bx+ax*bw+ay*bz-az*by,aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw,aw*bw-ax*bx-ay*by-az*bz)

def reconnect_path(path,current):
    """Same <=0.1 / <=1.5 m reconnect rule as the native PCI follower.

    The caller validates this complete modified path before sending the goal.
    The follower then validates its own fresh-odometry reconnection again.
    """
    route=copy.deepcopy(path);start=copy.deepcopy(current)
    distance=math.dist(position(start),position(route[0]))
    if distance<=.1:return [start]+route
    if distance>1.5 or len(route)<2:raise ValueError('path_start_too_far_for_reconnect')
    normal=[(a-b)/distance for a,b in zip(position(route[0]),position(start))]
    plane=sum(n*p for n,p in zip(normal,position(route[0])))
    side_start=sum(n*p for n,p in zip(normal,position(start)))-plane
    side_next=sum(n*p for n,p in zip(normal,position(route[1])))-plane
    if side_start*side_next>0:route.pop(0)
    return [start]+route

class PlannerControl(Node):
    TERMINAL={'IDLE','DONE','STOPPED','ERROR','PREVIEW'}
    def __init__(self):
        super().__init__('gbplanner_control')
        self.frame=self.declare_parameter('frame_id','map').value
        self.request_timeout=float(self.declare_parameter('request_timeout',10.0).value)
        self.execution_timeout=float(self.declare_parameter('execution_timeout',60.0).value)
        self.odom_timeout=float(self.declare_parameter('odom_timeout',2.0).value)
        self.endpoint_tolerance=float(self.declare_parameter('endpoint_tolerance',.1).value)
        self.waypoint_tf_timeout=float(self.declare_parameter('waypoint_tf_timeout',.2).value)
        self.robot_type=str(self.declare_parameter('robot_type','kAerial').value)
        # Defaults come from the upstream rmf_obelix / smb PCI profiles.
        self.init_z_takeoff=float(self.declare_parameter('init_motion.z_takeoff',2.5).value)
        self.init_z_drop=float(self.declare_parameter('init_motion.z_drop',1.7).value)
        self.init_x_forward=float(self.declare_parameter('init_motion.x_forward',3.0 if self.robot_type=='kGround' else 2.0).value)
        if self.robot_type not in {'kAerial','kGround'}:
            raise ValueError('robot_type must be kAerial or kGround')
        if any(not math.isfinite(v) or v<0. for v in (self.init_z_takeoff,self.init_z_drop,self.init_x_forward)):
            raise ValueError('Initialization distances must be finite and nonnegative')
        if (self.robot_type=='kGround' and self.init_x_forward==0.) or (self.robot_type=='kAerial' and self.init_z_takeoff+self.init_z_drop+self.init_x_forward==0.):
            raise ValueError('Initialization motion must contain a nonzero displacement')
        self.lead_time=float(self.declare_parameter('planner_trigger_lead_time',0.).value)
        if not math.isfinite(self.lead_time) or self.lead_time<0:
            raise ValueError('Invalid planner lead time')
        # Opt-in ROS 2 scheduling adaptation. ETA and measured latency use
        # the same trajectory clock; paused simulation time is not wall time.
        self.adaptive_lead=bool(self.declare_parameter('planner_trigger_adaptive_lead',False).value)
        # Opt-in pipeline: spend the current route's travel time on one next
        # endpoint plan instead of predicting a large graph from a small one.
        self.early_start=bool(self.declare_parameter('planner_trigger_early_start',False).value)
        self.planning_latencies=[];self.handoff_latencies=[];self.ahead_events=[]
        self.local_rpc_started=None;self.local_handoff_started=None
        self.max_cycles=int(self.declare_parameter('max_cycles',100).value)
        # ROS 2 adaptation, disabled unless a deployment explicitly opts in.
        # Only confirmed map-invalidated automatic local routes may recover.
        self.path_invalidation_retries=int(self.declare_parameter('path_invalidation_retries',0).value)
        if self.path_invalidation_retries<0:raise ValueError('Invalid path invalidation retry limit')
        self.recovery_pending=None;self.recovery_attempts=0;self.recovery_consecutive=0
        self.recovery_events=[];self.execution_outcomes={}
        self.require_mission=bool(self.declare_parameter('require_mission_status',False).value)
        self.ground_maneuver_enabled=bool(self.declare_parameter('ground_maneuver.enabled',False).value)
        self.ground_maneuver_parameters={name:float(self.declare_parameter('ground_maneuver.'+name,value).value)
            for name,value in {'control_offset':.2159,'max_speed':.3,'max_yaw_rate':.4,
                'max_acceleration':.3,'max_yaw_acceleration':.6,'max_duration':30.,
                'max_path_length':5.,'construction_wall_time':.1}.items()}
        if any(not math.isfinite(v) or v<=0 for v in self.ground_maneuver_parameters.values()):
            raise ValueError('Invalid ground maneuver parameters')
        self.ground_maneuver=None;self.maneuver_candidates=[];self.maneuver_report=None
        self.maneuver_candidate_obligations=[]
        self.maneuver_index=0;self.maneuver_home_cost=0.;self.maneuver_started_ros=None
        self.odom_motion=None;self.odom_stationary=deque(maxlen=64)
        if self.ground_maneuver_enabled:
            if self.robot_type!='kGround':raise ValueError('Ground maneuvers require kGround')
            # Optional execution adapter: the base planner remains independent
            # of this higher-level package unless the deployment opts in.
            import sys
            from pathlib import Path as FilePath
            from ament_index_python.packages import get_package_prefix
            adapter=FilePath(get_package_prefix('gbplanner_execution'))/'lib/gbplanner_execution'
            sys.path.insert(0,str(adapter))
            from ground_maneuver import Limits,construct_maneuvers
            from ground_maneuver_transport import candidate_to_message
            from ground_return_route import construct_return_route
            from ground_route_obligations import construct_route_obligations
            self.maneuver_limits_type=Limits;self.construct_maneuvers=construct_maneuvers
            self.encode_maneuver=candidate_to_message
            self.construct_return_route=construct_return_route
            self.construct_route_obligations=construct_route_obligations
        if not self.frame or any(not math.isfinite(v) or v<=0 for v in [self.request_timeout,self.execution_timeout,self.odom_timeout,self.endpoint_tolerance,self.waypoint_tf_timeout]) or self.max_cycles<1:
            raise ValueError('Invalid controller parameters')
        self.state='IDLE';self.reason='ready';self.mode='local';self.auto=False;self.preview=False
        self.requested_bound_mode=3;self.bound_mode=3
        self.manual_path=False;self.manual_task=None;self.manual_reply=None
        self.vel_max=0.;self.requested_vel_max=0.;self.configured_vel_max=0.
        # Upstream run_en_ is one pending trigger, not a queue. Keep its next
        # settings separate from the parameters owned by an active route.
        self.pending_trigger=None;self.trigger_sequence=0;self.applied_trigger=None
        # Upstream homing_request_ is another single slot, consumed before
        # run_en_ or automatic planning once PCI is ready to accept motion.
        self.pending_homing=None;self.homing_sequence=0;self.applied_homing=None
        self.pending_global=None;self.global_sequence=0;self.applied_global=None
        self.pending_search=None;self.search_sequence=0;self.applied_search=None
        self.pending_waypoint=None;self.waypoint_sequence=0;self.applied_waypoint=None
        self.waypoint_inbox=[];self.waypoint_inbox_lock=threading.Lock()
        # Upstream gate request/success flags survive resetPlanner. Gate
        # service callbacks only enqueue; the default group owns both flags.
        self.pending_gate=None;self.gate_sequence=0;self.applied_gate=None
        self.passing_gate_success=False
        self.gate_inbox=[];self.gate_inbox_lock=threading.Lock()
        # Search services may await a preview or an Action ACK. Keep their
        # reentrant callbacks out of the task state; tick owns this single slot.
        self.search_inbox=[];self.search_inbox_lock=threading.Lock()
        # Upstream init_request_ is one pending flag. It runs after homing
        # and before ordinary planning, once the previous motion is complete.
        self.pending_initialization=None;self.initialization_sequence=0;self.applied_initialization=None
        self.initialization_resume=None
        # The service awaits an idle Action ACK in a reentrant group. Only
        # this inbox crosses threads; tick owns acceptance and consumption.
        self.initialization_inbox=[];self.initialization_inbox_lock=threading.Lock()
        self.epoch=0;self.cycles=0;self.completed=0;self.map_revision=0;self.completed_revision=0
        self.known_cells=0;self.map_stamp=0.;self.completed_stamp=0.
        self.observation_revision=0;self.completed_observation_revision=0
        self.odom=None;self.odom_received=0;self.pending=None;self.pending_goal=False;self.goal=None
        self.pending_goal_epoch=None
        self.odom_ros_received=0.;self.odom_stamp=0.
        self.odom_clock_high_water=None;self.odom_clock_invalidated=False
        self.odom_diagnostics={'accepted':0,'rejected':0,'rejections':{},'out_of_order':0,
            'last_received':None,'last_accepted':None,'last_rejected':None,'rejection_events':[],
            'max_accepted_receipt_gap_s':0.,'max_accepted_source_gap_s':0.,'max_accepted_source_age_s':0.}
        self.readiness_failure=None
        self.target=None;self.deadline=0;self.after_stop='STOPPED';self.stop_reason='stopped'
        self.retry_bound_mode=None
        self.events=[];self.feedback={};self.requested=[]
        self.progress=None
        self.ahead=None;self.ahead_schedule=None;self.route_generation=0;self.route_token=0
        self.ahead_home_required=False;self.ahead_global_required=False;self.ahead_count=0;self.plan_observation=0
        self.feedback_received=0.;self.execution_route=None;self.deferred_result=None;self.follower_tracks_route=False
        self.explicit_global=None
        self.search_source=None;self.search_target=None;self.search_task=None;self.search_reply=None
        self.navigation_goal=None;self.navigation_goal_error='waypoint_not_set'
        self.waypoint_task=None;self.waypoint_reply=None;self.home_call=None;self.last_frontier_parameters=None
        self.gate_task=None;self.gate_reply=None
        self.mission=None;self.mission_received=0.;self.mission_ros_received=0.
        # Mission status carries a ROS-time source stamp. Keep receipt and
        # source ages separate: receipt uses the local steady clock in a live
        # system, while source age uses the shared ROS clock. In simulation
        # both are ROS-time ages so a paused /clock does not expire a healthy
        # mission heartbeat. Legacy publishers without stamp_s fall back to
        # receipt age and expose an explicit diagnostic in readiness state.
        self.mission_source_stamp=None;self.mission_source_valid=False
        self.mission_source_diagnostic='missing_stamp_s'
        self.mission_source_rejections=0
        self.mission_ros_clock_high_water=None;self.mission_ros_clock_invalidated=False
        self.landing=LandingPolicy(self.robot_type,self.request_timeout)
        self.landing_future=None;self.execution_health=None
        self.commanded=[];self.validated_revision=0;self.validation_pending=False;self.validation_deadline=0.;self.validation_future=None
        self.pre_execution_path=[];self.pre_execution_observation=False
        self.pre_execution_deadline=0.;self.pre_execution_rechecks=0
        self.tf_buffer=Buffer()
        # TransformListener owns a separate reentrant callback group. The
        # second executor thread can deliver TF while a waypoint lookup waits.
        self.tf_listener=TransformListener(self.tf_buffer,self)
        durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub=self.create_publisher(String,'control_status',durable)
        self.path_pub=self.create_publisher(Path,'command_path',durable)
        self.stop_request_pub=self.create_publisher(Bool,'planner_control_interface/stop_request',10)
        self.create_subscription(Odometry,'odometry',self.receive_odometry,10)
        self.create_subscription(String,'map_updates',self.receive_map,durable)
        self.create_subscription(String,'mission_status',self.receive_mission,durable)
        self.create_subscription(String,'execution_status',self.receive_execution_route,durable)
        self.create_subscription(PoseStamped,'search_source',lambda msg:self.receive_search_pose(msg,'source'),durable)
        self.create_subscription(PoseStamped,'search_target',lambda msg:self.receive_search_pose(msg,'target'),durable)
        self.create_subscription(PoseStamped,'/move_base_simple/goal',self.receive_waypoint,10)
        self.create_subscription(PoseStamped,'/global_planner/waypoint_request',self.receive_waypoint,10)
        self.local=self.create_client(PlannerSrv,'planner')
        self.ahead_planner=self.create_client(PlanAhead,'planner_ahead')
        self.continue_client=self.create_client(ContinuePath,'continue_path')
        self.global_plan=self.create_client(PlannerGlobal,'planner_global')
        self.homing=self.create_client(PlannerHoming,'exploration_homing')
        self.landing_service=self.create_client(Empty,'land_srv')
        self.search_group=ReentrantCallbackGroup()
        self.search_planner=self.create_client(PlannerSearch,'planner_search',callback_group=self.search_group)
        self.create_service(PciSearch,'pci_search',self.trigger_search,callback_group=self.search_group)
        self.command_group=ReentrantCallbackGroup()
        self.passing_gate_planner=self.create_client(PlannerRequestPath,'gbplanner/passing_gate',callback_group=self.command_group)
        self.waypoint_planner=self.create_client(PlannerGoToWaypoint,'planner_go_to_waypoint',callback_group=self.command_group)
        self.home_setter=self.create_client(PlannerSetHomingPos,'gbplanner/set_homing_pos',callback_group=self.command_group)
        self.create_service(PciSetHomingPos,'pci_set_homing_pos',self.set_home,callback_group=self.command_group)
        self.create_service(PciToWaypoint,'pci_to_waypoint',self.direct_waypoint,callback_group=self.command_group)
        self.create_service(PciInitialization,'pci_initialization_trigger',self.initialize_motion,callback_group=self.command_group)
        self.create_service(Trigger,'pci_rotate_180_trigger',self.rotate_180,callback_group=self.command_group)
        prefix='planner_control_interface/std_srvs/'
        self.create_service(Trigger,prefix+'set_homing_position_here',self.std_set_home,callback_group=self.command_group)
        self.create_service(Trigger,prefix+'go_to_waypoint',self.std_waypoint,callback_group=self.command_group)
        self.create_service(Trigger,prefix+'pass_gate',self.std_pass_gate,callback_group=self.command_group)
        self.create_service(Trigger,prefix+'preview_pass_gate',self.std_preview_gate,callback_group=self.command_group)
        self.create_service(Trigger,prefix+'automatic_planning',self.std_automatic)
        self.create_service(Trigger,prefix+'single_planning',self.std_single)
        self.create_service(Trigger,prefix+'homing_trigger',self.std_homing)
        self.create_service(Trigger,prefix+'stop',self.std_stop)
        self.create_service(Trigger,prefix+'replan_last_specified_frontier',self.std_replan)
        self.validate=self.create_client(ValidatePath,'validate_path')
        self.geofence_group=ReentrantCallbackGroup();self.geofence_calls=[]
        self.geofence_client=self.create_client(PlannerGeofence,'gbplanner/geofence',callback_group=self.geofence_group)
        self.create_service(PciGeofence,'pci_geofence',self.add_geofence,callback_group=self.geofence_group)
        self.follower=ActionClient(self,PathFollower,'path_follower')
        self.create_service(PciTrigger,'pci_trigger',self.trigger)
        self.create_service(PciTrigger,'planner_control_interface_trigger',self.trigger)
        self.create_service(PciGlobal,'pci_global',self.trigger_global)
        self.create_service(PciHomingTrigger,'pci_homing_trigger',self.trigger_homing)
        self.create_service(PciStop,'pci_stop',self.stop)
        self.create_service(Trigger,'control_state',self.get_status)
        # Service/action and stop deadlines remain live while /clock is paused.
        self.deadline_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05,self.tick,clock=self.deadline_clock);self.transition('IDLE','ready')

    async def add_geofence(self,request,response):
        # Independent callback group lets path validation and stop processing
        # continue while the planner handles this service. Preserve the frame.
        response.success=False
        if not self.geofence_client.service_is_ready() or len(self.geofence_calls)>=16:return response
        forwarded=PlannerGeofence.Request();forwarded.header=request.header;forwarded.rectangles=request.rectangles
        pending=self.geofence_client.call_async(forwarded);finished=Future()
        entry=(pending,finished,time.monotonic()+self.request_timeout)
        self.geofence_calls.append(entry)
        def complete(result):
            if finished.done():return
            try:finished.set_result(bool(result.result().success))
            except Exception:finished.set_result(False)
        pending.add_done_callback(complete)
        response.success=await finished
        if entry in self.geofence_calls:self.geofence_calls.remove(entry)
        return response

    def receive_odometry(self,message):
        received,now,simulated,clock_regressed=self.odometry_clock()
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        diagnostics=self.odom_diagnostics;previous=diagnostics['last_accepted']
        event={'header_stamp_s':stamp,'callback_ros_s':now,'callback_monotonic_s':received,
               'freshness_clock':'ros' if simulated else 'steady',
               'source_age_s':now-stamp,
               'source_stamp_gap_s':stamp-previous['header_stamp_s'] if previous else None,
               'accepted_receipt_gap_s':received-self.odom_received if previous else None,
               'accepted_receipt_ros_gap_s':now-self.odom_ros_received if previous else None}
        diagnostics['last_received']=event
        p=message.pose.pose;q=p.orientation
        # Receipt freshness plus source timestamp freshness: a replayed old pose
        # must not become current merely because it was just received.
        reason=('frame_mismatch' if message.header.frame_id!=self.frame else
                'nonfinite_pose' if not all(math.isfinite(v) for v in (*position(p),q.x,q.y,q.z,q.w)) else
                'invalid_quaternion' if q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<1e-12 else
                'nonpositive_stamp' if stamp<=0 else
                'ros_clock_regressed' if clock_regressed else
                'source_stale' if now-stamp>self.odom_timeout else
                'source_future' if stamp-now>.1 else
                'repeated_stamp' if stamp==self.odom_stamp else
                'out_of_order_stamp' if stamp<self.odom_stamp else None)
        if reason is not None:
            event=dict(event,reason=reason);diagnostics['last_received']=event
            diagnostics['rejected']+=1
            diagnostics['rejections'][reason]=diagnostics['rejections'].get(reason,0)+1
            diagnostics['last_rejected']=event
            diagnostics['rejection_events']=(diagnostics['rejection_events']+[event])[-8:]
            diagnostics['out_of_order']+=reason=='out_of_order_stamp'
            # Replayed samples neither replace the accepted pose nor renew its
            # receipt time; invalid source data still clears the current pose.
            if reason not in {'repeated_stamp','out_of_order_stamp'}:self.odom=None
            return
        diagnostics['accepted']+=1
        if previous:
            diagnostics['max_accepted_receipt_gap_s']=max(diagnostics['max_accepted_receipt_gap_s'],event['accepted_receipt_gap_s'])
            diagnostics['max_accepted_source_gap_s']=max(diagnostics['max_accepted_source_gap_s'],event['source_stamp_gap_s'])
        diagnostics['max_accepted_source_age_s']=max(diagnostics['max_accepted_source_age_s'],event['source_age_s'])
        diagnostics['last_accepted']=event
        self.odom=p;self.odom_stamp=stamp;self.odom_received=received;self.odom_ros_received=now
        linear=message.twist.twist.linear;angular=message.twist.twist.angular
        motion=(math.hypot(linear.x,linear.y,linear.z),abs(angular.z))
        self.odom_motion=motion if all(math.isfinite(v) for v in motion) else None
        self.odom_stationary.append((stamp,self.odom_motion))
        self.odom_clock_invalidated=False
        if self.progress is not None and self.state in {'SENDING','EXECUTING'} and not self.follower_tracks_route:
            try:self.progress.update(position(p))
            except ValueError as error:self.fail(str(error))

    def committing(self):
        return self.ahead is not None and self.ahead['stage']=='committing'

    def receive_execution_route(self,message):
        try:
            data=json.loads(message.data)
            if (isinstance(data.get('active'),bool) and
                    isinstance(data.get('output_in_flight'),bool) and
                    isinstance(data.get('output_unconfirmed'),bool)):
                self.execution_health=data
            outcome=data.get('terminal_outcome')
            if isinstance(outcome,dict):
                goal_id=outcome.get('goal_id');generation=outcome.get('generation')
                if (isinstance(goal_id,list) and len(goal_id)==16 and
                        all(type(v) is int and 0<=v<256 for v in goal_id) and
                        type(generation) is int and generation>=0):
                    # Status and Action results may arrive in either order;
                    # continuation ACKs may also lag the final generation.
                    self.execution_outcomes[(tuple(goal_id),generation)]=copy.deepcopy(outcome)
                    while len(self.execution_outcomes)>16:
                        self.execution_outcomes.pop(next(iter(self.execution_outcomes)))
                    self.confirm_recovery()
            if (data.get('state')!='EXECUTING' or not data.get('active') or
                    len(data.get('goal_id') or [])!=16 or len(data.get('endpoint') or [])!=4 or
                    not all(math.isfinite(v) for v in data['endpoint'])):return
            self.execution_route=data
            self.transfer_path_validation()
        except (ValueError,TypeError,KeyError):return

    def transfer_path_validation(self):
        route=self.execution_route
        if (self.manual_path or self.follower_tracks_route or self.state!='EXECUTING' or
                self.goal is None or route is None or route.get('state')!='EXECUTING' or
                route.get('active') is not True or route.get('validates_remaining_path') is not True or
                type(route.get('validated_revision')) is not int or route['validated_revision']<1 or
                route.get('goal_id')!=list(self.goal.goal_id.uuid) or
                type(route.get('generation')) is not int or route['generation']!=self.route_generation):return
        # The native follower validated its own latest-odometry reconnection.
        # Keeping a second cursor on the earlier preflight geometry can reject
        # valid motion. Retire only checks for this accepted action/generation;
        # the action result remains the sole task-completion signal.
        self.progress=None;self.follower_tracks_route=True;self.route_token+=1
        self.validation_pending=False;self.validation_future=None
        self.validated_revision=route['validated_revision']
        message=String();message.data=json.dumps(self.payload());self.status_pub.publish(message)

    def receive_map(self,message):
        try:
            update=json.loads(message.data);revision=int(update['map_revision']);known=int(update['known_cells'])
            stamp=int(update['stamp_sec'])+int(update['stamp_nanosec'])*1e-9
            observation=int(update.get('observation_revision',revision))
            if revision<1 or known<0 or stamp<0 or observation<0 or observation>revision:return
        except (ValueError,TypeError,KeyError):return
        reset=known==0 or revision<self.map_revision
        self.map_revision=revision;self.known_cells=known;self.map_stamp=stamp
        self.observation_revision=observation
        if reset:
            self.ahead_global_required=False;self.pending_trigger=None;self.pending_homing=None;self.pending_global=None
            self.pending_initialization=None;self.initialization_resume=None
            self.reject_initialization_inbox()
            self.clear_search_queue()
            self.clear_waypoint_queue()
            self.reject_gate_inbox('map_reset')
        # A READY gate slot must be suspended too. Otherwise the next tick
        # would consume it against the reset map and discard the retained flag.
        if (reset and self.state!='STOPPING' and
                (self.state not in self.TERMINAL or
                 (self.pending_gate is not None and self.state in {'IDLE','DONE','PREVIEW'}))):
            self.fail('map_reset_during_task')

    def receive_mission(self,message):
        try:
            status=json.loads(message.data)
            if not isinstance(status['enabled'],bool) or not isinstance(status['valid'],bool):return
            status['remaining_s']=float(status['remaining_s'])
            if not math.isfinite(status['remaining_s']):return
            now_ros=self.get_clock().now().nanoseconds*1e-9
            if not self.observe_mission_clock(now_ros):
                self.mission_source_rejections+=1
                self.mission_source_diagnostic='ros_clock_regressed'
                return
            source_stamp=None;source_valid=False;diagnostic='legacy_missing_stamp_s'
            previous=self.mission_source_stamp
            if 'stamp_s' in status:
                try:
                    raw_stamp=status['stamp_s']
                    if isinstance(raw_stamp,bool):raise ValueError('boolean_stamp')
                    source_stamp=float(raw_stamp)
                    if not math.isfinite(source_stamp) or source_stamp<0.:raise ValueError('invalid_stamp')
                except (ValueError,TypeError,OverflowError):
                    self.mission_source_rejections+=1
                    self.mission_source_diagnostic='invalid_stamp_s'
                    return
                if source_stamp-now_ros>.1:
                    self.mission_source_rejections+=1
                    self.mission_source_diagnostic='future_stamp_s'
                    return
                if previous is not None and source_stamp<=previous:
                    self.mission_source_rejections+=1
                    self.mission_source_diagnostic=('duplicate_stamp_s' if source_stamp==previous else 'out_of_order_stamp_s')
                    return
                # Repeated stamps during a paused simulation leave both the
                # stored budget and receipt unchanged. They cannot revive an
                # older, larger budget; freshness still follows paused ROS time.
                source_valid=True;diagnostic='ok'
            elif previous is not None:
                self.mission_source_rejections+=1
                self.mission_source_diagnostic='missing_stamp_s_after_stamped'
                return
            self.mission=status;self.mission_received=time.monotonic()
            self.mission_ros_received=now_ros
            self.mission_source_stamp=source_stamp
            self.mission_source_valid=source_valid
            self.mission_source_diagnostic=diagnostic
            self.mission_ros_clock_invalidated=False
        except (ValueError,TypeError,KeyError,OverflowError):return
        if self.landing.observe(status,self.mission_received):
            self.landing.expect_cancellation(self.goal is not None or self.pending_goal)
            deferred=self.deferred_result
            self.mode='landing'
            self.cancel('auto_landing_due','STOPPED')
            if deferred is not None:
                try:confirmed=deferred[0].result().status in {GoalStatus.STATUS_CANCELED,GoalStatus.STATUS_SUCCEEDED}
                except Exception:confirmed=False
                self.landing.motion_result(confirmed)
            self.publish_landing()
        if self.advance_landing():return
        if self.mission_error() and self.state not in self.TERMINAL and self.state!='STOPPING':
            self.fail(self.mission_error())

    def advance_landing(self):
        if not self.landing.latched:return False
        previous=self.landing.payload()
        health=self.execution_health
        # The follower's terminal state confirms it has relinquished output;
        # neither an Action cancel ACK nor an Empty response proves this.
        motion_clear=(self.goal is None and not self.pending_goal and health is not None
                      and not any(health.get(key,True) for key in
                                  ('active','output_in_flight','output_unconfirmed'))
                      and not str(health.get('reason','')).endswith(
                          (':hold_unconfirmed',':external_stop_unconfirmed')))
        fresh=self.odometry_freshness()['fresh']
        send=self.landing.advance(time.monotonic(),motion_clear,fresh,self.landing_service.service_is_ready())
        if send:
            try:
                self.landing_future=self.landing_service.call_async(Empty.Request())
                self.landing_future.add_done_callback(self.landing_response)
            except Exception:
                self.landing.service_result(False,time.monotonic())
        if previous!=self.landing.payload():self.publish_landing()
        return True

    def publish_landing(self):
        phase=self.landing.phase
        state='ERROR' if phase in {'FAILED','UNCONFIRMED'} else 'STOPPING' if phase=='QUIESCING' else 'STOPPED'
        self.transition(state,self.landing.reason)

    def landing_response(self,future):
        if future is not self.landing_future:return
        self.landing_future=None
        try:received=future.result() is not None
        except Exception:received=False
        self.landing.service_result(received,time.monotonic())
        self.publish_landing()

    def landing_motion_finished(self,confirmed):
        self.landing.motion_result(confirmed)
        self.publish_landing()
        self.advance_landing()

    def observe_mission_clock(self,now_ros):
        if self.mission_ros_clock_high_water is not None and now_ros<self.mission_ros_clock_high_water:
            self.mission_ros_clock_invalidated=True
            return False
        self.mission_ros_clock_high_water=now_ros
        # Merely catching up does not restore an old snapshot. Only an
        # accepted heartbeat after catch-up clears the invalidation latch.
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
        # Preserve signed diagnostic ages. Only a fresh snapshot may supply a
        # budget; its nonnegative receipt age bounds a tolerated future source.
        age=receipt_age if source_age is None else max(receipt_age,source_age)
        return {'fresh':fresh,'age_s':age,'receipt_steady_age_s':receipt_steady,
                'receipt_ros_age_s':receipt_ros,'source_age_s':source_age,
                'age_clock':'ros' if simulated else 'steady+ros',
                'clock_regressed':clock_regressed,
                'source_valid':self.mission_source_valid,
                'source_diagnostic':self.mission_source_diagnostic}

    def mission_error(self,now_steady=None,now_ros=None):
        if self.landing.latched:return 'auto_landing_engaged'
        if self.mission is None:return 'mission_status_missing' if self.require_mission else None
        if not self.mission['enabled']:return 'mission_budget_disabled' if self.require_mission else None
        freshness=self.mission_freshness(now_steady,now_ros)
        if freshness is not None and freshness['clock_regressed']:return 'mission_status_clock_regressed'
        if freshness is None or not freshness['fresh']:return 'mission_status_stale'
        if not self.mission['valid']:return self.mission['reason']
        if self.mission['remaining_s']-max(0.,freshness['age_s'])<=0:return 'mission_budget_exhausted'
        return None

    def odometry_clock(self):
        received=time.monotonic();now=self.get_clock().now().nanoseconds*1e-9
        simulated=bool(self.get_parameter('use_sim_time').value)
        regressed=(simulated and self.odom_clock_high_water is not None and now<self.odom_clock_high_water)
        if simulated:
            if regressed:self.odom_clock_invalidated=True
            else:self.odom_clock_high_water=now
        return received,now,simulated,regressed

    def odometry_freshness(self):
        received,now,simulated,_=self.odometry_clock()
        present=self.odom is not None;accepted=self.odom_stamp>0.
        steady_age=received-self.odom_received if accepted else None
        ros_age=now-self.odom_ros_received if accepted else None
        source_age=now-self.odom_stamp if accepted else None
        receipt_age=ros_age if simulated else steady_age
        # Slow or paused simulation must not age a pose in simulated seconds.
        # A clock rewind requires a new increasing sample after time catches up.
        fresh=(present and not self.odom_clock_invalidated and
               0.<=receipt_age<=self.odom_timeout and -.1<=source_age<=self.odom_timeout)
        return {'fresh':fresh,'odometry_present':present,'use_sim_time':simulated,
                'freshness_clock':'ros' if simulated else 'steady','deadline_clock':'steady',
                'receipt_steady_age_s':steady_age,'receipt_ros_age_s':ros_age,'source_age_s':source_age,
                'clock_regressed':self.odom_clock_invalidated,'odometry_timeout_s':self.odom_timeout,
                'last_accepted_source_stamp_s':self.odom_stamp if accepted else None,
                'tick_monotonic_s':received,'tick_ros_s':now}

    def ready(self):
        return self.odometry_freshness()['fresh'] and self.known_cells>0 and self.mission_error() is None

    def readiness_snapshot(self):
        snapshot=self.odometry_freshness()
        mission_freshness=self.mission_freshness(snapshot['tick_monotonic_s'],snapshot['tick_ros_s'])
        return dict(snapshot,receipt_age_s=snapshot['receipt_steady_age_s'],receipt_age_clock='steady',
                    known_cells=self.known_cells,mission_error=self.mission_error(snapshot['tick_monotonic_s'],snapshot['tick_ros_s']),
                    mission_age_s=(mission_freshness['age_s'] if mission_freshness else None),
                    mission_age_clock=(mission_freshness['age_clock'] if mission_freshness else None),
                    mission_receipt_steady_age_s=(mission_freshness['receipt_steady_age_s'] if mission_freshness else None),
                    mission_receipt_ros_age_s=(mission_freshness['receipt_ros_age_s'] if mission_freshness else None),
                    mission_source_age_s=(mission_freshness['source_age_s'] if mission_freshness else None),
                    mission_source_valid=(mission_freshness['source_valid'] if mission_freshness else False),
                    mission_source_diagnostic=(mission_freshness['source_diagnostic'] if mission_freshness else None),
                    mission_clock_regressed=(mission_freshness['clock_regressed'] if mission_freshness else False),
                    mission_source_rejections=self.mission_source_rejections,
                    odometry=copy.deepcopy(self.odom_diagnostics))

    def receive_search_pose(self,message,role):
        pose=copy.deepcopy(message.pose) if message.header.frame_id==self.frame and valid_pose(message.pose) else None
        if role=='source':self.search_source=pose
        else:self.search_target=pose

    def search_ready(self):
        # A hypothetical preview only needs its frozen poses and an observed
        # map. Execution and current-state requests also require fresh odometry.
        hypothetical=self.preview and self.search_task is not None and not self.search_task['use_current_state']
        return self.known_cells>0 and self.mission_error() is None and (hypothetical or self.ready())

    def search_status(self):
        if self.search_task is None:return None
        task=self.search_task
        return {'use_current_state':task['use_current_state'],'preview':self.preview,
                'requested_source':list(position(task['source'])),'requested_target':list(position(task['target'])),
                'endpoint':task.get('endpoint'),'endpoint_distance':task.get('endpoint_distance'),
                'observed_target_distance':task.get('observed_target_distance'),
                'exact_target':task.get('exact_target',False),'path_points':len(task.get('planned_path',[])),
                'command_points':len(self.commanded)}

    def complete_search(self,success):
        pending=self.search_reply
        if pending is None:return
        self.search_reply=None
        path=copy.deepcopy(self.search_task.get('planned_path',[])) if success and self.search_task else []
        if not pending['future'].done():pending['future'].set_result((bool(success and path),path))

    async def trigger_search(self,request,response):
        finished=Future()
        with self.search_inbox_lock:self.search_inbox.append((copy.deepcopy(request),finished))
        response.success,response.path=await finished
        return response

    def take_search_inbox(self):
        with self.search_inbox_lock:
            requests=self.search_inbox;self.search_inbox=[]
        return requests

    def clear_search_queue(self):
        self.pending_search=None
        for _,finished in self.take_search_inbox():
            if not finished.done():finished.set_result((False,[]))

    @staticmethod
    def search_request_status(request):
        if request is None:return None
        task=request['parameters']
        return {'sequence':request['sequence'],'preview':request['preview'],
                'bound_mode':request['bound_mode'],'use_current_state':task['use_current_state'],
                'source':list(position(task['source'])) if task['source'] is not None else None,
                'target':list(position(task['target']))}

    def search_request_error(self,task,preview,bound_mode):
        if bound_mode not in range(5):return 'invalid_bound_mode'
        if self.known_cells<=0 or self.mission_error() is not None:return 'map_or_mission_not_ready'
        if task['target'] is None or (not task['use_current_state'] and task['source'] is None):return 'search_pose_missing'
        if (task['use_current_state'] or not preview) and not self.ready():return 'fresh_odometry_missing'
        if not preview and not self.follower.server_is_ready():return 'executor_unavailable'
        if not self.search_planner.service_is_ready():return 'search_service_unavailable'
        if not self.validate.service_is_ready():return 'path_validation_unavailable'
        return None

    def receive_search_requests(self):
        """Accept search_request_ in the default group, without touching motion."""
        for request,finished in self.take_search_inbox():
            if finished.done():continue
            task={'source':None if request.use_current_state else copy.deepcopy(self.search_source),
                  'target':copy.deepcopy(self.search_target),'use_current_state':bool(request.use_current_state)}
            if (request.header.frame_id!=self.frame or self.landing.latched or self.home_call is not None or
                    self.state=='STOPPING' or self.search_request_error(task,request.not_exe_path,request.bound_mode)):
                finished.set_result((False,[]));continue
            if self.idle():
                if task['use_current_state']:task['source']=copy.deepcopy(self.odom)
                self.search_reply={'future':finished,'deadline':time.monotonic()+3.*self.request_timeout}
                self.begin('search',False,request.not_exe_path,search_parameters=task,bound_mode=request.bound_mode)
                continue
            if self.state not in {'PLANNING','VALIDATING','SENDING','EXECUTING',
                                  'WAITING_REPLAN','WAITING_OBSERVATION','PREVIEW','DONE'}:
                finished.set_result((False,[]));continue
            self.search_sequence+=1
            self.pending_search={'sequence':self.search_sequence,'preview':bool(request.not_exe_path),
                                 'bound_mode':int(request.bound_mode),'parameters':task}
            self.transition(self.state,'search_trigger_queued')
            # A busy request acknowledges queue acceptance, not a planned path
            # or an execution result. The latest unconsumed request wins.
            finished.set_result((True,[]))

    def consume_search(self):
        """READY priority: homing, initialization, auto/local, search, global."""
        if (self.pending_search is None or self.pending_homing is not None or self.pending_initialization is not None or
                self.pending_trigger is not None or self.auto or self.landing.latched or self.home_call is not None or
                self.state not in {'PREVIEW','DONE'} or self.goal is not None or self.pending_goal or self.committing()):return False
        request=self.pending_search;self.pending_search=None
        task=copy.deepcopy(request['parameters'])
        reason=self.search_request_error(task,request['preview'],request['bound_mode'])
        if reason:self.fail('queued_search_'+reason);return True
        # Freeze current-state at consumption, after the previous route has
        # arrived. A queued request must not reconnect from its old start pose.
        if task['use_current_state']:task['source']=copy.deepcopy(self.odom)
        self.applied_search=self.search_request_status(dict(request,parameters=task))
        self.applied_search.update(completed_actions=self.completed,cycles=self.cycles)
        # Do not begin(): it clears lower-priority queued work and task counts.
        self.epoch+=1;self.route_token+=1
        self.auto=False;self.preview=request['preview']
        self.requested_bound_mode=self.bound_mode=request['bound_mode']
        self.requested_vel_max=0.;self.vel_max=self.configured_vel_max
        self.manual_path=False;self.manual_task=None;self.explicit_global=None
        self.initialization_resume=None
        self.search_task=task;self.waypoint_task=None;self.gate_task=None
        self.retry_bound_mode=None;self.readiness_failure=None
        self.ahead=None;self.ahead_schedule=None;self.ahead_home_required=False;self.ahead_global_required=False
        self.feedback={};self.feedback_received=0.;self.commanded=[];self.progress=None
        self.validation_pending=False;self.validation_future=None
        self.request_plan('search');return True

    def waypoint_in_frame(self,message):
        if not message.header.frame_id or not valid_pose(message.pose):
            raise ValueError('waypoint_frame_or_pose_invalid')
        stamp=message.header.stamp
        if stamp.sec<0 or not 0<=stamp.nanosec<1000000000:
            raise ValueError('waypoint_stamp_invalid')
        result=copy.deepcopy(message);q=unit_quaternion(result.pose.orientation)
        if message.header.frame_id!=self.frame:
            # Nonzero stamps select the measured transform. An unstamped UI
            # click retains the original PCI's latest-transform semantics.
            when=Time.from_msg(stamp) if stamp.sec or stamp.nanosec else Time()
            transform=self.tf_buffer.lookup_transform(self.frame,message.header.frame_id,when,
                                                       timeout=Duration(seconds=self.waypoint_tf_timeout))
            rotation=unit_quaternion(transform.transform.rotation)
            translation=transform.transform.translation
            if not all(math.isfinite(v) for v in (translation.x,translation.y,translation.z)):
                raise ValueError('waypoint_transform_invalid')
            rotated=quaternion_product(quaternion_product(rotation,(*position(message.pose),0.)),
                                       (-rotation[0],-rotation[1],-rotation[2],rotation[3]))
            result.pose.position.x=rotated[0]+translation.x
            result.pose.position.y=rotated[1]+translation.y
            result.pose.position.z=rotated[2]+translation.z
            q=quaternion_product(rotation,q)
            if not stamp.sec and not stamp.nanosec:result.header.stamp=transform.header.stamp
        result.pose.orientation.x,result.pose.orientation.y,result.pose.orientation.z,result.pose.orientation.w=q
        result.header.frame_id=self.frame
        if not valid_pose(result.pose):raise ValueError('transformed_waypoint_invalid')
        return result

    def receive_waypoint(self,message):
        try:self.navigation_goal=self.waypoint_in_frame(message);self.navigation_goal_error=None
        except (TransformException,ValueError) as error:
            self.navigation_goal=None;self.navigation_goal_error='waypoint_transform_failed:'+str(error)

    def waypoint_status(self):
        task=self.waypoint_task
        return {'selected':list(position(self.navigation_goal.pose)) if self.navigation_goal else None,
                'selection_error':self.navigation_goal_error,
                'branch':task['branch'] if task else None,
                'requested_target':list(position(task['target'].pose)) if task else None,
                'endpoint':task.get('endpoint') if task else None,
                'endpoint_distance':task.get('endpoint_distance') if task else None,
                'observed_target_distance':task.get('observed_target_distance') if task else None,
                'exact_target':task.get('exact_target',False) if task else False}

    def complete_waypoint(self,success,reason=None):
        pending=self.waypoint_reply
        if pending is None:return
        self.waypoint_reply=None
        if not pending['future'].done():
            pending['future'].set_result((bool(success),'waypoint_execution_started' if success else reason or self.reason))

    async def start_waypoint(self,target,direct):
        # Reentrant services submit only immutable request data. Tick owns
        # admission, target freezing and the single pending waypoint slot.
        finished=Future()
        with self.waypoint_inbox_lock:
            self.waypoint_inbox.append((copy.deepcopy(target),bool(direct),finished))
        return await finished

    async def direct_waypoint(self,request,response):
        target=PoseStamped();target.header=request.header;target.pose=request.waypoint
        # The upstream response has no fields. Outcome is therefore reported
        # through control_status, never invented as a success response field.
        await self.start_waypoint(target,True)
        return response

    async def std_waypoint(self,request,response):
        response.success,response.message=await self.start_waypoint(None,False)
        return response

    def take_waypoint_inbox(self):
        with self.waypoint_inbox_lock:
            requests=self.waypoint_inbox;self.waypoint_inbox=[]
        return requests

    def clear_waypoint_queue(self):
        self.pending_waypoint=None
        for _,_,finished in self.take_waypoint_inbox():
            if not finished.done():finished.set_result((False,'waypoint_request_cleared'))

    @staticmethod
    def waypoint_request_status(request):
        if request is None:return None
        task=request['parameters'];target=task['target'];q=target.pose.orientation
        return {'sequence':request['sequence'],'branch':task['branch'],'bound_mode':request['bound_mode'],
                'target':list(position(target.pose)),'orientation':[q.x,q.y,q.z,q.w],
                'frame_id':target.header.frame_id,
                'stamp_ns':target.header.stamp.sec*1000000000+target.header.stamp.nanosec}

    def waypoint_request_error(self,task):
        if not self.ready():return 'map_or_fresh_odometry_missing'
        if not self.validate.service_is_ready():return 'path_validation_unavailable'
        if not self.follower.server_is_ready():return 'executor_unavailable'
        if task['branch']=='global' and not self.waypoint_planner.service_is_ready():return 'waypoint_service_unavailable'
        return None

    def receive_waypoint_requests(self):
        """Freeze the accepted world target; retain only the latest busy request."""
        for target,direct,finished in self.take_waypoint_inbox():
            if finished.done():continue
            target=target if direct else copy.deepcopy(self.navigation_goal)
            error=None
            if target is None:error=self.navigation_goal_error or 'waypoint_not_set'
            else:
                try:target=self.waypoint_in_frame(target)
                except (TransformException,ValueError) as exception:
                    error='waypoint_transform_failed:'+str(exception)
            task={'target':target,'source':None,'branch':'direct' if direct else 'global'}
            # The TF lookup can wait; inspect current ownership only after it
            # returns. Never overwrite a task or start motion in that lookup.
            if self.landing.latched or self.home_call is not None or self.state=='STOPPING':
                error=error or 'controller_unavailable'
            error=error or self.waypoint_request_error(task)
            if error:
                if direct and self.idle() and self.state not in {'ERROR','STOPPED'}:
                    self.transition('ERROR','direct_waypoint_request_rejected')
                finished.set_result((False,error));continue
            mode=int(self.bound_mode);task['bound_mode']=mode
            if self.idle():
                task['source']=copy.deepcopy(self.odom)
                self.waypoint_reply={'future':finished,'deadline':time.monotonic()+3.*self.request_timeout}
                self.begin('waypoint_direct' if direct else 'waypoint',False,False,
                           waypoint_parameters=task,bound_mode=mode)
                continue
            if self.state not in {'PLANNING','VALIDATING','SENDING','EXECUTING',
                                  'WAITING_REPLAN','WAITING_OBSERVATION','PREVIEW','DONE'}:
                finished.set_result((False,'controller_unavailable'));continue
            self.waypoint_sequence+=1
            self.pending_waypoint={'sequence':self.waypoint_sequence,'bound_mode':mode,'parameters':task}
            self.transition(self.state,'waypoint_trigger_queued')
            # This acknowledges the pending slot, not an Action acceptance.
            finished.set_result((True,'waypoint_queued'))

    def consume_waypoint(self):
        """READY order: homing, init, local/auto, search, global, gate, waypoint."""
        if (self.pending_waypoint is None or self.pending_homing is not None or self.pending_initialization is not None or
                self.pending_trigger is not None or self.pending_search is not None or self.pending_global is not None or
                self.pending_gate is not None or
                self.auto or self.landing.latched or self.home_call is not None or
                self.state not in {'PREVIEW','DONE'} or self.goal is not None or self.pending_goal or self.committing()):return False
        request=self.pending_waypoint;self.pending_waypoint=None
        task=copy.deepcopy(request['parameters'])
        reason=self.waypoint_request_error(task)
        if reason:self.fail('queued_waypoint_'+reason);return True
        # Target and branch were frozen at admission. Only the actual start
        # is selected now, after the preceding motion has really completed.
        task['source']=copy.deepcopy(self.odom)
        self.applied_waypoint=self.waypoint_request_status(request)
        self.applied_waypoint.update(source=list(position(task['source'])),source_stamp_s=self.odom_stamp,
                                     completed_actions=self.completed,cycles=self.cycles)
        self.epoch+=1;self.route_token+=1
        self.preview=False;self.requested_bound_mode=self.bound_mode=request['bound_mode']
        self.requested_vel_max=0.;self.vel_max=self.configured_vel_max
        self.manual_path=task['branch']=='direct';self.manual_task=None;self.explicit_global=None
        self.initialization_resume=None
        self.search_task=None;self.waypoint_task=task;self.gate_task=None
        self.retry_bound_mode=None;self.readiness_failure=None
        self.ahead=None;self.ahead_schedule=None;self.ahead_home_required=False;self.ahead_global_required=False
        self.feedback={};self.feedback_received=0.;self.commanded=[];self.progress=None
        self.validation_pending=False;self.validation_future=None
        self.request_plan('waypoint_direct' if self.manual_path else 'waypoint');return True

    def complete_manual(self,success):
        pending=self.manual_reply
        if pending is None:return
        self.manual_reply=None
        if not pending['future'].done():pending['future'].set_result(bool(success))

    async def start_manual(self,mode,path):
        if not self.idle():return False,'controller_busy'
        if not self.ready():return False,'map_or_fresh_odometry_missing'
        if not self.validate.service_is_ready():return False,'path_validation_unavailable'
        if not self.follower.server_is_ready():return False,'executor_unavailable'
        finished=Future()
        self.manual_reply={'future':finished,'deadline':time.monotonic()+3.*self.request_timeout}
        self.begin(mode,False,False,manual_parameters={'path':copy.deepcopy(path)},
                   bound_mode=PlannerSrv.Request.K_EXACT_BOUND)
        accepted=await finished
        return accepted,mode+'_execution_started' if accepted else self.reason

    async def rotate_180(self,request,response):
        if not self.ready():response.success=False;response.message='map_or_fresh_odometry_missing';return response
        source=copy.deepcopy(self.odom);target=copy.deepcopy(source)
        # Despite its name, the original callback targets absolute world yaw
        # pi. It does not add pi to the vehicle's present heading.
        target.orientation.x=0.;target.orientation.y=0.;target.orientation.z=1.;target.orientation.w=0.
        response.success,response.message=await self.start_manual('rotate_180',[source,target])
        return response

    def initialization_path(self):
        """Build from the pose at execution, never from a queued request."""
        source=copy.deepcopy(self.odom);path=[source]
        if self.robot_type=='kAerial':
            # Preserve the upstream order and world-X displacement. Aerial
            # initialization explicitly commands zero yaw after its first pose.
            up=copy.deepcopy(source);up.position.z+=self.init_z_takeoff
            up.orientation.x=0.;up.orientation.y=0.;up.orientation.z=0.;up.orientation.w=1.
            down=copy.deepcopy(up);down.position.z-=self.init_z_drop
            forward=copy.deepcopy(down);forward.position.x+=self.init_x_forward
            path.extend([up,down,forward])
        else:
            forward=copy.deepcopy(source);forward.position.x+=self.init_x_forward
            path.append(forward)
        return path

    async def initialize_motion(self,request,response):
        # Do not read/change a pending slot or start motion from this
        # reentrant callback: tick may be completing the previous Action.
        finished=Future()
        with self.initialization_inbox_lock:self.initialization_inbox.append(finished)
        response.success=await finished
        return response

    def take_initialization_inbox(self):
        with self.initialization_inbox_lock:
            requests=self.initialization_inbox;self.initialization_inbox=[]
        return requests

    def reject_initialization_inbox(self):
        for finished in self.take_initialization_inbox():
            if not finished.done():finished.set_result(False)

    def receive_initialization_requests(self):
        """Default-group handoff; no await or cross-group slot mutation."""
        for finished in self.take_initialization_inbox():
            if finished.done():continue
            if (self.landing.latched or self.home_call is not None or not self.ready() or
                    not self.validate.service_is_ready() or not self.follower.server_is_ready()):
                finished.set_result(False);continue
            if self.idle():
                # The service coroutine remains in its reentrant group while
                # the default group later resolves the accepted Action ACK.
                self.manual_reply={'future':finished,'deadline':time.monotonic()+3.*self.request_timeout}
                self.begin('initialization',False,False,
                    manual_parameters={'path':self.initialization_path()},bound_mode=PlannerSrv.Request.K_EXACT_BOUND)
                continue
            if self.state not in {'PLANNING','VALIDATING','SENDING','EXECUTING',
                                  'WAITING_REPLAN','WAITING_OBSERVATION','PREVIEW','DONE'}:
                finished.set_result(False);continue
            self.initialization_sequence+=1
            requests=1 if self.pending_initialization is None else self.pending_initialization['requests']+1
            self.pending_initialization={'sequence':self.initialization_sequence,'requests':requests}
            if self.ahead is not None and self.ahead['stage']=='planning':
                self.ahead['event']['outcome']='initialization_queued';self.ahead=None
            self.transition(self.state,'initialization_queued')
            finished.set_result(True)

    def consume_initialization(self):
        """Consume init_request_ at a completed request/motion boundary.

        An asynchronous planning or validation request is allowed to finish;
        unlike homing, initialization does not discard that ordinary work.
        Dispatched continuation acknowledgements still own their final route.
        """
        if (self.pending_initialization is None or self.pending_homing is not None or
                self.landing.latched or self.home_call is not None or self.goal is not None or
                self.pending_goal or self.committing() or
                self.state not in {'PREVIEW','DONE','WAITING_OBSERVATION','WAITING_REPLAN'}):return False
        if not self.ready():self.fail('queued_initialization_map_or_fresh_odometry_missing');return True
        if not self.validate.service_is_ready():self.fail('queued_initialization_path_validation_unavailable');return True
        if not self.follower.server_is_ready():self.fail('queued_initialization_executor_unavailable');return True
        request=self.pending_initialization;self.pending_initialization=None
        path=self.initialization_path()
        self.applied_initialization=dict(request,completed_actions=self.completed,cycles=self.cycles,
            source=list(position(path[0])),endpoint=list(position(path[-1])))
        # initMotion does not reset the original PCI trigger mode. Retain an
        # automatic task and ordinary pending slots while this manual motion
        # runs. A second queued init inherits the same interrupted settings.
        if self.initialization_resume is None:
            self.initialization_resume={'auto':self.auto,'preview':self.preview,
                'requested_bound_mode':self.requested_bound_mode,'bound_mode':self.bound_mode,
                'requested_vel_max':self.requested_vel_max,'vel_max':self.vel_max,
                'retry_pending':self.state=='WAITING_REPLAN','retry_bound_mode':self.retry_bound_mode,
                'ahead_home_required':self.ahead_home_required,'ahead_global_required':self.ahead_global_required}
        self.epoch+=1;self.route_token+=1
        self.auto=False;self.preview=False
        self.requested_bound_mode=self.bound_mode=PlannerSrv.Request.K_EXACT_BOUND
        self.manual_path=True;self.manual_task={'path':path};self.explicit_global=None
        self.search_task=None;self.waypoint_task=None;self.gate_task=None
        self.retry_bound_mode=None;self.readiness_failure=None
        self.ahead=None;self.ahead_schedule=None;self.ahead_home_required=False;self.ahead_global_required=False
        self.feedback={};self.feedback_received=0.;self.commanded=[];self.progress=None
        self.validation_pending=False;self.validation_future=None
        self.request_plan('initialization');return True

    def restore_after_initialization(self):
        resume=self.initialization_resume
        if resume is None:return False
        self.initialization_resume=None
        retry=resume.pop('retry_pending')
        for field,value in resume.items():setattr(self,field,value)
        self.manual_path=False;self.manual_task=None
        if self.auto or retry:self.mode='local'
        return retry

    def gate_status(self):
        if self.gate_task is None:return None
        return {'preview':self.gate_task['preview'],
                'bound_mode':self.gate_task.get('bound_mode'),
                'path_points':len(self.gate_task.get('planned_path',[])),
                'endpoint':self.gate_task.get('endpoint'),
                'command_points':len(self.commanded)}

    def complete_gate(self,success,reason=None):
        pending=self.gate_reply
        if pending is None:return
        self.gate_reply=None
        message=('passing_gate_path_ready' if self.preview else 'passing_gate_execution_started') if success else reason or self.reason
        if not pending['future'].done():pending['future'].set_result((bool(success),message))

    async def start_gate(self,response,preview):
        # The reentrant service callback must not mutate task state or race
        # an Action result/stop. Tick serializes admission with READY dispatch.
        finished=Future()
        with self.gate_inbox_lock:self.gate_inbox.append((bool(preview),finished))
        response.success,response.message=await finished
        return response

    def take_gate_inbox(self):
        with self.gate_inbox_lock:
            requests=self.gate_inbox;self.gate_inbox=[]
        return requests

    def reject_gate_inbox(self,reason):
        # Unaccepted callbacks are rejected; an already acknowledged pending
        # request and the upstream success latch deliberately survive reset.
        for _,finished in self.take_gate_inbox():
            if not finished.done():finished.set_result((False,reason))

    def gate_request_error(self,preview):
        if not preview and self.passing_gate_success:return 'passing_gate_already_activated'
        if not self.ready():return 'map_or_fresh_odometry_missing'
        if not self.passing_gate_planner.service_is_ready():return 'passing_gate_service_unavailable'
        if not self.validate.service_is_ready():return 'path_validation_unavailable'
        if not preview and not self.follower.server_is_ready():return 'executor_unavailable'
        return None

    def receive_gate_requests(self):
        for preview,finished in self.take_gate_inbox():
            if finished.done():continue
            reason=self.gate_request_error(preview)
            if self.landing.latched or self.home_call is not None or self.state=='STOPPING':
                reason=reason or 'controller_unavailable'
            if reason:finished.set_result((False,reason));continue
            if self.idle() and (not preview or self.pending_gate is None):
                # An explicit pass_gate can resume its suspended slot after
                # stop/error. Other begin() calls retain that slot instead.
                if not preview:self.pending_gate=None
                self.gate_reply={'future':finished,'deadline':time.monotonic()+3.*self.request_timeout}
                self.begin('passing_gate',False,preview,gate_parameters={'preview':preview})
                continue
            # Preview is a ROS 2 extension, not the upstream execution flag.
            # Keep it idle-only so it cannot replace an accepted pass_gate.
            if preview:finished.set_result((False,'controller_busy'));continue
            if self.state not in {'PLANNING','VALIDATING','SENDING','EXECUTING',
                                  'WAITING_REPLAN','WAITING_OBSERVATION','PREVIEW','DONE'}:
                finished.set_result((False,'controller_unavailable'));continue
            if self.mode=='passing_gate' and not self.preview and self.state not in self.TERMINAL:
                # Repeated activation while this same gate request is already
                # in flight is coalesced, not a second crossing after success.
                finished.set_result((True,'passing_gate_in_progress'));continue
            self.gate_sequence+=1
            requests=1 if self.pending_gate is None else self.pending_gate['requests']+1
            self.pending_gate={'sequence':self.gate_sequence,'requests':requests}
            self.transition(self.state,'passing_gate_trigger_queued')
            finished.set_result((True,'passing_gate_queued'))

    def consume_gate(self):
        """Consume the single gate flag after global and before waypoint."""
        if (self.pending_gate is None or self.pending_homing is not None or self.pending_initialization is not None or
                self.pending_trigger is not None or self.pending_search is not None or self.pending_global is not None or
                self.auto or self.landing.latched or self.home_call is not None or
                self.state not in {'PREVIEW','DONE'} or self.goal is not None or self.pending_goal or self.committing()):return False
        request=self.pending_gate;self.pending_gate=None
        reason=self.gate_request_error(False)
        if reason:self.fail('queued_passing_gate_'+reason);return True
        self.applied_gate=dict(request,completed_actions=self.completed,cycles=self.cycles)
        # Do not begin(): preserve the lower-priority waypoint and counters.
        self.epoch+=1;self.route_token+=1
        self.auto=False;self.preview=False
        self.requested_bound_mode=self.bound_mode=PlannerSrv.Request.K_EXACT_BOUND
        self.requested_vel_max=0.;self.vel_max=self.configured_vel_max
        self.manual_path=False;self.manual_task=None;self.explicit_global=None
        self.initialization_resume=None
        self.search_task=None;self.waypoint_task=None;self.gate_task={'preview':False}
        self.retry_bound_mode=None;self.readiness_failure=None
        self.ahead=None;self.ahead_schedule=None;self.ahead_home_required=False;self.ahead_global_required=False
        self.feedback={};self.feedback_received=0.;self.commanded=[];self.progress=None
        self.validation_pending=False;self.validation_future=None
        self.request_plan('passing_gate');return True

    async def std_pass_gate(self,request,response):
        return await self.start_gate(response,False)

    async def std_preview_gate(self,request,response):
        return await self.start_gate(response,True)

    async def set_home_result(self):
        if not self.idle() or not self.ready() or not self.home_setter.service_is_ready():return False
        finished=Future();pending=self.home_setter.call_async(PlannerSetHomingPos.Request())
        entry={'future':finished,'pending':pending,'deadline':time.monotonic()+self.request_timeout}
        self.home_call=entry
        def complete(done):
            if self.home_call is not entry or finished.done():return
            try:success=bool(done.result().success)
            except Exception:success=False
            finished.set_result(success)
        pending.add_done_callback(complete)
        success=await finished
        if self.home_call is entry:self.home_call=None
        return success

    async def set_home(self,request,response):
        response.success=await self.set_home_result();return response

    async def std_set_home(self,request,response):
        response.success=await self.set_home_result()
        response.message='homing_position_set' if response.success else 'homing_position_not_set'
        return response

    def std_planning(self,response,automatic):
        request=PciTrigger.Request();request.set_auto=automatic;request.not_exe_path=False
        request.bound_mode=PlannerSrv.Request.K_EXTENDED_BOUND;request.vel_max=0.
        if not self.local.service_is_ready():response.success=False;response.message='local_service_unavailable';return response
        forwarded=self.trigger(request,PciTrigger.Response())
        response.success=forwarded.success and self.state not in {'ERROR','STOPPED'}
        response.message=self.reason if response.success else 'planning_request_rejected';return response

    def std_automatic(self,request,response):return self.std_planning(response,True)
    def std_single(self,request,response):return self.std_planning(response,False)

    def std_homing(self,request,response):
        if not self.homing.service_is_ready():response.success=False;response.message='homing_service_unavailable';return response
        forwarded=PciHomingTrigger.Request();forwarded.not_exe_path=False
        result=self.trigger_homing(forwarded,PciHomingTrigger.Response())
        response.success=result.success and self.state not in {'ERROR','STOPPED'}
        response.message=self.reason if response.success else 'homing_request_rejected';return response

    def std_stop(self,request,response):
        self.cancel('user_stop','STOPPED');response.success=True;response.message='stop_requested';return response

    def std_replan(self,request,response):
        response.success=False
        if self.last_frontier_parameters is None:response.message='no_previous_explicit_frontier';return response
        forwarded=PciGlobal.Request();forwarded.not_exe_path=False;forwarded.set_auto=False
        forwarded.bound_mode=self.last_frontier_parameters.get('bound_mode',PlannerSrv.Request.K_EXACT_BOUND);forwarded.vel_max=0.
        for field,value in self.last_frontier_parameters.items():setattr(forwarded,field,value)
        result=self.trigger_global(forwarded,PciGlobal.Response())
        response.success=result.success and self.state not in {'ERROR','STOPPED'}
        response.message=self.reason if response.success else 'global_replan_rejected';return response

    def idle(self):
        return (not self.landing.latched and self.state in self.TERMINAL
                and self.goal is None and not self.pending_goal and self.home_call is None
                and self.pending_trigger is None and self.pending_homing is None and self.pending_global is None
                and self.pending_initialization is None and self.pending_search is None and self.pending_waypoint is None
                # A retained gate is suspended in these terminal states.
                # Permit an explicit restart without automatically moving.
                and (self.pending_gate is None or self.state in {'ERROR','STOPPED'}))

    def payload(self):
        return {'state':self.state,'reason':self.reason,'mode':self.mode,'auto':self.auto,
                'requested_bound_mode':self.requested_bound_mode,'bound_mode':self.bound_mode,
                'manual_path':self.manual_path,'requested_vel_max':self.requested_vel_max,'vel_max':self.vel_max,
                'configured_vel_max':self.configured_vel_max,
                'ground_maneuver':{'enabled':self.ground_maneuver_enabled,
                    'active':self.ground_maneuver is not None,'report':self.maneuver_report},
                'pending_trigger':self.pending_trigger,'applied_trigger':self.applied_trigger,
                'pending_homing':self.pending_homing,'applied_homing':self.applied_homing,
                'pending_global':self.pending_global,'applied_global':self.applied_global,
                'pending_search':self.search_request_status(self.pending_search),'applied_search':self.applied_search,
                'pending_waypoint':self.waypoint_request_status(self.pending_waypoint),'applied_waypoint':self.applied_waypoint,
                'pending_gate':self.pending_gate,'applied_gate':self.applied_gate,
                'passing_gate_success':self.passing_gate_success,
                'pending_initialization':self.pending_initialization,'applied_initialization':self.applied_initialization,
                'follower_tracks_route':self.follower_tracks_route,
                'path_invalidation_recovery':{'limit':self.path_invalidation_retries,
                    'attempts':self.recovery_attempts,'consecutive_attempts':self.recovery_consecutive,
                    'pending':self.recovery_pending,'events':self.recovery_events[-32:]},
                'initialization':{'robot_type':self.robot_type,'z_takeoff':self.init_z_takeoff,
                                  'z_drop':self.init_z_drop,'x_forward':self.init_x_forward,
                                  'resume':self.initialization_resume},
                'landing':self.landing.payload(),
                'cycles':self.cycles,'completed_actions':self.completed,'map_revision':self.map_revision,'known_cells':self.known_cells,
                'observation_revision':self.observation_revision,
                'odometry_freshness':self.odometry_freshness(),
                'odometry_diagnostics':self.odom_diagnostics,'readiness_failure':self.readiness_failure,
                'mission_freshness':self.mission_freshness(),
                'mission_source_rejections':self.mission_source_rejections,
                'feedback':self.feedback,'plan_ahead':{'lead_time':self.lead_time,
                    'adaptive_lead':self.adaptive_lead,'effective_lead_time':self.effective_lead_time(),
                    'early_start':self.early_start,'schedule':self.ahead_schedule,
                    'planning_latencies':self.planning_latencies,'handoff_latencies':self.handoff_latencies,
                    'attempts':self.ahead_events,'generation':self.route_generation,
                    'continuations':self.ahead_count,'pending':self.ahead is not None,'home_required':self.ahead_home_required,
                    'result_deferred':self.deferred_result is not None,
                    'global_required':self.ahead_global_required},'explicit_global':self.explicit_global,'last_explicit_frontier':self.last_frontier_parameters,'search':self.search_status(),'waypoint':self.waypoint_status(),'passing_gate':self.gate_status(),'events':self.events[-100:]}

    def transition(self,state,reason):
        self.state=state;self.reason=reason
        self.events.append({'state':state,'reason':reason,'mode':self.mode,'bound_mode':self.bound_mode})
        message=String();message.data=json.dumps(self.payload());self.status_pub.publish(message)

    def clear_command(self):
        message=Path();message.header.frame_id=self.frame;message.header.stamp=self.get_clock().now().to_msg()
        self.path_pub.publish(message)

    def get_status(self,request,response):
        response.success=True;response.message=json.dumps(self.payload());return response

    def trigger(self,request,response):
        response.success=False
        if not self.ready() or request.bound_mode not in range(5) or not math.isfinite(request.vel_max) or (request.not_exe_path and request.set_auto):return response
        if not request.not_exe_path and not self.follower.server_is_ready():return response
        if self.idle():
            self.begin('local',request.set_auto,request.not_exe_path,bound_mode=request.bound_mode,vel_max=request.vel_max)
        else:
            if (self.landing.latched or self.home_call is not None or
                    self.state not in {'PLANNING','VALIDATING','SENDING','EXECUTING',
                                       'WAITING_REPLAN','WAITING_OBSERVATION','PREVIEW','DONE'}):return response
            # Every accepted request overwrites the single run_en_ slot. The
            # original setVelocity ignores values outside [0.2, 1.0], so a
            # later zero must preserve an earlier queued effective speed.
            if .2<=request.vel_max<=1.:self.configured_vel_max=float(request.vel_max)
            self.trigger_sequence+=1
            self.pending_trigger={'sequence':self.trigger_sequence,'auto':bool(request.set_auto),
                'preview':bool(request.not_exe_path),'bound_mode':int(request.bound_mode),
                'requested_vel_max':float(request.vel_max),'vel_max':self.configured_vel_max}
            # A speculative plan can be forgotten; a dispatched continuation
            # may already have changed the follower route before its ACK.
            # Keep that commit and its generation authoritative until finish.
            if self.ahead is not None and self.ahead['stage']=='planning':self.ahead=None
            self.transition(self.state,'planning_trigger_queued')
        response.success=True;return response

    def consume_trigger(self):
        """Apply the latest accepted trigger at an observed planning boundary."""
        if (self.pending_trigger is None or self.pending_homing is not None or self.pending_initialization is not None or
                self.goal or self.pending_goal or self.committing()):return False
        trigger=self.pending_trigger;self.pending_trigger=None
        self.applied_trigger=dict(trigger,completed_actions=self.completed,cycles=self.cycles)
        # Do not call begin(): this is the next round of the same task, with
        # its completed actions, cycles, events and continuation count intact.
        self.epoch+=1;self.route_token+=1
        self.auto=trigger['auto'];self.preview=trigger['preview']
        self.requested_bound_mode=trigger['bound_mode'];self.bound_mode=trigger['bound_mode']
        self.requested_vel_max=trigger['requested_vel_max'];self.vel_max=trigger['vel_max']
        self.manual_path=False;self.manual_task=None;self.explicit_global=None
        self.initialization_resume=None
        self.search_task=None;self.waypoint_task=None;self.gate_task=None
        self.retry_bound_mode=None;self.readiness_failure=None
        self.ahead=None;self.ahead_schedule=None;self.ahead_home_required=False;self.ahead_global_required=False
        self.feedback={};self.feedback_received=0.;self.commanded=[]
        self.progress=None;self.validation_pending=False;self.validation_future=None
        self.request_plan('local');return True

    def trigger_homing(self,request,response):
        response.success=False
        if self.landing.latched or self.home_call is not None or self.state=='STOPPING' or not self.ready():return response
        if self.state not in self.TERMINAL|{'PLANNING','VALIDATING','SENDING','EXECUTING','WAITING_REPLAN','WAITING_OBSERVATION'}:return response
        if self.state in {'ERROR','STOPPED'} and (self.goal is not None or self.pending_goal):return response
        if not request.not_exe_path and not self.follower.server_is_ready():return response
        if self.idle():
            self.begin('homing',False,request.not_exe_path);response.success=True;return response
        self.homing_sequence+=1
        self.pending_homing={'sequence':self.homing_sequence,'preview':bool(request.not_exe_path)}
        if self.ahead is not None and self.ahead['stage']=='planning':
            self.ahead['event']['outcome']='explicit_homing_queued';self.ahead=None
        self.transition(self.state,'homing_trigger_queued')
        self.consume_homing()
        response.success=True;return response

    def consume_homing(self):
        """Run the latest homing request at PCI's no-motion boundary."""
        if (self.pending_homing is None or self.landing.latched or self.home_call is not None or
                self.state in {'STOPPING','WAITING_RECOVERY','WAITING_RECOVERY_OBSERVATION'} or
                self.goal is not None or self.pending_goal or self.committing() or
                not self.ready()):return False
        request=self.pending_homing;self.pending_homing=None
        self.applied_homing=dict(request,completed_actions=self.completed,cycles=self.cycles)
        # PLANNING/VALIDATING have not committed an Action. Invalidate those
        # asynchronous replies before requesting home; an executing Action
        # and a dispatched continuation must instead finish normally first.
        self.epoch+=1;self.route_token+=1
        if self.pending is not None:self.pending.cancel()
        if self.validation_future is not None:self.validation_future.cancel()
        self.pending=None;self.validation_future=None;self.validation_pending=False
        self.complete_search(False);self.complete_waypoint(False);self.complete_gate(False,'superseded_by_homing');self.complete_manual(False)
        self.pending_trigger=None;self.applied_trigger=None;self.auto=False;self.preview=request['preview']
        self.initialization_resume=None
        # PciHomingTrigger carries no bound setting. Match an idle explicit
        # homing request instead of inheriting local relaxation/point mode.
        self.requested_bound_mode=self.bound_mode=PlannerSrv.Request.K_EXACT_BOUND
        self.manual_path=False;self.manual_task=None;self.explicit_global=None
        self.search_task=None;self.waypoint_task=None;self.gate_task=None
        self.retry_bound_mode=None;self.readiness_failure=None
        self.ahead=None;self.ahead_schedule=None;self.ahead_home_required=False;self.ahead_global_required=False
        self.feedback={};self.feedback_received=0.;self.commanded=[];self.progress=None
        self.request_plan('homing');return True

    def trigger_global(self,request,response):
        """Forward the upstream PCI global target as a single explicit task.

        An unavailable explicit target must fail without sending the vehicle
        to a different frontier or home.
        """
        response.success=False
        if request.set_auto:return response
        if self.landing.latched or self.home_call is not None or self.state=='STOPPING':return response
        if self.state not in self.TERMINAL|{'PLANNING','VALIDATING','SENDING','EXECUTING','WAITING_REPLAN','WAITING_OBSERVATION'}:return response
        if self.state in {'ERROR','STOPPED'} and (self.goal is not None or self.pending_goal):return response
        parameters={'id':int(request.id),'not_check_frontier':bool(request.not_check_frontier),
                    'ignore_time':bool(request.ignore_time),'bound_mode':int(request.bound_mode)}
        if self.global_request_error(parameters,request.not_exe_path,request.vel_max):return response
        if request.id>0:self.last_frontier_parameters=copy.deepcopy(parameters)
        if self.idle():
            self.begin('global',False,request.not_exe_path,parameters,bound_mode=request.bound_mode,vel_max=request.vel_max)
        else:
            # Original global_request_ retains only the latest frontier. Its
            # settings must not change a route already owned by the follower.
            if .2<=request.vel_max<=1.:self.configured_vel_max=float(request.vel_max)
            self.global_sequence+=1
            self.pending_global={'sequence':self.global_sequence,'parameters':parameters,
                'preview':bool(request.not_exe_path),'requested_vel_max':float(request.vel_max),
                'vel_max':self.configured_vel_max}
            self.transition(self.state,'global_trigger_queued')
        response.success=True;return response

    def global_request_error(self,parameters,preview,velocity):
        if parameters['bound_mode'] not in range(5) or not math.isfinite(velocity):return 'invalid_parameters'
        if parameters['id']<0 or (parameters['id']==0 and parameters['not_check_frontier']):return 'invalid_target'
        if not parameters['ignore_time'] and (self.mission is None or not self.mission['enabled']):return 'mission_budget_unavailable'
        if not self.ready():return 'map_or_fresh_odometry_missing'
        if not self.global_plan.service_is_ready():return 'global_service_unavailable'
        if not preview and not self.follower.server_is_ready():return 'executor_unavailable'
        return None

    def consume_global(self):
        """Drain global_request_ after homing, local work and current search."""
        if (self.pending_global is None or self.pending_homing is not None or self.pending_initialization is not None or self.pending_trigger is not None or
                self.pending_search is not None or
                self.auto or self.landing.latched or self.home_call is not None or
                self.state not in {'PREVIEW','DONE'} or self.goal is not None or self.pending_goal or self.committing()):return False
        request=copy.deepcopy(self.pending_global);self.pending_global=None
        # The mission/service state may have changed while a previous route
        # ran. Acceptance of the queue slot is not permission to bypass it.
        reason=self.global_request_error(request['parameters'],request['preview'],request['requested_vel_max'])
        if reason:self.fail('queued_global_'+reason);return True
        self.applied_global=dict(request,completed_actions=self.completed,cycles=self.cycles)
        self.epoch+=1;self.route_token+=1
        self.auto=False;self.preview=request['preview']
        self.requested_bound_mode=self.bound_mode=request['parameters']['bound_mode']
        self.requested_vel_max=request['requested_vel_max'];self.vel_max=request['vel_max']
        self.manual_path=False;self.manual_task=None;self.explicit_global=request['parameters']
        self.initialization_resume=None
        self.search_task=None;self.waypoint_task=None;self.gate_task=None
        self.retry_bound_mode=None;self.readiness_failure=None
        self.ahead=None;self.ahead_schedule=None;self.ahead_home_required=False;self.ahead_global_required=False
        self.feedback={};self.feedback_received=0.;self.commanded=[];self.progress=None
        self.validation_pending=False;self.validation_future=None
        self.request_plan('global');return True

    def begin(self,mode,auto,preview,global_parameters=None,search_parameters=None,waypoint_parameters=None,gate_parameters=None,bound_mode=3,manual_parameters=None,vel_max=0.):
        # Freeze the incoming task's mode. Only local padding fallback and a
        # gate response can select a different effective mode for its route.
        self.requested_bound_mode=int(bound_mode);self.bound_mode=int(bound_mode)
        self.manual_path=mode in {'waypoint_direct','rotate_180','initialization'}
        self.manual_task=manual_parameters;self.requested_vel_max=float(vel_max)
        # PCIGeneral only changes speed within [0.2, 1.0] m/s. Its accepted
        # setting survives reset/stop even when a queued round was discarded;
        # active vel_max remains frozen until this next command is applied.
        if .2<=vel_max<=1.:self.configured_vel_max=float(vel_max)
        self.vel_max=self.configured_vel_max
        self.epoch+=1;self.cycles=0;self.completed=0;self.events=[];self.feedback={}
        self.recovery_pending=None;self.recovery_attempts=0;self.recovery_consecutive=0
        self.recovery_events.clear();self.execution_outcomes.clear()
        self.pending_trigger=None;self.applied_trigger=None
        self.pending_homing=None;self.applied_homing=None
        self.pending_global=None;self.applied_global=None
        self.pending_search=None;self.applied_search=None
        self.pending_waypoint=None;self.applied_waypoint=None
        # Unlike the ordinary task slots, upstream resetPlanner preserves
        # both gate flags. Stop/error suspend pending_gate until a new start.
        self.applied_gate=None
        self.pending_initialization=None;self.applied_initialization=None;self.initialization_resume=None
        self.readiness_failure=None
        self.retry_bound_mode=None
        self.ahead=None;self.ahead_schedule=None;self.ahead_home_required=False;self.ahead_global_required=False;self.ahead_count=0
        self.route_generation=0;self.route_token+=1;self.feedback_received=0.
        self.planning_latencies.clear();self.handoff_latencies.clear();self.ahead_events.clear()
        self.local_rpc_started=None;self.local_handoff_started=None
        self.explicit_global=global_parameters
        self.search_task=search_parameters
        self.waypoint_task=waypoint_parameters
        self.gate_task=gate_parameters
        self.auto=auto;self.preview=preview;self.commanded=[];self.validation_pending=False;self.request_plan(mode)

    def request_plan(self,mode,bound_mode=None):
        # Only a completed PlanAhead handoff owns the automatic global target.
        # Save the latch before consuming it; manual requests remain one-shot
        # global queries, even during an automatic PCI task.
        automatic_repositioning=(mode=='global' and self.ahead_global_required and self.explicit_global is None)
        self.mode=mode
        self.ground_maneuver=None;self.maneuver_candidates=[];self.maneuver_started_ros=None
        self.maneuver_candidate_obligations=[]
        self.maneuver_report=None;self.maneuver_home_cost=0.
        if mode in {'global','homing'}:self.ahead_global_required=False
        if mode=='homing':self.auto=False;self.pending_trigger=None
        if mode=='local':
            # A new exploration cycle starts from its original request; a
            # retry supplies the next padding level explicitly.
            self.bound_mode=self.requested_bound_mode if bound_mode is None else bound_mode
        self.plan_observation=self.observation_revision
        if not (self.search_ready() if mode=='search' else self.ready()):self.fail('map_or_fresh_odometry_missing');return
        # Planning, reconnect validation and dispatch share this request's
        # deadline. A moving source must not restart the clock on every check.
        self.pre_execution_deadline=time.monotonic()+self.request_timeout
        self.deadline=self.pre_execution_deadline;self.pre_execution_rechecks=0
        self.pre_execution_path=[];self.pre_execution_observation=False
        if mode=='waypoint_direct':
            self.transition('PLANNING','direct_waypoint_requested')
            self.prepare_path([copy.deepcopy(self.waypoint_task['source']),copy.deepcopy(self.waypoint_task['target'].pose)],self.epoch,mode)
            return
        if mode in {'rotate_180','initialization'}:
            self.transition('PLANNING',mode+'_requested')
            self.prepare_path(copy.deepcopy(self.manual_task['path']),self.epoch,mode)
            return
        if mode=='local':
            request=PlannerSrv.Request();request.header.frame_id=self.frame
            request.header.stamp=self.get_clock().now().to_msg();request.bound_mode=self.bound_mode
            request.root_pose.orientation.w=0.;client=self.local
        elif mode=='global':
            request=PlannerGlobal.Request();request.ignore_time=not(self.mission and self.mission['enabled']);client=self.global_plan
            request.bound_mode=self.bound_mode
            request.automatic_repositioning=automatic_repositioning
            if self.explicit_global is not None:
                request.id=self.explicit_global['id']
                request.not_check_frontier=self.explicit_global['not_check_frontier']
                request.ignore_time=self.explicit_global['ignore_time']
        elif mode=='search':
            request=PlannerSearch.Request();request.header.frame_id=self.frame
            request.header.stamp=self.get_clock().now().to_msg();request.bound_mode=self.bound_mode
            # Always send the request's frozen source explicitly, including
            # use_current_state. Planner queueing must not choose a later pose.
            request.use_current_state=False;request.source=copy.deepcopy(self.search_task['source'])
            request.target=copy.deepcopy(self.search_task['target']);client=self.search_planner
        elif mode=='waypoint':
            request=PlannerGoToWaypoint.Request();request.check_collision=True
            request.bound_mode=self.bound_mode
            request.waypoint=copy.deepcopy(self.waypoint_task['target']);client=self.waypoint_planner
        elif mode=='passing_gate':
            request=PlannerRequestPath.Request();client=self.passing_gate_planner
        else:
            request=PlannerHoming.Request();request.header.frame_id=self.frame
            request.bound_mode=self.bound_mode;client=self.homing
        if not client.service_is_ready():self.fail(mode+'_service_unavailable');return
        self.transition('PLANNING',mode+'_requested')
        if mode=='local':self.local_rpc_started=self.timing_start()
        epoch=self.epoch;self.pending=client.call_async(request)
        self.pending.add_done_callback(lambda future:self.planned(future,epoch,mode))

    def wait_local_retry(self,reason,bound_mode=None):
        # Upstream runPlanner waits 0.5 s after an empty reply. Keep that wait
        # asynchronous so stop, odometry, mission and landing remain live.
        # None starts the next automatic round at the requested padding;
        # a numeric mode continues this round's Extended/Relaxed/Minimum pass.
        self.retry_bound_mode=bound_mode
        self.deadline=time.monotonic()+.5
        self.clear_command()
        self.transition('WAITING_REPLAN',reason)

    def retry_local(self,reason,wait=False):
        # Upstream PCI tries Extended -> Relaxed -> Minimum, preferring a
        # forward route until the final level. Exact and Point are explicit
        # user choices; neither participates in automatic relaxation.
        if self.requested_bound_mode>2 or self.bound_mode not in (0,1):return False
        next_mode=self.bound_mode+1
        if wait:
            self.wait_local_retry(reason+':retry_bound_'+str(next_mode),next_mode);return True
        self.transition('PLANNING',reason+':retry_bound_'+str(next_mode))
        self.request_plan('local',next_mode);return True

    def planned(self,future,epoch,mode):
        if epoch!=self.epoch or future is not self.pending:return
        self.pending=None
        try:response=future.result()
        except Exception as error:
            if mode=='local' and self.retry_local('local_service_error'):return
            self.fail('planning_service_error:'+str(error));return
        if mode=='local':
            self.record_latency(self.planning_latencies,self.local_rpc_started,'local')
            self.local_handoff_started=self.timing_start()
            if response.planning_bound_mode not in range(5) or response.planning_bound_mode!=self.bound_mode:
                self.fail('unexpected_local_bound_mode');return
            if response.status not in (PlannerSrv.Response.K_HOMING,PlannerSrv.Response.K_REPOSITIONING):
                if not response.path and self.retry_local('local_path_empty',wait=True):return
                if response.path and response.status==PlannerSrv.Response.K_BACKWARD and self.retry_local('local_backward_path'):return
        # The original local response can carry a full homing route. Never
        # truncate it to the first local edge, or fall back on a stopped budget.
        if mode=='local' and response.status==PlannerSrv.Response.K_HOMING:
            if self.landing.enabled:
                self.cancel('automatic_homing_suppressed_by_landing','STOPPED');return
            mode='homing';self.mode='homing';self.auto=False;self.pending_trigger=None
            if not response.path:self.fail('mission_homing_unavailable');return
        if mode=='global' and getattr(response,'homing',False):
            # PlannerGlobal may itself return home when every frontier has
            # been explored. Carry that decision into completion handling so
            # an automatic PlanAhead global handoff cannot start local again.
            if self.landing.enabled:
                self.cancel('automatic_homing_suppressed_by_landing','STOPPED');return
            mode='homing';self.mode='homing';self.auto=False;self.pending_trigger=None
            if not response.path:self.fail('fully_explored_homing_unavailable');return
        if mode=='local' and response.status==PlannerSrv.Response.K_REPOSITIONING:
            # The planner already ran global selection after its persistent
            # no-frontier threshold. Preserve the whole route, and do not
            # restart local bound retries or request global selection twice.
            mode='global';self.mode='global'
            if not response.path:
                self.cancel('global_frontier_unavailable','STOPPED');return
        path=list(response.path)
        if mode=='search' and (not response.success or not path):self.fail('search_no_feasible_path');return
        if mode=='waypoint' and not path:self.fail('waypoint_global_path_empty');return
        if mode=='passing_gate':
            if not path:self.fail('passing_gate_path_empty');return
            # Upstream runPassingGate latches on a nonempty planner response,
            # before execution. Validation/Action failures must not clear it.
            # Preview does not activate the upstream one-shot operation.
            if not self.preview:
                self.passing_gate_success=True;self.pending_gate=None
            # Gate search can relax padding, but it must never shrink below
            # the physical robot or use the point-only NoBound mode.
            if response.bound.mode not in (0,1,2,3):self.fail('invalid_passing_gate_bound_mode');return
            self.bound_mode=int(response.bound.mode)
            self.gate_task.update(bound_mode=int(response.bound.mode),planned_path=copy.deepcopy(path),
                                  endpoint=list(position(path[-1])))
        # An empty route does not establish that global exploration or homing
        # is required. The planner owns those decisions and reports them using
        # K_REPOSITIONING / K_HOMING / PlannerGlobal.homing above. In auto mode
        # the original PCI keeps asking local, allowing the RRG's persistent
        # no-frontier counter to reach its own threshold.
        if not path:
            if mode=='local':
                if self.pending_trigger is not None:self.wait_local_retry('local_path_empty:retry_queued')
                elif self.auto:self.wait_local_retry('local_path_empty:retry_auto')
                else:self.fail('local_path_empty')
            elif mode=='global':
                if self.explicit_global is not None:self.fail('explicit_global_path_empty')
                else:self.cancel('global_frontier_unavailable','STOPPED')
            else:self.fail('homing_path_empty')
            return
        self.prepare_path(path,epoch,mode)

    def prepare_path(self,path,epoch,mode):
        if not (self.search_ready() if mode=='search' else self.ready()):self.fail('odometry_stale_after_planning');return
        for pose in path:
            q=pose.orientation
            if not all(math.isfinite(v) for v in (*position(pose),q.x,q.y,q.z,q.w)) or q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<1e-12:
                self.fail('invalid_planned_pose');return
        if mode=='search':
            self.search_task['planned_path']=copy.deepcopy(path)
            self.search_task['endpoint']=list(position(path[-1]))
            self.search_task['endpoint_distance']=math.dist(position(path[-1]),position(self.search_task['target']))
            self.search_task['exact_target']=self.search_task['endpoint_distance']<=1e-6
        elif mode in {'waypoint','waypoint_direct'}:
            self.waypoint_task['endpoint']=list(position(path[-1]))
            self.waypoint_task['endpoint_distance']=math.dist(position(path[-1]),position(self.waypoint_task['target'].pose))
            self.waypoint_task['exact_target']=self.waypoint_task['endpoint_distance']<=1e-6
        if self.manual_path and math.dist(position(path[0]),position(self.odom))>self.endpoint_tolerance:
            self.fail('manual_start_differs_from_odometry');return
        use_ground=(self.ground_maneuver_enabled and mode in {'local','search','homing'}
                and not self.manual_path and not self.preview)
        if use_ground and mode=='homing':
            # Homing must retain every ordered routing pose, including a
            # single-point request's heading and stationary-start requirement.
            self.prepare_ground_maneuver(path,epoch);return
        if len(path)==1 and mode not in {'search','waypoint','waypoint_direct','passing_gate'}:
            if math.dist(position(path[0]),position(self.odom))>self.endpoint_tolerance:
                self.fail('planned_start_differs_from_odometry');return
            self.transition('DONE','already_home' if mode=='homing' else 'no_motion');return
        if use_ground:
            self.prepare_ground_maneuver(path,epoch);return
        # Keep the planner response intact for previews and for any later
        # reconnect attempt; never repeatedly discard vertices from an already
        # modified route. All nonmanual execution modes share the upstream PCI
        # reconnect rule, including ordinary local/global/homing responses.
        self.pre_execution_path=copy.deepcopy(path)
        self.pre_execution_observation=not self.manual_path and not self.preview
        if self.pre_execution_observation:
            try:path=reconnect_path(self.pre_execution_path,self.odom)
            except ValueError as error:self.fail(str(error));return
        # Upstream executePath consumes the complete planner response. Lead
        # time controls speculative planning, not how many output/terrain
        # samples are executed before the next endpoint observation.
        self.route_generation=0;self.route_token+=1;self.ahead_schedule=None;self.feedback={}
        self.follower_tracks_route=False
        self.commanded=path;self.progress=PathProgress([position(p) for p in path]);self.validation_pending=False;self.validated_revision=0
        self.transition('VALIDATING','check_before_execution');self.deadline=self.pre_execution_deadline
        self.check_path(epoch,before_execution=True)

    @staticmethod
    def maneuver_yaw(pose):
        x,y,z,w=unit_quaternion(pose.orientation)
        return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))

    def maneuver_stopped(self):
        recent=[(stamp,motion) for stamp,motion in self.odom_stationary if stamp>=self.odom_stamp-.15]
        return (len(recent)>=3 and recent[-1][0]-recent[0][0]>=.1 and self.odom_motion is not None and
                all(m is not None and m[0]<=.02 and m[1]<=.03 for _,m in recent))

    def maneuver_budget_error(self):
        reason=self.mission_error()
        if reason:return reason
        if not(self.mission and self.mission['enabled']):return None
        freshness=self.mission_freshness()
        remaining=self.mission['remaining_s']-max(0.,freshness['age_s'])
        if self.mode=='homing':
            # remaining_s already includes the exploration reserve. Return
            # may spend it, but cannot extend the actual mission deadline.
            allowance=min(self.ground_maneuver_parameters['max_duration'],remaining)
            if self.ground_maneuver is not None:
                allowance=min(allowance,self.ground_maneuver.max_duration)
            reference=self.maneuver_report.get('reference_duration_s') if self.maneuver_report else None
            if self.maneuver_report is not None:
                self.maneuver_report['budget']={'enabled':True,'remaining_s':remaining,
                    'reference_duration_s':reference,'approved_execution_budget_s':allowance,
                    'mission_age_s':freshness['age_s'],
                    'reserve_consumable_for_homing':True}
            if allowance<=0 or (reference is not None and reference>=allowance):
                return 'ground_maneuver_mission_budget'
            if self.ground_maneuver is not None:
                self.ground_maneuver.max_duration=allowance
                self.maneuver_report['approved_duration_s']=allowance
                self.maneuver_report['reference_slack_s']=(
                    allowance-reference if reference is not None else None)
            return None
        # Reserve a complete inverse maneuver back to the current home-route
        # attachment as well as the approved outward execution upper bound.
        reserve=max(20.,float(self.mission.get('reserve_s',20.)))
        cost=2*self.ground_maneuver_parameters['max_duration']+self.maneuver_home_cost+reserve
        if self.maneuver_report is not None:
            self.maneuver_report['budget']={'enabled':True,'remaining_s':remaining,
                'approved_outward_s':self.ground_maneuver_parameters['max_duration'],
                'inverse_return_s':self.ground_maneuver_parameters['max_duration'],
                'current_home_cost_s':self.maneuver_home_cost,'reserve_s':reserve,'total_cost_s':cost}
        return 'ground_maneuver_mission_budget' if not math.isfinite(cost) or remaining<cost else None

    def prepare_ground_maneuver(self,path,epoch):
        if not self.maneuver_stopped():self.fail('ground_maneuver_start_not_stopped');return
        self.pre_execution_path=copy.deepcopy(path)
        self.pre_execution_observation=True
        self.maneuver_report={'task_mode':self.mode,'original_endpoint':list(position(path[-1]))+
            [self.maneuver_yaw(path[-1])],'original_route_points':len(path),
            'original_route':[[*position(p),self.maneuver_yaw(p)] for p in path],
            'candidate_rejections':[],'committed':False,
            'route_semantics':('complete ordered homing poses with stop boundaries' if self.mode=='homing'
                else 'complete ordered local graph poses with explicit straight crossings' if self.mode=='local'
                else 'new ordinary search endpoint route, proposed before dispatch')}
        if self.mode!='homing' and self.mission and self.mission['enabled']:
            if not self.homing.service_is_ready():self.fail('ground_maneuver_return_cost_unavailable');return
            request=PlannerHoming.Request();request.header.frame_id=self.frame;request.bound_mode=self.bound_mode
            self.transition('PLANNING','ground_maneuver_return_cost_requested')
            future=self.homing.call_async(request);self.pending=future
            future.add_done_callback(lambda done:self.maneuver_return_cost(done,epoch))
        else:self.construct_ground_maneuver(epoch)

    def maneuver_return_cost(self,future,epoch):
        if epoch!=self.epoch or future is not self.pending:return
        self.pending=None
        try:
            path=list(future.result().path)
            if not path or any(not valid_pose(p) for p in path):raise ValueError('return_path_missing')
            route=[self.odom]+path
            length=sum(math.dist(position(a),position(b)) for a,b in zip(route,route[1:]))
            angle=sum(abs(math.remainder(self.maneuver_yaw(b)-self.maneuver_yaw(a),2*math.pi))
                      for a,b in zip(route,route[1:]))
            limits=self.ground_maneuver_parameters
            speed=min(limits['max_speed'],self.vel_max) if self.vel_max>0 else limits['max_speed']
            # Existing route travel model plus explicit turns and stop/ramp
            # allowance; no disabled budget or zero return-cost fallback.
            self.maneuver_home_cost=(length/speed+angle/limits['max_yaw_rate']+
                len(path)*(limits['max_speed']/limits['max_acceleration']+
                    limits['max_yaw_rate']/limits['max_yaw_acceleration']+.15))
            self.construct_ground_maneuver(epoch)
        except Exception as error:self.fail('ground_maneuver_return_cost:'+str(error))

    def construct_ground_maneuver(self,epoch):
        if epoch!=self.epoch:return
        if time.monotonic()>=self.pre_execution_deadline:self.fail('pre_execution_timeout');return
        if not self.ready() or not self.maneuver_stopped():self.fail('ground_maneuver_start_changed');return
        try:
            limits=self.ground_maneuver_parameters
            start4=(*position(self.odom),self.maneuver_yaw(self.odom))
            start=(start4[0],start4[1],start4[3])
            endpoint=self.pre_execution_path[-1]
            if abs(endpoint.position.z-start4[2])>.04:
                raise ValueError('ground_maneuver_height_change_unsupported')
            speed=min(limits['max_speed'],self.vel_max) if self.vel_max>0 else limits['max_speed']
            allowance=limits['max_duration']
            if self.mode=='homing' and self.mission and self.mission['enabled']:
                allowance=min(allowance,self.mission['remaining_s']-max(0.,self.mission_freshness()['age_s']))
            self.maneuver_limits=self.maneuver_limits_type(max_speed=speed,
                max_yaw_rate=limits['max_yaw_rate'],max_acceleration=limits['max_acceleration'],
                max_yaw_acceleration=limits['max_yaw_acceleration'],max_duration=allowance,
                max_path_length=limits['max_path_length'],integration_dt=.05,max_points=20000,
                max_wall_time=min(limits['construction_wall_time'],self.pre_execution_deadline-time.monotonic()),
                max_candidates=2)
            if self.mode=='homing':
                result=self.construct_return_route(start4,
                    [(*position(p),self.maneuver_yaw(p)) for p in self.pre_execution_path],
                    offset=limits['control_offset'],limits=self.maneuver_limits,valid_pose=lambda _:True,
                    measured_speed=self.odom_motion[0],measured_yaw_rate=self.odom_motion[1],
                    route_source='planner_response_homing')
                self.maneuver_report['return_route']=result.diagnostics
                elapsed=result.diagnostics['elapsed_s']
                if result.status=='already_at_home':
                    # The adapter checked the entire route and the current
                    # stop window was checked above; no movement was executed.
                    self.maneuver_report.update(construction_status=result.status,construction_time_s=elapsed,
                        candidate_count=0,start=list(start4))
                    if time.monotonic()>=self.pre_execution_deadline:raise ValueError('pre_execution_timeout')
                    if not self.ready() or not self.maneuver_stopped():raise ValueError('ground_maneuver_start_changed')
                    if (math.dist(position(self.odom),position(endpoint))>min(.04,self.endpoint_tolerance) or
                            abs(math.remainder(self.maneuver_yaw(self.odom)-self.maneuver_yaw(endpoint),2*math.pi))>.1):
                        raise ValueError('ground_home_observation_outside_tolerance')
                    self.transition('DONE','already_home');return
            elif self.mode=='local':
                result=self.construct_route_obligations(start4,
                    [(*position(p),self.maneuver_yaw(p)) for p in self.pre_execution_path],
                    offset=limits['control_offset'],limits=self.maneuver_limits,valid_pose=lambda _:True,
                    measured_speed=self.odom_motion[0],measured_yaw_rate=self.odom_motion[1])
                self.maneuver_report['ordered_route']=result.diagnostics
                self.maneuver_candidate_obligations=list(result.candidate_obligations)
                elapsed=result.diagnostics['elapsed_s']
                if result.status=='already_at_endpoint':
                    self.maneuver_report.update(construction_status=result.status,construction_time_s=elapsed,
                        candidate_count=0,start=list(start4))
                    if time.monotonic()>=self.pre_execution_deadline:raise ValueError('pre_execution_timeout')
                    if not self.ready() or not self.maneuver_stopped():raise ValueError('ground_maneuver_start_changed')
                    if (math.dist(position(self.odom),position(endpoint))>min(.04,self.endpoint_tolerance) or
                            abs(math.remainder(self.maneuver_yaw(self.odom)-self.maneuver_yaw(endpoint),2*math.pi))>.1):
                        raise ValueError('ground_endpoint_observation_outside_tolerance')
                    self.transition('DONE','no_motion');return
            else:
                result=self.construct_maneuvers(start,[(*position(endpoint)[:2],self.maneuver_yaw(endpoint))],
                    offset=limits['control_offset'],limits=self.maneuver_limits,valid_pose=lambda _:True)
                elapsed=result.elapsed_s
            # Pure geometric enumeration has no map claim. Every candidate
            # below must pass the real service before becoming a command.
            self.maneuver_candidates=list(result.candidates);self.maneuver_index=0
            self.maneuver_base_z=result.base_z if self.mode in {'homing','local'} else start4[2]
            self.maneuver_report.update(construction_status=result.status,
                construction_time_s=elapsed,candidate_count=len(result.candidates),
                start=list(start4))
            reason=self.maneuver_budget_error()
            if reason:raise ValueError(reason)
            self.select_ground_maneuver(epoch)
        except Exception as error:self.fail('ground_maneuver_construction:'+str(error))

    def select_ground_maneuver(self,epoch):
        if self.maneuver_index>=len(self.maneuver_candidates):
            self.fail('ground_maneuver_no_valid_candidate');return
        candidate=self.maneuver_candidates[self.maneuver_index]
        obligations=(self.maneuver_candidate_obligations[self.maneuver_index]
            if self.maneuver_candidate_obligations else None)
        try:
            stamped,self.ground_maneuver=self.encode_maneuver(candidate,z=self.maneuver_base_z,
                frame_id=self.frame,limits=self.maneuver_limits,
                max_duration=self.maneuver_limits.max_duration,
                stamp=self.get_clock().now().to_msg(),
                pass_through_distances=obligations.pass_through_distances if obligations else None)
            if not self.ground_maneuver.segments:raise ValueError('empty_maneuver')
        except Exception as error:self.fail('ground_maneuver_encoding:'+str(error));return
        self.commanded=[point.pose for point in stamped]
        self.progress=PathProgress([position(p) for p in self.commanded])
        self.route_generation=0;self.route_token+=1;self.ahead_schedule=None;self.feedback={}
        self.follower_tracks_route=False;self.validation_pending=False;self.validated_revision=0
        approved_duration = float(self.ground_maneuver.max_duration)
        # Contact dynamics and observed-stop boundaries can take longer than
        # this kinematic profile. Keep the actual selected primitives in the
        # report so execution timing can be traced without reconstructing it.
        self.maneuver_report.update(candidate_index=self.maneuver_index,
            reference_duration_s=candidate.duration, approved_duration_s=approved_duration,
            reference_slack_s=approved_duration-candidate.duration,
            reference_duration_semantics='kinematic_profile_only_not_completion_bound',
            control_offset=candidate.control_offset,
            primitives=[asdict(primitive) for primitive in candidate.primitives],
            base_arc_length_m=candidate.path_length, reference_points=len(self.commanded),
            directions=list(candidate.directions))
        if obligations is not None:
            self.maneuver_report['route_obligations']=asdict(obligations)
        self.transition('VALIDATING','ground_maneuver_candidate_validation')
        self.deadline=self.pre_execution_deadline;self.check_path(epoch,before_execution=True)

    def check_path(self,epoch,before_execution=False):
        if before_execution and time.monotonic()>self.pre_execution_deadline:
            self.fail('pre_execution_timeout');return
        if not self.validate.service_is_ready():self.fail('path_validation_unavailable');return
        request=ValidatePath.Request();request.header.frame_id=self.frame;request.path=self.commanded
        request.bound_mode=self.bound_mode
        if before_execution and self.pre_execution_observation:
            # Only the inserted current pose is an observation. Native ground
            # validation still checks its support, collision and slope; every
            # retained planner segment keeps the ordinary validation contract.
            request.first_pose_is_observation=True
        if not before_execution and self.progress is not None and self.odom is not None:
            request.path=[]
            request.first_pose_is_observation=True
            for xyz in self.progress.remaining(position(self.odom)):
                pose=type(self.odom)();pose.position.x,pose.position.y,pose.position.z=xyz;pose.orientation.w=1.
                request.path.append(pose)
        self.validation_pending=True
        self.validation_deadline=self.pre_execution_deadline if before_execution else time.monotonic()+self.request_timeout
        future=self.validate.call_async(request)
        self.validation_future=future
        token=self.route_token
        future.add_done_callback(lambda done:self.checked(done,epoch,before_execution,token))

    def checked(self,future,epoch,before_execution,token):
        if epoch!=self.epoch or token!=self.route_token:return
        self.validation_pending=False
        self.validation_future=None
        try:response=future.result()
        except Exception as error:self.fail('path_validation_error:'+str(error));return
        if not response.valid:
            if before_execution and self.ground_maneuver is not None:
                self.maneuver_report['candidate_rejections'].append({'candidate':self.maneuver_index,
                    'reason':response.reason,'map_revision':response.map_revision})
                self.maneuver_index+=1;self.select_ground_maneuver(epoch);return
            self.fail('path_invalidated:'+response.reason);return
        self.validated_revision=response.map_revision
        if before_execution:self.dispatch(epoch)

    def dispatch(self,epoch):
        path=self.commanded
        if not (self.search_ready() if self.mode=='search' else self.ready()):self.fail('odometry_stale_before_execution');return
        if time.monotonic()>self.pre_execution_deadline:self.fail('pre_execution_timeout');return
        if self.ground_maneuver is not None:
            yaw_difference=abs(math.remainder(self.maneuver_yaw(self.odom)-self.maneuver_yaw(path[0]),2*math.pi))
            if (not self.maneuver_stopped() or math.dist(position(path[0]),position(self.odom))>.02
                    or yaw_difference>.03):self.fail('ground_maneuver_start_changed');return
            reason=self.maneuver_budget_error()
            if reason:self.fail(reason);return
        if not self.preview:
            # The explicit maneuver already checked its own frozen start.
            # A new map may require revalidation, never ordinary reconnect's
            # pose rewrite while the signed controls still describe old poses.
            moved=(self.ground_maneuver is None and
                math.dist(position(path[0]),position(self.odom))>self.endpoint_tolerance)
            if self.manual_path and moved:self.fail('planned_start_changed_before_execution');return
            if not self.manual_path and (moved or self.map_revision>self.validated_revision):
                # Keep the existing small-drift tolerance; the follower checks
                # its fresh connector independently. Larger drift is reconnected
                # and validated here, rather than accepted by widening a bound.
                if self.pre_execution_rechecks>=3:self.fail('pre_execution_recheck_limit');return
                self.pre_execution_rechecks+=1
                if moved:
                    try:self.commanded=reconnect_path(self.pre_execution_path,self.odom)
                    except ValueError as error:self.fail(str(error));return
                    self.progress=PathProgress([position(p) for p in self.commanded])
                    self.pre_execution_observation=True;self.route_token+=1
                self.transition('VALIDATING','reconnect_before_execution' if moved else 'map_changed_before_execution')
                self.check_path(epoch,before_execution=True);return
        message=Path();message.header.frame_id=self.frame;message.header.stamp=self.get_clock().now().to_msg()
        for pose in path:
            stamped=PoseStamped();stamped.header=message.header;stamped.pose=pose;message.poses.append(stamped)
        self.path_pub.publish(message)
        if self.preview:
            self.transition('PREVIEW','search_path_ready' if self.mode=='search' else 'passing_gate_path_ready' if self.mode=='passing_gate' else 'path_ready')
            if self.mode=='search':self.complete_search(True)
            if self.mode=='passing_gate':self.complete_gate(True)
            return
        if not self.follower.server_is_ready():self.fail('executor_unavailable');return
        self.target=position(self.pre_execution_path[-1] if self.ground_maneuver is not None else path[-1])
        goal=PathFollower.Goal();goal.path=message.poses;goal.bound_mode=self.bound_mode
        if self.pre_execution_observation and self.ground_maneuver is None:
            # Preflight and command_path describe the connector checked here.
            # The follower must reconnect the complete original response from
            # its own latest observation, not prepend a second observed pose
            # to our connector or repeat the half-plane rule on a trimmed path.
            goal.path=[]
            for pose in self.pre_execution_path:
                stamped=PoseStamped();stamped.header=message.header;stamped.pose=pose
                goal.path.append(stamped)
        goal.manual_path=self.manual_path;goal.vel_max=self.vel_max
        if self.ground_maneuver is not None:
            goal.execution_mode=PathFollower.Goal.GROUND_MANEUVER
            goal.maneuver=self.ground_maneuver;goal.vel_max=0.
            goal.pre_execution_timeout=max(0.,self.pre_execution_deadline-time.monotonic())
            self.maneuver_report.update(committed=True,validated_revision=self.validated_revision)
        self.pending_goal=True;self.pending_goal_epoch=epoch;self.transition('SENDING','action_requested')
        self.deadline=self.pre_execution_deadline
        future=self.follower.send_goal_async(goal,feedback_callback=self.on_feedback)
        future.add_done_callback(lambda done:self.accepted(done,epoch))

    def accepted(self,future,epoch):
        owns_pending=self.pending_goal and self.pending_goal_epoch==epoch
        if owns_pending:self.pending_goal=False;self.pending_goal_epoch=None
        try:handle=future.result()
        except Exception as error:
            if not owns_pending:return
            if self.landing.latched:self.landing_motion_finished(False);return
            if epoch==self.epoch:self.fail('goal_transport_error:'+str(error))
            else:self.transition(self.after_stop,self.stop_reason)
            return
        if not handle.accepted:
            if not owns_pending:return
            if self.landing.latched:self.landing_motion_finished(True);return
            self.clear_command();self.auto=False;self.pending_trigger=None;self.pending_homing=None;self.pending_global=None
            self.pending_initialization=None;self.initialization_resume=None
            self.reject_initialization_inbox()
            self.clear_search_queue()
            self.clear_waypoint_queue()
            self.reject_gate_inbox('executor_rejected')
            if epoch==self.epoch:self.complete_search(False)
            if epoch==self.epoch:self.complete_waypoint(False,'executor_rejected')
            if epoch==self.epoch:self.complete_gate(False,'executor_rejected')
            if epoch==self.epoch:self.complete_manual(False)
            self.transition('ERROR' if epoch==self.epoch else self.after_stop,'executor_rejected' if epoch==self.epoch else self.stop_reason)
            return
        if epoch==self.epoch and self.mode=='local':
            self.record_latency(self.handoff_latencies,self.local_handoff_started,'local_dispatch')
        goal_id=tuple(int(v) for v in handle.goal_id.uuid)
        if not owns_pending:
            # A delayed acceptance must not steal a newer task's goal slot.
            handle.cancel_goal_async();return
        self.goal=handle
        handle.get_result_async().add_done_callback(lambda result:self.finished(result,epoch,goal_id))
        if epoch!=self.epoch:
            if self.landing.latched:
                try:handle.cancel_goal_async()
                except Exception:self.landing_motion_finished(False)
            else:handle.cancel_goal_async()
            return
        if self.manual_path:
            # The follower tracks the ordered manual trajectory, including
            # turns in place and the initialization's overlapping up/down
            # legs. Its live map checks own the remaining executable route.
            self.progress=None;self.follower_tracks_route=True
        self.transition('EXECUTING',self.mode+'_executing');self.deadline=time.monotonic()+self.execution_timeout
        if self.ground_maneuver is not None:
            self.maneuver_started_ros=self.get_clock().now().nanoseconds*1e-9
        # execution_status may arrive before the action acceptance response.
        self.transfer_path_validation()
        if self.mode=='search':self.complete_search(True)
        if self.mode in {'waypoint','waypoint_direct'}:self.complete_waypoint(True)
        if self.mode=='passing_gate':self.complete_gate(True)
        if self.mode in {'rotate_180','initialization'}:self.complete_manual(True)

    def on_feedback(self,message):
        if not self.goal or list(message.goal_id.uuid)!=list(self.goal.goal_id.uuid):return
        if self.state not in {'SENDING','EXECUTING'}:return
        feedback=message.feedback
        self.feedback={'remaining_waypoints':feedback.remaining_waypoints,'dist_to_goal':feedback.dist_to_goal,
                       'estimated_time_remaining':feedback.estimated_time_remaining}
        self.feedback_received=time.monotonic()

    def timing_start(self):
        return {'wall':time.monotonic(),'ros':self.get_clock().now().nanoseconds*1.e-9,
                'simulated':bool(self.get_parameter('use_sim_time').value)}

    def record_latency(self,history,started,stage):
        if started is None:return None
        ended=self.timing_start()
        if ended['simulated']!=started['simulated']:return None
        wall=ended['wall']-started['wall'];ros=ended['ros']-started['ros']
        duration=ros if started['simulated'] else wall
        # Clock regressions are handled by the motion freshness checks. They
        # cannot create a negative lead-time estimate in this diagnostics path.
        if not math.isfinite(duration) or duration<0:return None
        sample={'stage':stage,'wall_s':wall,'ros_s':ros,'trajectory_s':duration}
        history.append(sample);del history[:-8]
        return sample

    def effective_lead_time(self):
        if not self.adaptive_lead or self.lead_time<=0:return self.lead_time
        # Keep the configured lead as dispatch allowance. Use the maximum of
        # the last eight observed RPC/dispatch delays, not a tuned flight speed
        # or an extension of the current action's completion time.
        return self.lead_time+max((x['trajectory_s'] for x in self.planning_latencies),default=0.)+max(
            (x['trajectory_s'] for x in self.handoff_latencies),default=0.)

    def maybe_plan_ahead(self):
        # A real observation during travel may start a speculative plan. Its
        # failure never cancels the trajectory already being executed.
        eta=self.feedback.get('estimated_time_remaining',math.inf)
        if (self.state!='EXECUTING' or self.lead_time<=0 or not self.auto or self.mode!='local' or not self.goal or
                self.pending_trigger is not None or self.pending_homing is not None or self.pending_initialization is not None or
                self.ahead is not None or
                self.ahead_home_required or self.ahead_global_required or self.cycles+1>=self.max_cycles or
                self.observation_revision<=self.plan_observation or
                time.monotonic()-self.feedback_received>.5 or not math.isfinite(eta) or eta<=0):return
        # Root at the follower's actual smoothed endpoint, as upstream's
        # current_path_.back(), and bind the status to this action/route.
        route=self.execution_route
        # Humble exposes uint8 arrays as NumPy scalars. Diagnostics and the
        # schedule key must contain native ints for JSON status publication.
        goal_id=[int(value) for value in self.goal.goal_id.uuid]
        if (not route or route['goal_id']!=goal_id or
                route['generation']!=self.route_generation):return
        # Generation zero is reused by every new Action. Bind its attempt
        # budget to the task epoch and UUID as well, including delayed replies.
        key=(self.epoch,tuple(goal_id),self.route_generation)
        if self.ahead_schedule is None or self.ahead_schedule['key']!=key:
            self.ahead_schedule={'key':key,'attempts':0,'retry_after_observation':None}
        schedule=self.ahead_schedule
        lead=self.effective_lead_time()
        if schedule['attempts']:
            # A missing-map/root refusal or an explicitly rejected speculative
            # splice may reopen one attempt. Require an observation after the
            # reply and the ordinary lead window; never retry the old path.
            if (not self.early_start or schedule['attempts']>=2 or
                    schedule['retry_after_observation'] is None or
                    self.observation_revision<=schedule['retry_after_observation'] or eta>lead):return
        elif not self.early_start and eta>lead:return
        if not self.ahead_planner.service_is_ready() or not self.continue_client.service_is_ready():
            if self.reason!='plan_ahead_service_unavailable':self.transition('EXECUTING','plan_ahead_service_unavailable')
            return
        request=PlanAhead.Request();request.header.frame_id=self.frame
        request.bound_mode=self.bound_mode
        request.header.stamp=self.get_clock().now().to_msg()
        request.root_pose=copy.deepcopy(self.commanded[-1]);request.arrival_time=float(eta)
        endpoint=route['endpoint']
        request.root_pose.position.x,request.root_pose.position.y,request.root_pose.position.z=endpoint[:3]
        request.root_pose.orientation.x=0.;request.root_pose.orientation.y=0.
        request.root_pose.orientation.z=math.sin(endpoint[3]/2);request.root_pose.orientation.w=math.cos(endpoint[3]/2)
        schedule['attempts']+=1;schedule['retry_after_observation']=None
        candidate={'epoch':self.epoch,'generation':self.route_generation,'goal_id':goal_id,'root':request.root_pose,
                   'schedule':schedule,'attempt':schedule['attempts'],
                   'bound_mode':self.bound_mode,
                   'observation':self.observation_revision,'stage':'planning',
                   'deadline':time.monotonic()+self.request_timeout,'requested_at':time.monotonic()}
        candidate['timing']=self.timing_start()
        candidate['event']={'generation':self.route_generation,'goal_id':goal_id,'attempt':schedule['attempts'],
                            'trigger':'retry' if schedule['attempts']>1 else 'early' if self.early_start else 'lead',
                            'retry_reason':schedule.pop('retry_reason',None),
                            'eta_s':eta,'lead_s':lead,'requested_ros_s':candidate['timing']['ros'],
                            'observation_revision':self.observation_revision,'outcome':'planning'}
        self.ahead_events.append(candidate['event']);del self.ahead_events[:-50]
        self.ahead=candidate
        self.transition('EXECUTING','plan_ahead_requested')
        try:self.ahead_planner.call_async(request).add_done_callback(lambda f:self.ahead_planned(f,candidate))
        except Exception as error:
            candidate['event']['outcome']='error:'+str(error)
            self.ahead=None;self.transition('EXECUTING','plan_ahead_skipped:'+str(error))

    def ahead_planned(self,future,candidate):
        timing=(self.record_latency(self.planning_latencies,candidate['timing'],'ahead')
                if candidate['epoch']==self.epoch else None)
        candidate['event'].update(planning_latency=timing,response_ros_s=self.timing_start()['ros'])
        if (candidate is not self.ahead or candidate['epoch']!=self.epoch or candidate['generation']!=self.route_generation or
                self.state!='EXECUTING' or not self.goal or candidate['goal_id']!=list(self.goal.goal_id.uuid)):
            candidate['event']['outcome']='stale_planning_reply';return
        try:
            response=future.result()
            candidate['event'].update(planner_reason=response.reason,planner_map_revision=response.map_revision)
            if response.homing_required:self.ahead_home_required=True
            if getattr(response,'global_required',False):
                # This is a decision about the next task, not a continuation
                # from the speculative root. Keep the current action intact.
                candidate['event']['outcome']='global_required'
                self.ahead_global_required=True;self.ahead=None
                self.transition('EXECUTING','plan_ahead_global_required');return
            if not response.success:
                candidate['event']['outcome']='skipped:'+response.reason
                if (self.early_start and candidate['attempt']==1 and candidate['schedule'] is self.ahead_schedule and
                        not response.homing_required and response.reason in {'future_root_not_free','map_missing'}):
                    self.ahead_schedule['retry_after_observation']=self.observation_revision
                    self.ahead_schedule['retry_reason']='planning:'+response.reason
                self.ahead=None;self.transition('EXECUTING','plan_ahead_skipped:'+response.reason);return
            path=list(response.path)
            if len(path)<2 or math.dist(position(path[0]),position(candidate['root']))>.001:
                raise ValueError('invalid_plan_ahead_root')
            if not self.ready():raise ValueError('odometry_stale_after_plan_ahead')
            request=ContinuePath.Request();request.header.frame_id=self.frame
            request.header.stamp=self.get_clock().now().to_msg()
            request.goal_id=self.goal.goal_id.uuid;request.expected_generation=candidate['generation']
            request.path=path;request.budget_enabled=response.budget_enabled
            request.required_time_after_root=response.required_time_after_root
            candidate['stage']='committing';candidate['handoff_timing']=self.timing_start()
            candidate['event']['outcome']='committing'
            # Follower may publish the new route before this service reply.
            # Retire old controller checks now; follower keeps checking motion.
            self.route_token+=1;self.validation_pending=False;self.follower_tracks_route=True
            candidate['deadline']=time.monotonic()+self.request_timeout
            self.continue_client.call_async(request).add_done_callback(lambda f:self.continued(f,candidate))
        except Exception as error:
            candidate['event']['outcome']='error:'+str(error)
            self.ahead=None;self.transition('EXECUTING','plan_ahead_skipped:'+str(error))

    def continued(self,future,candidate):
        timing=(self.record_latency(self.handoff_latencies,candidate.get('handoff_timing'),'continue')
                if candidate['epoch']==self.epoch else None)
        candidate['event'].update(handoff_latency=timing,ack_ros_s=self.timing_start()['ros'])
        if candidate is not self.ahead or candidate['epoch']!=self.epoch or self.state!='EXECUTING':return
        self.ahead=None
        try:
            response=future.result()
            candidate['event'].update(outcome=response.reason,accepted=response.accepted)
            if not response.accepted:
                if response.reason.startswith('continuation_time_'):self.ahead_home_required=True
                if (response.reason=='extension_invalid:path_not_free' and self.early_start and
                        candidate['attempt']==1 and candidate['schedule'] is self.ahead_schedule and
                        candidate['generation']==self.route_generation and self.goal and
                        candidate['goal_id']==list(self.goal.goal_id.uuid) and
                        not self.ahead_home_required and not self.ahead_global_required):
                    # The original trajectory remains active. A later accepted
                    # observation may justify one new plan near its endpoint;
                    # it does not make this rejected geometry admissible.
                    self.ahead_schedule['retry_after_observation']=self.observation_revision
                    self.ahead_schedule['retry_reason']=response.reason
                self.transition('EXECUTING','continuation_skipped:'+response.reason)
                self.drain_result();return
            if response.generation!=candidate['generation']+1 or len(response.path)<2:
                self.fail('invalid_continuation_acknowledgment');return
            self.commanded=list(response.path);self.target=position(self.commanded[-1])
            # After a splice the follower owns the measured cursor and map
            # revalidation. Reconstructing a second cursor from a delayed ack
            # would discard the odometry received while the vehicle moved.
            self.progress=None
            self.route_generation=response.generation;self.route_token+=1;self.ahead_schedule=None
            self.validation_pending=False;self.validated_revision=response.map_revision
            self.plan_observation=candidate['observation'];self.feedback={};self.feedback_received=0.
            self.cycles+=1;self.ahead_count+=1
            self.deadline=time.monotonic()+self.execution_timeout
            message=Path();message.header.frame_id=self.frame;message.header.stamp=self.get_clock().now().to_msg()
            for p in self.commanded:
                stamped=PoseStamped();stamped.header=message.header;stamped.pose=p;message.poses.append(stamped)
            self.path_pub.publish(message)
            self.transition('EXECUTING','continuation_committed')
            self.drain_result()
        except Exception as error:
            # A lost commit acknowledgement is uncertain: stop instead of
            # pretending the old endpoint remains authoritative.
            self.fail('continuation_ack_error:'+str(error))

    def drain_result(self):
        if self.deferred_result is not None:
            future,epoch,goal_id=self.deferred_result;self.deferred_result=None
            self.finished(future,epoch,goal_id)

    def finished(self,future,epoch,goal_id):
        owns_goal=self.goal is not None and tuple(self.goal.goal_id.uuid)==goal_id
        if not owns_goal:return
        if epoch==self.epoch and self.committing():
            # The action result and continuation response use independent
            # channels. Install the acknowledged final route before finishing.
            self.deferred_result=(future,epoch,goal_id);return
        self.goal=None
        if epoch!=self.epoch:
            if self.landing.latched:
                try:confirmed=future.result().status in {GoalStatus.STATUS_CANCELED,GoalStatus.STATUS_SUCCEEDED}
                except Exception:confirmed=False
                self.landing_motion_finished(confirmed)
            elif self.state=='STOPPING':self.transition(self.after_stop,self.stop_reason)
            return
        try:result=future.result()
        except Exception as error:self.fail('execution_transport_error:'+str(error));return
        if result.status!=GoalStatus.STATUS_SUCCEEDED or not result.result.success:
            if self.start_recovery(result,goal_id):return
            self.fail('execution_failed');return
        # Invalidate any validation response still pending for this action.
        self.epoch+=1;self.validation_pending=False;self.ahead=None;self.ahead_schedule=None
        self.completed+=1
        self.recovery_consecutive=0
        # An inserted initialization is a real Action, not an extra local
        # exploration round charged against the interrupted task's max_cycles.
        if not (self.mode=='initialization' and self.initialization_resume is not None):self.cycles+=1
        # A scan received during travel does not prove that the endpoint has
        # been observed. Require another snapshot after the action result.
        self.completed_revision=self.map_revision
        self.completed_observation_revision=self.observation_revision
        self.completed_stamp=self.get_clock().now().nanoseconds*1e-9
        self.deadline=time.monotonic()+self.request_timeout
        if self.consume_homing():return
        self.transition('WAITING_OBSERVATION','await_current_pose_and_map')

    def start_recovery(self,result,goal_id):
        if (self.path_invalidation_retries<=0 or not self.auto or self.mode!='local' or
                self.manual_path or self.preview or self.landing.latched or
                self.state!='EXECUTING' or result.status!=GoalStatus.STATUS_ABORTED or
                result.result.success):return False
        self.recovery_pending={'epoch':self.epoch,'goal_id':list(goal_id),
            'generation':self.route_generation,'action_status':int(result.status),
            'map_revision':self.map_revision,'observation_revision':self.observation_revision,
            'bound_mode':self.bound_mode}
        # Retire all geometry owned by the failed action. No plan is issued
        # until a matching frozen terminal outcome proves the stop.
        self.route_token+=1;self.ahead=None;self.ahead_schedule=None
        self.ahead_home_required=False;self.ahead_global_required=False
        if self.validation_future is not None:self.validation_future.cancel()
        self.validation_future=None;self.validation_pending=False
        self.commanded=[];self.pre_execution_path=[];self.progress=None
        self.execution_route=None;self.follower_tracks_route=False;self.target=None
        self.retry_bound_mode=None;self.feedback={};self.clear_command()
        self.deadline=time.monotonic()+self.request_timeout
        self.transition('WAITING_RECOVERY','await_failed_action_stop_evidence')
        self.confirm_recovery();return True

    def confirm_recovery(self):
        pending=self.recovery_pending
        if (pending is None or pending['epoch']!=self.epoch or
                self.state!='WAITING_RECOVERY' or self.landing.latched):return
        # Ground execution owns a ROS motion-time budget; admission, RPC and
        # cancellation retain their wall deadlines even in paused simulation.
        if (time.monotonic()>self.deadline and
                not(self.state=='EXECUTING' and self.ground_maneuver is not None)):
            self.fail('waiting_recovery_timeout');return
        outcome=self.execution_outcomes.get((tuple(pending['goal_id']),pending['generation']))
        if outcome is None:return
        # Exact reasons exclude transport errors, user cancellation, failed
        # stop confirmation and malformed validation requests.
        reasons={'path_invalidated:'+reason for reason in
                 ('path_not_free','terrain_support_missing','terrain_slope_exceeded','path_off_terrain')}
        stamp=outcome.get('stop_stamp_s');finished=outcome.get('finished_ros_s')
        revision=outcome.get('failure_map_revision');validated=outcome.get('validated_revision')
        numeric=lambda value:type(value) in (int,float) and math.isfinite(value)
        dispatched=outcome.get('dispatched');confirmation=outcome.get('confirmation')
        confirmed=(confirmation in {'action_terminal_and_stationary_odometry','topic_hold_and_stationary_odometry'}
                   if dispatched is True else confirmation=='not_dispatched' if dispatched is False else False)
        if (outcome.get('action_status')!=pending['action_status'] or outcome.get('success') is not False or
                outcome.get('reason') not in reasons or outcome.get('stop_confirmed') is not True or not confirmed or
                type(revision) is not int or revision<1 or type(validated) is not int or revision<=validated or
                not numeric(stamp) or stamp<=0 or not numeric(finished) or finished<stamp or
                finished>self.get_clock().now().nanoseconds*1e-9+.1):
            self.fail('execution_failed:'+str(outcome.get('reason','invalid_stop_evidence')));return
        if self.recovery_consecutive>=self.path_invalidation_retries:
            self.fail('path_invalidation_retry_limit');return
        self.recovery_attempts+=1;self.recovery_consecutive+=1
        event={'goal_id':pending['goal_id'],'generation':pending['generation'],
            'attempt':self.recovery_attempts,'consecutive_attempt':self.recovery_consecutive,
            'failure_map_revision':revision,'confirmed_stop_stamp_s':stamp,
            'reason':outcome['reason'],'confirmation':confirmation,'bound_mode':pending['bound_mode']}
        self.recovery_events.append(event)
        pending.update(failure_map_revision=revision,confirmed_stop_stamp_s=stamp,attempt=self.recovery_attempts)
        self.transition('WAITING_RECOVERY_OBSERVATION','await_post_stop_map_and_odometry')

    def advance_recovery(self):
        pending=self.recovery_pending
        if pending is None or pending['epoch']!=self.epoch:return
        if time.monotonic()>self.deadline:
            self.fail(self.state.lower()+'_timeout');return
        if self.state=='WAITING_RECOVERY':self.confirm_recovery()
        if self.state=='WAITING_RECOVERY_OBSERVATION':
            stamp=pending['confirmed_stop_stamp_s']
            # A revision alone can be a policy edit/republication. Require a
            # new observation with its acquisition stamp at/after the stop.
            observed=(self.map_revision>max(pending['failure_map_revision'],pending['map_revision']) and
                      self.observation_revision>pending['observation_revision'] and self.map_stamp>=stamp)
            if observed and self.ready() and self.odom_stamp>=stamp:
                self.recovery_events[-1].update(replan_map_revision=self.map_revision,
                    replan_map_stamp_s=self.map_stamp,replan_observation_revision=self.observation_revision,
                    replan_odom_stamp_s=self.odom_stamp)
                bound=pending['bound_mode'];self.recovery_pending=None
                self.epoch+=1;self.route_token+=1
                self.transition('PLANNING','path_invalidation_recovery_ready')
                if self.consume_homing():return
                self.request_plan('local',bound);return

    def stop(self,request,response):
        self.cancel('user_stop','STOPPED');response.success=True;return response

    def cancel(self,reason,terminal='ERROR'):
        self.recovery_pending=None
        # Both public stop services enter here. Internal cancellation leaves
        # the planner's retained target intact; an explicit stop clears it.
        if reason=='user_stop':
            message=Bool();message.data=True;self.stop_request_pub.publish(message)
        self.ahead_global_required=False;self.pending_trigger=None;self.pending_homing=None;self.pending_global=None
        self.pending_initialization=None;self.initialization_resume=None
        self.reject_initialization_inbox()
        self.clear_search_queue()
        self.clear_waypoint_queue()
        self.reject_gate_inbox(reason)
        if self.landing.latched and reason!='auto_landing_due':
            self.advance_landing();return
        self.epoch+=1;self.auto=False
        self.retry_bound_mode=None
        if self.pending is not None:self.pending.cancel()
        if self.validation_future is not None:self.validation_future.cancel()
        self.pending=None;self.validation_future=None;self.ahead=None;self.ahead_schedule=None;self.validation_pending=False;self.after_stop=terminal;self.stop_reason=reason
        self.complete_search(False)
        self.complete_waypoint(False,reason)
        self.complete_gate(False,reason)
        self.complete_manual(False)
        if self.home_call is not None:
            entry=self.home_call;self.home_call=None;entry['pending'].cancel()
            if not entry['future'].done():entry['future'].set_result(False)
        self.clear_command()
        if self.deferred_result is not None:
            self.deferred_result=None;self.goal=None
        if self.goal or self.pending_goal:
            self.transition('STOPPING',reason);self.deadline=time.monotonic()+self.request_timeout
            if self.goal:self.goal.cancel_goal_async()
        else:self.transition(terminal,reason)

    def fail(self,reason):self.cancel(reason,'ERROR')

    def tick(self):
        for pending,finished,deadline in list(self.geofence_calls):
            if not finished.done() and time.monotonic()>deadline:
                finished.set_result(False);pending.cancel()
        # Deadline handoff also runs from IDLE/STOPPED and takes precedence
        # over ordinary mission exhaustion. It does not require a valid map.
        if self.advance_landing():
            self.reject_initialization_inbox();self.clear_search_queue();self.clear_waypoint_queue()
            self.reject_gate_inbox('landing_in_progress');return
        self.receive_initialization_requests()
        self.receive_search_requests()
        self.receive_gate_requests()
        self.receive_waypoint_requests()
        if self.search_reply is not None and time.monotonic()>self.search_reply['deadline']:
            self.fail('search_response_timeout');return
        if self.waypoint_reply is not None and time.monotonic()>self.waypoint_reply['deadline']:
            self.fail('waypoint_response_timeout');return
        if self.gate_reply is not None and time.monotonic()>self.gate_reply['deadline']:
            self.fail('passing_gate_response_timeout');return
        if self.manual_reply is not None and time.monotonic()>self.manual_reply['deadline']:
            self.fail('manual_response_timeout');return
        if self.home_call is not None and time.monotonic()>self.home_call['deadline']:
            entry=self.home_call;self.home_call=None;entry['pending'].cancel()
            if not entry['future'].done():entry['future'].set_result(False)
        # Explicit homing has PCI priority and needs no new exploration scan.
        # consume_homing itself excludes an in-flight Action or commit ACK.
        if self.pending_homing is not None and self.consume_homing():return
        if self.pending_initialization is not None and self.consume_initialization():return
        if self.state in self.TERMINAL:
            # Previews and already-at-target routes have no Action result to
            # establish a later boundary. Errors and stops never drain a slot.
            if self.state in {'PREVIEW','DONE'}:
                if self.consume_trigger():return
                if self.consume_search():return
                if self.consume_global():return
                if self.consume_gate():return
                self.consume_waypoint()
            return
        if self.state=='EXECUTING':
            if (self.ground_maneuver is not None and self.maneuver_started_ros is not None and
                    self.get_clock().now().nanoseconds*1e-9-self.maneuver_started_ros>
                    self.ground_maneuver.max_duration+self.request_timeout):
                self.fail('ground_maneuver_execution_timeout');return
            # Diagnose from one clock sample: a mission heartbeat becoming
            # stale must not be relabelled as lost odometry by a later check.
            readiness=self.readiness_snapshot()
            reason=(readiness['mission_error'] or
                    ('odometry_lost_during_execution' if not readiness['fresh'] else
                     'map_empty_during_execution' if readiness['known_cells']<=0 else None))
            if reason:
                self.readiness_failure=readiness;self.fail(reason);return
        elif self.state!='STOPPING':
            reason=self.mission_error()
            if reason:self.fail(reason);return
        if self.state in {'WAITING_RECOVERY','WAITING_RECOVERY_OBSERVATION'}:
            self.advance_recovery();return
        if self.state=='WAITING_REPLAN':
            if not self.ready():self.fail('map_or_fresh_odometry_missing');return
            if time.monotonic()>=self.deadline:
                bound_mode=self.retry_bound_mode;self.retry_bound_mode=None
                if bound_mode is None and self.consume_trigger():return
                self.request_plan('local',bound_mode)
            return
        if self.state=='WAITING_OBSERVATION':
            resume=self.initialization_resume if self.mode=='initialization' else None
            wait_for_observation=resume is not None and (resume['auto'] or resume['retry_pending'])
            observed=(self.observation_revision>self.completed_observation_revision and
                      (self.map_stamp==0 or self.map_stamp>=self.completed_stamp))
            # A frozen map delays resumption, but must still reach the common
            # waiting_observation deadline check at the bottom of this tick.
            if (self.ready() and math.dist(position(self.odom),self.target)<=self.endpoint_tolerance and
                    (not wait_for_observation or observed)):
                # Restore before selecting ordinary queued work, so its
                # cycle/budget checks see the interrupted task's real flags.
                if self.mode=='initialization':
                    if self.restore_after_initialization():
                        # A completed no-path RPC may be waiting to retry a
                        # smaller bound. Keep that pass (including manual
                        # one-shot work) rather than restarting or losing it.
                        bound_mode=self.retry_bound_mode;self.retry_bound_mode=None
                        if bound_mode is None and self.consume_trigger():return
                        self.request_plan('local',bound_mode);return
                if self.pending_trigger is not None:
                    # Queued requests never bypass actual arrival or the new
                    # endpoint observation required for a subsequent round.
                    if self.observation_revision>self.completed_observation_revision and (self.map_stamp==0 or self.map_stamp>=self.completed_stamp):
                        if self.cycles>=self.max_cycles or self.ahead_home_required:
                            if self.landing.enabled:self.cancel('exploration_complete_waiting_for_landing','STOPPED')
                            else:self.request_plan('homing')
                        else:self.consume_trigger()
                    return
                if self.mode=='search':
                    self.search_task['observed_target_distance']=math.dist(position(self.odom),position(self.search_task['target']))
                    reason='search_target_reached' if self.search_task['exact_target'] else 'search_target_neighborhood_reached'
                    self.transition('DONE',reason);return
                if self.mode in {'waypoint','waypoint_direct'}:
                    self.waypoint_task['observed_target_distance']=math.dist(position(self.odom),position(self.waypoint_task['target'].pose))
                    reason='waypoint_reached' if self.waypoint_task['exact_target'] else 'waypoint_neighborhood_reached'
                    self.transition('DONE',reason);return
                if self.mode=='homing':self.transition('DONE','returned_home');return
                if self.mode=='passing_gate':self.transition('DONE','passing_gate_complete');return
                if self.mode=='rotate_180':self.transition('DONE','rotate_180_complete');return
                if self.mode=='initialization':
                    if not self.auto:self.transition('DONE','initialization_complete');return
                if not self.auto:self.transition('DONE','manual_path_complete');return
                # Zero-stamp snapshots remain supported for static fixtures;
                # timestamped sensor maps must be observed after completion.
                if self.observation_revision>self.completed_observation_revision and (self.map_stamp==0 or self.map_stamp>=self.completed_stamp):
                    if self.cycles>=self.max_cycles or self.ahead_home_required:
                        if self.landing.enabled:self.cancel('exploration_complete_waiting_for_landing','STOPPED')
                        else:self.request_plan('homing')
                    elif self.ahead_global_required:self.request_plan('global')
                    else:self.request_plan('local')
                    return
        if self.state=='EXECUTING':
            if self.ahead and time.monotonic()>self.ahead['deadline']:
                if self.ahead['stage']=='committing':self.fail('continuation_ack_timeout');return
                self.ahead['event']['outcome']='planning_timeout'
                self.ahead=None;self.transition('EXECUTING','plan_ahead_timeout')
            self.maybe_plan_ahead()
        if self.state in {'SENDING','EXECUTING'} and not self.follower_tracks_route:
            if self.validation_pending and time.monotonic()>self.validation_deadline:self.fail('path_validation_timeout');return
            if not self.validation_pending and self.map_revision>self.validated_revision:self.check_path(self.epoch)
        if time.monotonic()>self.deadline:
            if self.state=='STOPPING':self.transition('ERROR','cancellation_unconfirmed')
            else:
                if self.state=='PLANNING' and self.mode=='local':
                    pending=self.pending;self.pending=None
                    if pending is not None:pending.cancel()
                    if self.retry_local('local_request_timeout'):return
                self.fail(self.state.lower()+'_timeout')

def shutdown_executor(executor):
    # Humble's executor shutdown does not join its worker pool. Finish every
    # submitted callback before destroying DDS entities, then close suspended
    # service coroutines so their work-tracker scopes cannot block shutdown.
    executor._executor.shutdown(wait=True)
    errors=[]
    for task in set(executor._futures):
        try:
            if not task.done():task.cancel()
            error=task.exception()
            if error is not None:errors.append(error)
        except Exception as error:errors.append(error)
    executor.shutdown()
    # spin_once normally retrieves these exceptions. A final worker may finish
    # after the stop signal, so preserve that failure instead of losing it at GC.
    for error in errors[1:]:traceback.print_exception(type(error),error,error.__traceback__)
    if errors:raise errors[0]

def main():
    # The default signal handler closes the context while executor workers can
    # still be taking messages. Signals only request stop; cleanup owns teardown.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop_requested=False
    def request_stop(signum,frame):
        nonlocal stop_requested
        stop_requested=True
    signal.signal(signal.SIGINT,request_stop)
    signal.signal(signal.SIGTERM,request_stop)
    node=PlannerControl()
    executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node)
    try:
        while rclpy.ok() and not stop_requested:executor.spin_once(timeout_sec=.1)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        # ros2 launch may forward SIGINT after the process group already received
        # it. A second interrupt must not abort destruction halfway through.
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        signal.signal(signal.SIGTERM,signal.SIG_IGN)
        try:shutdown_executor(executor)
        finally:
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
