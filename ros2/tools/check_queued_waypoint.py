#!/usr/bin/env python3
"""Check queued waypoints through actual ROS services, TF and Action transport.

One source controller uses deterministic planner, validation and follower peers.
This checks queue ownership and forwarded poses, not planner geometry, physical
motion or sensor quality. No simulator or robot is started.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

sys.dont_write_bytecode = True


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def close(actual, expected):
    return len(actual) == len(expected) and all(abs(a-b) < 1e-8 for a, b in zip(actual, expected))


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=45.)
    args = parser.parse_args()
    output = args.output.resolve()
    require(not output.exists() and not output.with_suffix('.log').exists(), 'Use a new result filename')
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    source = workspace/'src/gbplanner_local/scripts/control_node.py'
    started = time.monotonic()
    deadline = started+args.timeout
    summary = {'status':'failed', 'scope':__doc__, 'cases':{},
               'control_source_sha256':hashlib.sha256(source.read_bytes()).hexdigest()}
    process = probe = None
    initialized = False
    try:
        import rclpy
        from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
        from nav_msgs.msg import Odometry
        from planner_msgs.action import PathFollower
        from planner_msgs.srv import (ContinuePath, PciGlobal, PciHomingTrigger, PciInitialization,
                                      PciSearch, PciStop, PciToWaypoint, PciTrigger, PlanAhead,
                                      PlannerGlobal, PlannerGoToWaypoint, PlannerHoming,
                                      PlannerSearch, PlannerSrv, ValidatePath)
        from rclpy.action import ActionServer, CancelResponse, GoalResponse
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.task import Future
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from tf2_ros import StaticTransformBroadcaster

        rclpy.init(args=[])
        initialized = True
        durable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        def pose(x, yaw=0.):
            value = Pose()
            value.position.x = float(x)
            value.orientation.z, value.orientation.w = math.sin(yaw/2.), math.cos(yaw/2.)
            return value

        def pose_data(value):
            return [value.position.x, value.position.y, value.position.z,
                    value.orientation.x, value.orientation.y, value.orientation.z, value.orientation.w]

        def yaw(value):
            q = value.orientation
            return math.atan2(2.*(q.w*q.z+q.x*q.y), 1.-2.*(q.y*q.y+q.z*q.z))

        def stamp_ns(header):
            return header.stamp.sec*1000000000+header.stamp.nanosec

        class Probe(Node):
            def __init__(self):
                self.ns = '/queued_waypoint_'+str(os.getpid())
                super().__init__('fixture', namespace=self.ns)
                self.group = ReentrantCallbackGroup()
                self.x, self.yaw, self.revision, self.emit_maps = 0., 0., 1, True
                self.plans, self.waypoints, self.globals, self.searches, self.homes = [], [], [], [], []
                self.actions, self.states, self.commits, self.validations = [], [], [], []
                self.rpc_clients, self.active = {}, None
                self.overlaps = self.cancels = self.heartbeats = 0
                self.allow_ahead = self.hold_commit = self.hold_goal = False
                self.commit_wait = self.goal_wait = self.selection = self.search_selection = None
                self.frame = 'queued_waypoint_frame_'+str(os.getpid())
                self.tf = StaticTransformBroadcaster(self)
                self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)
                self.map_pub = self.create_publisher(String, 'map_updates', durable)
                self.route_pub = self.create_publisher(String, 'execution_status', durable)
                self.selection_pub = self.create_publisher(PoseStamped, 'waypoint_selection', 10)
                self.search_pub = self.create_publisher(PoseStamped, 'search_target', durable)
                self.create_subscription(String, 'control_status', lambda m:self.states.append(json.loads(m.data)), durable)
                self.create_service(PlannerSrv, 'planner', self.plan, callback_group=self.group)
                self.create_service(PlannerGoToWaypoint, 'planner_go_to_waypoint', self.waypoint_plan, callback_group=self.group)
                self.create_service(PlannerGlobal, 'planner_global', self.global_plan, callback_group=self.group)
                self.create_service(PlannerSearch, 'planner_search', self.search_plan, callback_group=self.group)
                self.create_service(PlannerHoming, 'exploration_homing', self.home, callback_group=self.group)
                self.create_service(ValidatePath, 'validate_path', self.validate, callback_group=self.group)
                self.create_service(PlanAhead, 'planner_ahead', self.ahead, callback_group=self.group)
                self.create_service(ContinuePath, 'continue_path', self.continue_path, callback_group=self.group)
                self.server = ActionServer(self, PathFollower, 'path_follower', execute_callback=self.execute,
                    goal_callback=self.accept, cancel_callback=self.cancel, callback_group=self.group)
                self.timer = self.create_timer(.03, self.publish, callback_group=self.group)

            def publish(self):
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                message.header.stamp = self.get_clock().now().to_msg()
                message.pose.pose = pose(self.x, self.yaw)
                self.odom_pub.publish(message)
                if self.emit_maps:self.revision += 1
                self.map_pub.publish(String(data=json.dumps({'map_revision':self.revision,
                    'observation_revision':self.revision, 'known_cells':100, 'stamp_sec':0, 'stamp_nanosec':0})))
                if self.selection is not None:
                    self.selection.header.stamp = copy.deepcopy(message.header.stamp)
                    self.selection_pub.publish(self.selection)
                if self.search_selection is not None:
                    selected = PoseStamped()
                    selected.header, selected.pose = copy.deepcopy(message.header), pose(self.search_selection)
                    self.search_pub.publish(selected)
                if self.active is not None:
                    entry = self.active
                    self.route_pub.publish(String(data=json.dumps({'state':'EXECUTING', 'active':True,
                        'output_in_flight':False, 'output_unconfirmed':False,
                        'validates_remaining_path':True, 'validated_revision':self.revision,
                        'goal_id':[int(v) for v in entry['handle'].goal_id.uuid],
                        'generation':entry['generation'],
                        'endpoint':[entry['end'].position.x,0.,0.,yaw(entry['end'])]})))
                    feedback = PathFollower.Feedback()
                    feedback.remaining_waypoints = 1
                    feedback.dist_to_goal = float(abs(entry['end'].position.x-self.x))
                    feedback.estimated_time_remaining = .2 if self.allow_ahead else 5.
                    entry['handle'].publish_feedback(feedback)
                self.heartbeats += 1
                return message.header.stamp.sec+message.header.stamp.nanosec*1e-9

            def transform(self, translation):
                transform = TransformStamped()
                transform.header.frame_id, transform.child_frame_id = 'map', self.frame
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.transform.translation.x, transform.transform.rotation.w = float(translation), 1.
                self.tf.sendTransform(transform)
                # A selected pose transformed by the controller acknowledges
                # TF reception, including updates of this same test frame.
                self.select(0., frame=self.frame, expected=translation)

            def select(self, x, orientation=0., frame='map', expected=None):
                self.selection = PoseStamped()
                self.selection.header.frame_id, self.selection.pose = frame, pose(x, orientation)
                wanted = x if expected is None else expected
                self.until(lambda:self.status()['waypoint']['selected']==[float(wanted),0.,0.],
                           'Waypoint selector acknowledged')

            def plan(self, request, response):
                self.plans.append({'bound_mode':request.bound_mode, 'x':self.x})
                response.planning_bound_mode, response.status = request.bound_mode, PlannerSrv.Response.K_FORWARD
                response.path = [pose(self.x,self.yaw), pose(self.x+1.,self.yaw)]
                return response

            def waypoint_plan(self, request, response):
                self.waypoints.append({'frame':request.waypoint.header.frame_id,
                    'stamp_ns':stamp_ns(request.waypoint.header), 'target':pose_data(request.waypoint.pose),
                    'source':pose_data(pose(self.x,self.yaw)), 'bound_mode':request.bound_mode,
                    'check_collision':request.check_collision, 'active_action':self.active is not None})
                response.path = [pose(self.x,self.yaw), copy.deepcopy(request.waypoint.pose)]
                return response

            def global_plan(self, request, response):
                self.globals.append({'id':request.id, 'x':self.x, 'active_action':self.active is not None})
                response.path = [pose(self.x,self.yaw), pose(self.x+2.)]
                return response

            def search_plan(self, request, response):
                self.searches.append({'source':pose_data(request.source), 'target':pose_data(request.target),
                                      'active_action':self.active is not None})
                response.success = True
                response.path = [copy.deepcopy(request.source), copy.deepcopy(request.target)]
                return response

            def home(self, request, response):
                self.homes.append({'x':self.x, 'active_action':self.active is not None})
                response.path = [pose(self.x,self.yaw), pose(-1.)]
                return response

            def validate(self, request, response):
                self.validations.append({'path':[pose_data(p) for p in request.path]})
                response.valid, response.reason, response.map_revision = True, 'protocol_fixture', self.revision
                return response

            async def accept(self, request):
                if self.hold_goal:
                    self.hold_goal = False
                    self.goal_wait = Future()
                    await self.goal_wait
                return GoalResponse.ACCEPT

            async def execute(self, handle):
                if self.active is not None:self.overlaps += 1
                points = [pose_data(p.pose) for p in handle.request.path]
                entry = {'handle':handle, 'done':Future(), 'generation':0,
                         'end':copy.deepcopy(handle.request.path[-1].pose)}
                self.actions.append({'id':[int(v) for v in handle.goal_id.uuid], 'poses':points,
                                     'bound_mode':handle.request.bound_mode, 'manual':handle.request.manual_path})
                self.active = entry
                outcome = await entry['done']
                if outcome=='success':handle.succeed()
                elif outcome=='cancel':handle.canceled()
                else:handle.abort()
                if self.active is entry:self.active = None
                result = PathFollower.Result()
                result.success = outcome=='success'
                return result

            def cancel(self, handle):
                self.cancels += 1
                entry = self.active
                timer = None
                def complete():
                    timer.cancel()
                    if entry is not None and not entry['done'].done():entry['done'].set_result('cancel')
                timer = self.create_timer(.03, complete, callback_group=self.group)
                return CancelResponse.ACCEPT

            def ahead(self, request, response):
                response.success, response.reason, response.map_revision = True, 'protocol_fixture', self.revision
                response.path = [copy.deepcopy(request.root_pose), pose(request.root_pose.position.x+1.,yaw(request.root_pose))]
                return response

            async def continue_path(self, request, response):
                entry = self.active
                require(entry is not None and list(request.goal_id)==list(entry['handle'].goal_id.uuid), 'Wrong continuation action')
                require(request.expected_generation==entry['generation'], 'Wrong continuation generation')
                entry['generation'] += 1
                entry['end'] = copy.deepcopy(request.path[-1])
                self.commits.append({'generation':entry['generation'], 'endpoint':pose_data(entry['end'])})
                response.accepted, response.reason = True, 'protocol_fixture'
                response.generation, response.map_revision = entry['generation'], self.revision
                response.path = [pose(self.x,self.yaw), copy.deepcopy(entry['end'])]
                if self.hold_commit:
                    self.hold_commit = False
                    self.commit_wait = Future()
                    await self.commit_wait
                return response

            def until(self, predicate, label):
                while not predicate():
                    require(process.poll() is None, 'Controller exited before '+label)
                    if time.monotonic()>=deadline:raise TimeoutError(label)
                    rclpy.spin_once(self, timeout_sec=.01)

            def call_async(self, name, kind, request):
                if name not in self.rpc_clients:
                    self.rpc_clients[name] = self.create_client(kind, name, callback_group=self.group)
                client = self.rpc_clients[name]
                self.until(client.service_is_ready, name+' discovery')
                return client.call_async(request)

            def call(self, name, kind, request):
                future = self.call_async(name, kind, request)
                self.until(future.done, name+' response')
                return future.result()

            def status(self):
                return json.loads(self.call('control_state', Trigger, Trigger.Request()).message)

            def state(self, state):
                self.until(lambda:self.status()['state']==state, state)
                return self.status()

            def trigger(self, automatic=False, bound=3):
                request = PciTrigger.Request(set_auto=automatic, bound_mode=bound, vel_max=.3)
                require(self.call('pci_trigger', PciTrigger, request).success, 'Local trigger rejected')

            def direct(self, x, orientation=0., frame='map'):
                request = PciToWaypoint.Request()
                request.header.frame_id, request.waypoint = frame, pose(x,orientation)
                response = self.call('pci_to_waypoint', PciToWaypoint, request)
                require(response is not None and not response.get_fields_and_field_types(),
                        'PciToWaypoint must retain its empty response')

            def waypoint(self, busy=True):
                response = self.call('planner_control_interface/std_srvs/go_to_waypoint', Trigger, Trigger.Request())
                require(response.success and response.message==('waypoint_queued' if busy else 'waypoint_execution_started'),
                        'Waypoint reply does not distinguish queued acceptance from Action ACK')
                return response

            def queued(self, x, branch, orientation=0., bound=3):
                status = self.status()
                slot = status['pending_waypoint']
                require(slot is not None and slot['branch']==branch and slot['bound_mode']==bound and
                        slot['target']==[float(x),0.,0.] and slot['frame_id']=='map' and
                        close(slot['orientation'],pose_data(pose(x,orientation))[3:]), 'Wrong pending waypoint snapshot')
                return status

            def applied(self, queued, source_x, target_x, branch, orientation=0., source_stamp=0.):
                status = self.status()
                slot, action = status['applied_waypoint'], self.actions[-1]
                require(status['pending_waypoint'] is None and slot is not None and
                        slot['sequence']==queued['pending_waypoint']['sequence'] and slot['branch']==branch and
                        slot['source']==[float(source_x),0.,0.] and slot['source_stamp_s']>=source_stamp and
                        slot['target']==[float(target_x),0.,0.] and slot['frame_id']=='map' and
                        slot['stamp_ns']==queued['pending_waypoint']['stamp_ns'] and
                        close(slot['orientation'],pose_data(pose(target_x,orientation))[3:]),
                        'Waypoint consumption changed its frozen target or used old odometry')
                require(close(action['poses'][0][:3],[source_x,0.,0.]) and
                        close(action['poses'][-1],pose_data(pose(target_x,orientation))) and
                        action['bound_mode']==slot['bound_mode'] and action['manual']==(branch=='direct'),
                        'Action does not match the consumed waypoint branch/poses')
                require(self.validations and close(self.validations[-1]['path'][0][:3],[source_x,0.,0.]) and
                        close(self.validations[-1]['path'][-1],pose_data(pose(target_x,orientation))),
                        'Waypoint route bypassed validation or validated different endpoints')
                if branch=='global':
                    forwarded = self.waypoints[-1]
                    require(forwarded['frame']=='map' and forwarded['stamp_ns']==slot['stamp_ns'] and
                            forwarded['check_collision'] and forwarded['bound_mode']==slot['bound_mode'] and
                            close(forwarded['target'],pose_data(pose(target_x,orientation))) and
                            not forwarded['active_action'], 'Global waypoint RPC fields or boundary differ')
                return status

            def action(self, count):
                self.until(lambda:len(self.actions)>=count and self.status()['state']=='EXECUTING', 'Action '+str(count))
                require(len(self.actions)==count, 'Unexpected extra Action')
                return self.status()

            def finish(self, outcome='success'):
                require(self.active is not None, 'No active action')
                self.until(lambda:self.status()['follower_tracks_route'], 'Follower route acknowledgement')
                entry = self.active
                stamp = 0.
                if outcome=='success':
                    self.x, self.yaw = entry['end'].position.x, yaw(entry['end'])
                    stamp = self.publish()
                    self.until(lambda:self.status()['odometry_freshness']['last_accepted_source_stamp_s']>=stamp,
                               'Fresh endpoint odometry')
                entry['done'].set_result(outcome)
                return stamp

            def snapshot(self):
                return {'status':self.status(), 'actions':copy.deepcopy(self.actions),
                        'waypoint_calls':copy.deepcopy(self.waypoints), 'global_calls':copy.deepcopy(self.globals),
                        'search_calls':copy.deepcopy(self.searches), 'homing_calls':copy.deepcopy(self.homes),
                        'commits':copy.deepcopy(self.commits), 'cancels':self.cancels}

        probe = Probe()
        command = [sys.executable, str(source), '--ros-args', '-r', '__ns:='+probe.ns,
                   '-r', '/global_planner/waypoint_request:='+probe.ns+'/waypoint_selection',
                   '-p', 'robot_type:=kGround', '-p', 'planner_trigger_lead_time:=0.5',
                   '-p', 'max_cycles:=20', '-p', 'request_timeout:=2.0', '-p', 'execution_timeout:=15.0']
        summary['command'] = command
        with output.with_suffix('.log').open('x') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       env=dict(os.environ), start_new_session=True)
        probe.until(lambda:probe.odom_pub.get_subscription_count()>0 and probe.selection_pub.get_subscription_count()>0 and
                    probe.search_pub.get_subscription_count()>0, 'Controller inputs discovered')
        probe.until(lambda:probe.status()['known_cells']>0 and probe.status()['odometry_freshness']['fresh'], 'Readiness')

        # Direct -> global replaces one slot. After acceptance, both the TF
        # and the selected goal change; neither may retarget the queued task.
        probe.trigger();probe.action(1)
        active = probe.active
        probe.transform(10.)
        probe.direct(1., .2, probe.frame)
        first = probe.queued(11.,'direct',.2)['pending_waypoint']
        probe.select(2.,.4,probe.frame,12.);probe.waypoint()
        queued = probe.queued(12.,'global',.4)
        require(queued['pending_waypoint']['sequence']>first['sequence'] and
                probe.active is active and active['end'].position.x==1. and
                len(probe.actions)==1 and not probe.waypoints and probe.cancels==0,
                'Latest waypoint modified the running Action')
        probe.transform(100.)
        stamp = probe.finish();probe.action(2)
        applied = probe.applied(queued,1.,12.,'global',.4,stamp)
        require(len(probe.waypoints)==1, 'Replaced direct request was also executed')
        probe.finish();probe.state('DONE')
        summary['cases']['latest_global_tf_frozen'] = {'queued':queued,'applied':applied, **probe.snapshot()}

        # The opposite replacement must consume the direct branch without a
        # global-planner call, using the old Action's actual final odometry.
        probe.trigger(bound=2);probe.action(3)
        probe.select(50.);probe.waypoint()
        first = probe.queued(50.,'global',bound=2)['pending_waypoint']
        probe.transform(20.);probe.direct(3.,.6,probe.frame)
        queued = probe.queued(23.,'direct',.6,bound=2)
        require(queued['pending_waypoint']['sequence']>first['sequence'] and
                len(probe.waypoints)==1 and len(probe.actions)==3, 'Direct replacement started before the motion boundary')
        probe.transform(200.)
        stamp = probe.finish();probe.action(4)
        probe.applied(queued,13.,23.,'direct',.6,stamp)
        require(len(probe.waypoints)==1, 'Direct waypoint unexpectedly called the global planner')
        probe.finish();probe.state('DONE')
        summary['cases']['latest_direct_fresh_odometry'] = {'queued':queued, **probe.snapshot()}

        # Homing clears an already queued local trigger. Reissue local work
        # during homing to exercise the remaining PCI order in one chain.
        probe.trigger();probe.action(5)
        probe.direct(40.);queued = probe.queued(40.,'direct')
        global_request = PciGlobal.Request(id=7, bound_mode=3, ignore_time=True)
        require(probe.call('pci_global',PciGlobal,global_request).success, 'Global request rejected')
        probe.search_selection = 30.
        before = probe.heartbeats
        probe.until(lambda:probe.heartbeats>=before+3, 'Search selector publication')
        search_request = PciSearch.Request(use_current_state=True,bound_mode=3)
        search_request.header.frame_id, search_request.header.stamp = 'map', probe.get_clock().now().to_msg()
        reply = probe.call('pci_search',PciSearch,search_request)
        require(reply.success and not reply.path and probe.status()['pending_search']['target']==[30.,0.,0.],
                'Search was not queued with its selected target')
        probe.trigger(False)
        old_local = probe.status()['pending_trigger']
        require(probe.call('pci_initialization_trigger',PciInitialization,PciInitialization.Request()).success,
                'Initialization rejected')
        require(probe.call('pci_homing_trigger',PciHomingTrigger,PciHomingTrigger.Request()).success, 'Homing rejected')
        order = []
        homing_boundary = requeued_local = None
        for count, expected in enumerate(('homing','initialization','local','search','global','waypoint_direct'),6):
            stamp = probe.finish()
            current = probe.action(count)
            order.append(current['mode'])
            require(current['mode']==expected, 'Wrong request priority: '+str(order))
            if expected=='homing':
                require(current['pending_trigger'] is None and current['applied_trigger'] is None,
                        'Homing retained a local trigger queued before homing started')
                homing_boundary = current
                probe.trigger(False)
                requeued_local = probe.status()['pending_trigger']
                require(requeued_local is not None and requeued_local['sequence']>old_local['sequence'],
                        'New local request was not queued during homing')
            if expected!='waypoint_direct':
                require(current['pending_waypoint']['sequence']==queued['pending_waypoint']['sequence'],
                        'Higher-priority task discarded the waypoint slot')
            else:probe.applied(queued,32.,40.,'direct',source_stamp=stamp)
        probe.finish();probe.state('DONE')
        summary['cases']['original_request_priority'] = {'order':order, 'cleared_local':old_local,
            'homing_boundary':homing_boundary, 'requeued_local':requeued_local, **probe.snapshot()}

        # A continuation's changed endpoint is already committed while its ACK
        # is held. Queued work waits for the ACK and then the local round.
        probe.allow_ahead = probe.hold_commit = True
        probe.trigger(True);probe.action(12)
        probe.until(lambda:probe.commit_wait is not None, 'Held continuation acknowledgement')
        probe.select(60.);probe.waypoint();queued = probe.queued(60.,'global')
        probe.trigger(False,bound=1)
        probe.finish()
        probe.until(lambda:probe.status()['plan_ahead']['result_deferred'], 'Deferred action result')
        deferred = probe.status()
        require(deferred['pending_waypoint'] is not None and deferred['pending_trigger'] is not None and
                len(probe.actions)==12 and len(probe.waypoints)==1, 'Waypoint bypassed continuation acknowledgement')
        probe.allow_ahead = False
        endpoint = probe.commits[-1]['endpoint'][0]
        probe.commit_wait.set_result(True)
        local = probe.action(13)
        require(local['mode']=='local' and not local['auto'] and local['pending_waypoint'] is not None and
                local['plan_ahead']['continuations']==1 and probe.actions[-1]['poses'][0][0]==endpoint and
                len(probe.waypoints)==1, 'Waypoint bypassed local work after continuation ACK')
        stamp = probe.finish();probe.action(14)
        probe.applied(queued,endpoint+1.,60.,'global',source_stamp=stamp)
        probe.finish();probe.state('DONE')
        summary['cases']['continuation_ack_and_local_priority'] = {'deferred':deferred,'local':local, **probe.snapshot()}

        probe.trigger();probe.action(15)
        probe.direct(70.);probe.queued(70.,'direct')
        require(probe.call('pci_stop',PciStop,PciStop.Request()).success,'Stop rejected')
        stopped = probe.state('STOPPED')
        require(stopped['pending_waypoint'] is None and len(probe.actions)==15, 'Stop retained or consumed queued waypoint')
        probe.trigger();probe.action(16)
        probe.select(80.);probe.waypoint();probe.queued(80.,'global')
        probe.finish('abort');failed = probe.state('ERROR')
        require(failed['pending_waypoint'] is None and len(probe.waypoints)==2 and len(probe.actions)==16,
                'Execution error retained or consumed queued waypoint')
        summary['cases']['stop_and_error_clear'] = {'stopped':stopped, **probe.snapshot()}

        # Automatic exploration still owns its next observation boundary even
        # when a waypoint is waiting and the previous motion has finished.
        probe.trigger(True);probe.action(17)
        probe.direct(85.);probe.queued(85.,'direct')
        probe.emit_maps = False
        probe.until(lambda:probe.status()['map_revision']==probe.revision, 'Frozen map acknowledged')
        probe.finish();probe.state('WAITING_OBSERVATION')
        before = probe.heartbeats
        probe.until(lambda:probe.heartbeats>=before+3, 'Automatic observation boundary held')
        waiting = probe.status()
        require(waiting['state']=='WAITING_OBSERVATION' and waiting['auto'] and
                waiting['pending_waypoint'] is not None and len(probe.actions)==17 and len(probe.waypoints)==2,
                'Waypoint bypassed automatic exploration waiting for its next observation')
        require(probe.call('pci_stop',PciStop,PciStop.Request()).success,'Observation wait stop rejected')
        require(probe.state('STOPPED')['pending_waypoint'] is None,'Observation wait stop retained waypoint')
        probe.emit_maps = True
        summary['cases']['automatic_observation_priority'] = {'waiting':waiting, **probe.snapshot()}

        # Explicitly hold Action acceptance: idle Trigger must remain pending
        # until the follower accepts, then return without waiting for arrival.
        probe.select(90.)
        probe.hold_goal = True
        response = probe.call_async('planner_control_interface/std_srvs/go_to_waypoint',Trigger,Trigger.Request())
        probe.until(lambda:probe.goal_wait is not None, 'Held Action acceptance')
        sending = probe.state('SENDING')
        require(not response.done() and sending['pending_waypoint'] is None and len(probe.actions)==17,
                'Idle Trigger replied before Action acknowledgement or acquired queue semantics')
        probe.goal_wait.set_result(True)
        probe.until(response.done,'Idle Action acknowledgement response')
        reply = response.result()
        require(reply.success and reply.message=='waypoint_execution_started', 'Idle waypoint lost its original ACK contract')
        idle = probe.action(18)
        require(idle['applied_waypoint'] is None and idle['pending_waypoint'] is None and
                probe.active is not None and not probe.active['done'].done() and not probe.actions[-1]['manual'],
                'Idle waypoint reply waited for completion or changed global branch')
        probe.finish();probe.state('DONE')
        # The direct PCI still has an empty response and reports acceptance
        # through control status and the Action rather than invented fields.
        probe.direct(95.,.3);idle_direct = probe.action(19)
        require(idle_direct['pending_waypoint'] is None and idle_direct['applied_waypoint'] is None and
                probe.actions[-1]['manual'] and close(probe.actions[-1]['poses'][-1],pose_data(pose(95.,.3))),
                'Idle direct waypoint changed its response or route contract')
        probe.finish();probe.state('DONE')
        summary['cases']['idle_action_ack_contracts'] = {'sending':sending, **probe.snapshot()}
        require(probe.overlaps==0 and not any(x['active_action'] for x in probe.waypoints+probe.globals+probe.searches+probe.homes),
                'Planner requests or Actions overlapped the motion boundary')
        require(probe.cancels==1, 'Waypoint queue unexpectedly canceled an active Action')
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'], summary['traceback'] = type(error).__name__+': '+str(error), traceback.format_exc()
        if probe is not None:
            summary['last_state'] = probe.states[-1] if probe.states else None
            summary['actions_at_failure'], summary['waypoint_calls_at_failure'] = probe.actions, probe.waypoints
    finally:
        if process is not None:
            forced = False
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:process.wait(timeout=4.)
                except subprocess.TimeoutExpired:
                    forced = True;os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=2.)
            summary['cleanup'] = {'exit_code':process.returncode,'forced':forced}
            if forced or process.returncode!=0:
                summary['status'] = 'failed'
                summary.setdefault('error','Controller did not exit cleanly')
        if probe is not None:
            probe.server.destroy();probe.destroy_node()
        if initialized and rclpy.ok():rclpy.shutdown()
        summary['elapsed_s'] = time.monotonic()-started
        output.write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'status':summary['status'],'cases':list(summary['cases']),'output':str(output),
                      'elapsed_s':summary['elapsed_s'],'error':summary.get('error')}))
    return 0 if summary['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())
