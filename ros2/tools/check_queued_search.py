#!/usr/bin/env python3
"""Check queued PCI searches with real ROS services and Action transport.

One source controller uses deterministic local/search/validation/follower peers.
This observes scheduling, forwarded requests and commanded poses, not search
geometry, physical motion or sensor quality. No simulator or robot is started.
"""
import argparse
import copy
import hashlib
import json
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
        from geometry_msgs.msg import Pose, PoseStamped
        from nav_msgs.msg import Odometry
        from planner_msgs.action import PathFollower
        from planner_msgs.srv import (ContinuePath, PciHomingTrigger, PciSearch, PciStop,
                                      PciTrigger, PlanAhead, PlannerHoming, PlannerSearch,
                                      PlannerSrv, ValidatePath)
        from rclpy.action import ActionServer, CancelResponse
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.task import Future
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        rclpy.init(args=[])
        initialized = True
        durable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        def pose(x):
            value = Pose()
            value.position.x, value.orientation.w = float(x), 1.
            return value

        def pose_data(value):
            return [value.position.x, value.position.y, value.position.z,
                    value.orientation.x, value.orientation.y, value.orientation.z, value.orientation.w]

        class Probe(Node):
            def __init__(self):
                self.ns = '/queued_search_'+str(os.getpid())
                super().__init__('fixture', namespace=self.ns)
                self.group = ReentrantCallbackGroup()
                self.x, self.revision, self.emit_maps = 0., 1, True
                self.plans, self.searches, self.homes, self.actions = [], [], [], []
                self.states, self.commits = [], []
                self.rpc_clients, self.active = {}, None
                self.overlaps = self.cancels = self.heartbeats = 0
                self.allow_ahead = self.hold_commit = False
                self.commit_wait, self.selection = None, None
                self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)
                self.map_pub = self.create_publisher(String, 'map_updates', durable)
                self.route_pub = self.create_publisher(String, 'execution_status', durable)
                self.source_pub = self.create_publisher(PoseStamped, 'search_source', durable)
                self.target_pub = self.create_publisher(PoseStamped, 'search_target', durable)
                self.create_subscription(String, 'control_status', lambda m:self.states.append(json.loads(m.data)), durable)
                self.create_service(PlannerSrv, 'planner', self.plan, callback_group=self.group)
                self.create_service(PlannerSearch, 'planner_search', self.search_plan, callback_group=self.group)
                self.create_service(PlannerHoming, 'exploration_homing', self.home, callback_group=self.group)
                self.create_service(ValidatePath, 'validate_path', self.validate, callback_group=self.group)
                self.create_service(PlanAhead, 'planner_ahead', self.ahead, callback_group=self.group)
                self.create_service(ContinuePath, 'continue_path', self.continue_path, callback_group=self.group)
                self.server = ActionServer(self, PathFollower, 'path_follower', execute_callback=self.execute,
                                           cancel_callback=self.cancel, callback_group=self.group)
                self.timer = self.create_timer(.03, self.publish, callback_group=self.group)

            def publish(self):
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                message.header.stamp = self.get_clock().now().to_msg()
                message.pose.pose = pose(self.x)
                self.odom_pub.publish(message)
                if self.emit_maps:
                    self.revision += 1
                # Republishing the same identity during a freeze is not a
                # new endpoint observation; odometry remains fresh.
                self.map_pub.publish(String(data=json.dumps({'map_revision':self.revision,
                    'observation_revision':self.revision, 'known_cells':100, 'stamp_sec':0, 'stamp_nanosec':0})))
                if self.selection is not None:
                    for publisher, x in zip((self.source_pub, self.target_pub), self.selection):
                        selected = PoseStamped()
                        selected.header, selected.pose = copy.deepcopy(message.header), pose(x)
                        publisher.publish(selected)
                if self.active is not None:
                    entry = self.active
                    self.route_pub.publish(String(data=json.dumps({'state':'EXECUTING', 'active':True,
                        'output_in_flight':False, 'output_unconfirmed':False,
                        'validates_remaining_path':True, 'validated_revision':self.revision,
                        'goal_id':[int(v) for v in entry['handle'].goal_id.uuid],
                        'generation':entry['generation'], 'endpoint':[entry['end'],0.,0.,0.]})))
                    feedback = PathFollower.Feedback()
                    feedback.remaining_waypoints = 1
                    feedback.dist_to_goal = float(abs(entry['end']-self.x))
                    feedback.estimated_time_remaining = .2 if self.allow_ahead else 5.
                    entry['handle'].publish_feedback(feedback)
                self.heartbeats += 1
                return message.header.stamp.sec+message.header.stamp.nanosec*1e-9

            def select(self, source_x, target_x):
                self.selection = float(source_x), float(target_x)
                before = self.heartbeats
                # The selector topics have no acknowledgement service. Send
                # three reliable samples, then verify the accepted pending
                # snapshot rather than assuming the chosen values arrived.
                self.until(lambda:self.heartbeats>=before+3, 'Search selector publication')

            def plan(self, request, response):
                self.plans.append({'bound_mode':request.bound_mode, 'x':self.x})
                response.planning_bound_mode, response.status = request.bound_mode, PlannerSrv.Response.K_FORWARD
                response.path = [pose(self.x), pose(self.x+1.)]
                return response

            def search_plan(self, request, response):
                self.searches.append({'frame':request.header.frame_id,
                    'stamp_ns':request.header.stamp.sec*1000000000+request.header.stamp.nanosec,
                    'use_current_state':request.use_current_state, 'source':pose_data(request.source),
                    'target':pose_data(request.target), 'bound_mode':request.bound_mode,
                    'active_action':self.active is not None})
                response.success = True
                response.path = [copy.deepcopy(request.source), copy.deepcopy(request.target)]
                return response

            def home(self, request, response):
                self.homes.append({'x':self.x, 'active_action':self.active is not None})
                response.path = [pose(self.x), pose(-1.)]
                return response

            def validate(self, request, response):
                response.valid, response.reason, response.map_revision = True, 'protocol_fixture', self.revision
                return response

            async def execute(self, handle):
                if self.active is not None:
                    self.overlaps += 1
                points = [p.pose.position.x for p in handle.request.path]
                entry = {'handle':handle, 'done':Future(), 'generation':0, 'end':points[-1]}
                self.actions.append({'id':[int(v) for v in handle.goal_id.uuid], 'points':points,
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
                response.path = [copy.deepcopy(request.root_pose), pose(request.root_pose.position.x+1.)]
                return response

            async def continue_path(self, request, response):
                entry = self.active
                require(entry is not None and list(request.goal_id)==list(entry['handle'].goal_id.uuid), 'Wrong continuation action')
                require(request.expected_generation==entry['generation'], 'Wrong continuation generation')
                entry['generation'] += 1
                entry['end'] = request.path[-1].position.x
                self.commits.append({'generation':entry['generation'], 'end':entry['end']})
                response.accepted, response.reason = True, 'protocol_fixture'
                response.generation, response.map_revision = entry['generation'], self.revision
                response.path = [pose(self.x), pose(entry['end'])]
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

            def call(self, name, kind, request):
                if name not in self.rpc_clients:
                    self.rpc_clients[name] = self.create_client(kind, name, callback_group=self.group)
                client = self.rpc_clients[name]
                self.until(client.service_is_ready, name+' discovery')
                future = client.call_async(request)
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

            def search(self, current=True, preview=False, bound=3, busy=True):
                request = PciSearch.Request(use_current_state=current, not_exe_path=preview, bound_mode=bound)
                request.header.frame_id, request.header.stamp = 'map', self.get_clock().now().to_msg()
                response = self.call('pci_search', PciSearch, request)
                require(response.success, 'Search request rejected')
                require(not response.path if busy else bool(response.path),
                        'Busy enqueue reply or idle completed-path reply has wrong meaning')
                return response

            def queued(self, target, source=None, bound=3, preview=False):
                status = self.status()
                slot = status['pending_search']
                require(slot is not None and slot['target']==[float(target),0.,0.] and
                        slot['source']==(None if source is None else [float(source),0.,0.]) and
                        slot['use_current_state']==(source is None) and slot['bound_mode']==bound and
                        slot['preview']==preview, 'Pending search differs from accepted selector snapshot')
                return status

            def forwarded(self, count, source_x, target_x, bound, queued=True):
                require(len(self.searches)==count, 'Wrong number of actual planner_search calls')
                call = self.searches[-1]
                require(call['frame']=='map' and call['stamp_ns']>0 and not call['use_current_state'] and
                        call['source']==pose_data(pose(source_x)) and call['target']==pose_data(pose(target_x)) and
                        call['bound_mode']==bound and not call['active_action'], 'Forwarded PlannerSearch fields or boundary differ')
                status = self.status()
                require(status['pending_search'] is None, 'Consumed search retained its pending slot')
                if queued:
                    require(status['applied_search']['source']==[float(source_x),0.,0.] and
                            status['applied_search']['target']==[float(target_x),0.,0.], 'Applied search snapshot differs from RPC')
                else:
                    require(status['applied_search'] is None and
                            status['search']['requested_source']==[float(source_x),0.,0.] and
                            status['search']['requested_target']==[float(target_x),0.,0.], 'Idle search acquired queue semantics')
                return status

            def action(self, count):
                self.until(lambda:len(self.actions)>=count and self.status()['state']=='EXECUTING', 'Action '+str(count))
                require(len(self.actions)==count, 'Unexpected extra Action')
                return self.status()

            def finish(self, outcome='success'):
                require(self.active is not None, 'No active action')
                self.until(lambda:self.status()['follower_tracks_route'], 'Follower route acknowledgement')
                entry = self.active
                if outcome=='success':
                    self.x = entry['end']
                    stamp = self.publish()
                    self.until(lambda:self.status()['odometry_freshness']['last_accepted_source_stamp_s']>=stamp,
                               'Fresh endpoint odometry')
                entry['done'].set_result(outcome)

            def stop(self):
                require(self.call('pci_stop', PciStop, PciStop.Request()).success, 'Stop rejected')
                return self.state('STOPPED')

            def snapshot(self):
                return {'status':self.status(), 'actions':copy.deepcopy(self.actions),
                        'searches':copy.deepcopy(self.searches), 'homes':copy.deepcopy(self.homes),
                        'commits':copy.deepcopy(self.commits), 'cancels':self.cancels}

        probe = Probe()
        command = [sys.executable, str(source), '--ros-args', '-r', '__ns:='+probe.ns,
                   '-p', 'robot_type:=kGround', '-p', 'planner_trigger_lead_time:=0.5',
                   '-p', 'max_cycles:=20', '-p', 'request_timeout:=2.0', '-p', 'execution_timeout:=15.0']
        summary['command'] = command
        with output.with_suffix('.log').open('x') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       env=dict(os.environ), start_new_session=True)
        probe.until(lambda:probe.odom_pub.get_subscription_count()>0 and probe.target_pub.get_subscription_count()>0 and
                    probe.source_pub.get_subscription_count()>0, 'Controller inputs discovered')
        probe.until(lambda:probe.status()['known_cells']>0 and probe.status()['odometry_freshness']['fresh'], 'Readiness')

        # Queued current-state requests do not replan or edit the active goal.
        # Their source is chosen only after arrival; target/bound are latest-wins.
        probe.trigger();probe.action(1)
        active = probe.active
        probe.select(-10.,10.);probe.search(bound=1)
        first = probe.queued(10.,bound=1)['pending_search']
        probe.select(-20.,20.);probe.search(bound=2)
        queued = probe.queued(20.,bound=2)
        require(queued['pending_search']['sequence']>first['sequence'] and queued['state']=='EXECUTING' and
                probe.active is active and active['end']==1. and len(probe.actions)==1 and not probe.searches and
                probe.actions[-1]['bound_mode']==3 and probe.cancels==0, 'Search queue modified current execution')
        probe.select(-99.,99.)
        probe.finish();probe.action(2)
        applied = probe.forwarded(1,1.,20.,2)
        require(applied['applied_search']['sequence']==queued['pending_search']['sequence'] and
                probe.actions[-1]['points']==[1.,20.] and probe.actions[-1]['bound_mode']==2,
                'Latest search did not own the dispatched route')
        probe.finish();probe.state('DONE')
        summary['cases']['latest_current_state_boundary'] = {'queued':queued, **probe.snapshot()}

        # A hypothetical source/target snapshot remains frozen while the
        # actual robot finishes its existing Action; preview creates no Action.
        probe.trigger();probe.action(3)
        probe.select(-4.,7.);probe.search(current=False,preview=True,bound=4)
        queued = probe.queued(7.,source=-4.,bound=4,preview=True)
        probe.select(-40.,70.)
        probe.finish();preview = probe.state('PREVIEW')
        probe.forwarded(2,-4.,7.,4)
        require(preview['mode']=='search' and preview['search']['preview'] and
                preview['search']['requested_source']==[-4.,0.,0.] and len(probe.actions)==3,
                'Explicit frozen-source preview changed or executed motion')
        summary['cases']['frozen_explicit_preview'] = {'queued':queued, **probe.snapshot()}

        probe.trigger(True);probe.action(4)
        probe.select(-5.,4.);probe.search();probe.queued(4.)
        require(probe.call('pci_homing_trigger',PciHomingTrigger,PciHomingTrigger.Request()).success,'Homing rejected')
        probe.finish();home = probe.action(5)
        require(home['mode']=='homing' and home['pending_search'] is not None and not home['auto'] and
                len(probe.searches)==2, 'Search bypassed homing or lost its queued task')
        probe.finish();probe.action(6);probe.forwarded(3,-1.,4.,3)
        probe.finish();probe.state('DONE')
        summary['cases']['homing_priority'] = {'homing':home, **probe.snapshot()}

        probe.trigger();probe.action(7)
        probe.select(-6.,5.);probe.search();probe.queued(5.)
        stopped = probe.stop()
        require(stopped['pending_search'] is None and len(probe.searches)==3 and len(probe.actions)==7,
                'Stop consumed or retained queued search')
        probe.trigger();probe.action(8)
        probe.select(-7.,6.);probe.search();probe.queued(6.)
        probe.finish('abort');failed = probe.state('ERROR')
        require(failed['pending_search'] is None and len(probe.searches)==3 and len(probe.actions)==8,
                'Execution error consumed or retained queued search')
        summary['cases']['stop_and_error_clear'] = {'stopped':stopped, **probe.snapshot()}

        # A committed continuation owns its endpoint before the ACK. Search
        # waits for that ACK and for the higher-priority queued local round.
        probe.allow_ahead = probe.hold_commit = True
        probe.trigger(True);probe.action(9)
        probe.until(lambda:probe.commit_wait is not None, 'Held continuation acknowledgement')
        probe.select(-8.,12.);probe.search(bound=2);probe.queued(12.,bound=2)
        probe.trigger(False,bound=1)
        probe.finish()
        probe.until(lambda:probe.status()['plan_ahead']['result_deferred'], 'Deferred action result')
        deferred = probe.status()
        require(deferred['pending_search'] is not None and deferred['pending_trigger'] is not None and
                len(probe.actions)==9 and len(probe.searches)==3, 'Search bypassed continuation acknowledgement')
        probe.allow_ahead = False
        endpoint = probe.commits[-1]['end']
        probe.commit_wait.set_result(True)
        manual = probe.action(10)
        require(manual['mode']=='local' and not manual['auto'] and manual['pending_search'] is not None and
                manual['plan_ahead']['continuations']==1 and probe.actions[-1]['points']==[endpoint,endpoint+1.] and
                len(probe.searches)==3, 'Search bypassed the queued local round or used obsolete continuation state')
        probe.finish();probe.action(11);probe.forwarded(4,endpoint+1.,12.,2)
        probe.finish();probe.state('DONE')
        summary['cases']['continuation_ack_and_local_priority'] = {'deferred':deferred,'manual':manual, **probe.snapshot()}

        # Auto exploration outranks search; a frozen observation must still
        # reach the existing wait deadline, not spin forever with a full slot.
        probe.trigger(True);probe.action(12)
        probe.select(-9.,15.);probe.search();probe.queued(15.)
        probe.emit_maps = False
        probe.until(lambda:probe.status()['map_revision']==probe.revision, 'Frozen observation acknowledged')
        probe.finish();waiting = probe.state('WAITING_OBSERVATION')
        require(waiting['pending_search'] is not None and len(probe.searches)==4, 'Search bypassed automatic observation gate')
        timeout_started = time.monotonic()
        timed_out = probe.state('ERROR')
        require(timed_out['reason']=='waiting_observation_timeout' and timed_out['pending_search'] is None and
                timed_out['odometry_freshness']['fresh'] and len(probe.searches)==4 and len(probe.actions)==12,
                'Frozen map skipped the ordinary observation timeout or executed queued search')
        summary['cases']['frozen_observation_timeout'] = {'waiting':waiting,
            'wait_s':time.monotonic()-timeout_started, **probe.snapshot()}

        # Retain the idle service's completed-path contract, not its busy ACK.
        probe.emit_maps = True
        probe.select(-3.,8.)
        response = probe.search(current=False,preview=True,bound=1,busy=False)
        probe.state('PREVIEW');probe.forwarded(5,-3.,8.,1,queued=False)
        require([pose_data(p) for p in response.path]==[pose_data(pose(-3.)),pose_data(pose(8.))] and
                len(probe.actions)==12, 'Idle search did not return its validated preview path')
        summary['cases']['idle_preview_response'] = probe.snapshot()
        require(probe.overlaps==0 and not any(x['active_action'] for x in probe.searches+probe.homes),
                'Planner calls or Actions overlapped motion boundaries')
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'], summary['traceback'] = type(error).__name__+': '+str(error), traceback.format_exc()
        if probe is not None:
            summary['last_state'] = probe.states[-1] if probe.states else None
            summary['actions_at_failure'], summary['searches_at_failure'] = probe.actions, probe.searches
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
