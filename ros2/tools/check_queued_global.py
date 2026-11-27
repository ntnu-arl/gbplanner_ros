#!/usr/bin/env python3
"""Check queued PCI global requests with actual ROS services and Actions.

Runs the source controller with deterministic planner/follower interfaces.
Checks request priority and motion/continuation boundaries; does not validate
planner geometry, physical dynamics or real sensor observations.
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


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/queued_global/native.json')
    parser.add_argument('--timeout', type=float, default=35.)
    args = parser.parse_args()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    started = time.monotonic()
    deadline = started+args.timeout
    source = workspace/'src/gbplanner_local/scripts/control_node.py'
    summary = {'status':'failed', 'scope':__doc__, 'checks':{}, 'cases':{},
               'control_source_sha256':hashlib.sha256(source.read_bytes()).hexdigest()}
    process = probe = None
    initialized = False
    try:
        import rclpy
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import Odometry
        from planner_msgs.action import PathFollower
        from planner_msgs.srv import (PciGlobal, PlannerGlobal, PciHomingTrigger, PciStop, PciTrigger, PlannerHoming,
                                      PlannerSrv, PlanAhead, ContinuePath, ValidatePath)
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
            value.position.x = float(x)
            value.orientation.w = 1.
            return value

        class Probe(Node):
            def __init__(self):
                self.ns = '/queued_global_'+str(os.getpid())
                super().__init__('fixture', namespace=self.ns)
                self.group = ReentrantCallbackGroup()
                self.x, self.revision, self.emit_maps = 0., 1, True
                self.plans, self.homes, self.globals, self.actions, self.commits, self.states = [], [], [], [], [], []
                self.service_clients, self.active = {}, None
                self.overlapping_actions = self.cancel_count = 0
                self.allow_ahead = False
                self.hold_plan = self.hold_commit = False
                self.plan_wait = self.commit_wait = None
                self.plan_replies = 0
                self.map_pub = self.create_publisher(String, 'map_updates', durable)
                self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)
                self.route_pub = self.create_publisher(String, 'execution_status', durable)
                self.create_subscription(String, 'control_status',
                    lambda m:self.states.append(json.loads(m.data)), durable)
                self.create_service(PlannerSrv, 'planner', self.plan, callback_group=self.group)
                self.create_service(PlannerHoming, 'exploration_homing', self.home, callback_group=self.group)
                self.create_service(PlannerGlobal, 'planner_global', self.global_plan, callback_group=self.group)
                self.create_service(ValidatePath, 'validate_path', self.validate, callback_group=self.group)
                self.create_service(PlanAhead, 'planner_ahead', self.ahead, callback_group=self.group)
                self.create_service(ContinuePath, 'continue_path', self.continue_path, callback_group=self.group)
                self.server = ActionServer(self, PathFollower, 'path_follower',
                    execute_callback=self.execute, cancel_callback=self.cancel, callback_group=self.group)
                self.timer = self.create_timer(.03, self.publish, callback_group=self.group)

            def publish(self):
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                message.header.stamp = self.get_clock().now().to_msg()
                message.pose.pose = pose(self.x)
                self.odom_pub.publish(message)
                if self.emit_maps:
                    self.revision += 1
                # During a freeze, republish the same snapshot identity. This
                # gives the reader a confirmed plateau without creating a new
                # endpoint observation or relying on the final high-rate sample.
                self.map_pub.publish(String(data=json.dumps({'map_revision':self.revision,
                    'known_cells':100, 'observation_revision':self.revision,
                    'stamp_sec':0, 'stamp_nanosec':0})))
                if self.active is not None:
                    entry = self.active
                    self.route_pub.publish(String(data=json.dumps({'state':'EXECUTING', 'active':True,
                        'output_in_flight':False, 'output_unconfirmed':False,
                        'validates_remaining_path':True, 'validated_revision':self.revision,
                        'goal_id':[int(v) for v in entry['handle'].goal_id.uuid], 'generation':entry['generation'],
                        'endpoint':[entry['end'],0.,0.,0.]})))
                    feedback = PathFollower.Feedback()
                    feedback.remaining_waypoints = 1
                    feedback.dist_to_goal = float(abs(entry['end']-self.x))
                    feedback.estimated_time_remaining = .2 if self.allow_ahead else 5.
                    entry['handle'].publish_feedback(feedback)
                return message.header.stamp.sec+message.header.stamp.nanosec*1e-9

            async def plan(self, request, response):
                self.plans.append({'bound_mode':request.bound_mode, 'x':self.x})
                response.planning_bound_mode = request.bound_mode
                response.status = PlannerSrv.Response.K_FORWARD
                response.path = [pose(self.x), pose(self.x+1.)]
                if self.hold_plan:
                    self.hold_plan = False
                    self.plan_wait = Future()
                    await self.plan_wait
                self.plan_replies += 1
                return response

            def home(self, request, response):
                self.homes.append({'x':self.x, 'bound_mode':request.bound_mode,
                                   'active_action':self.active is not None})
                response.path = [pose(self.x), pose(-1.)]
                return response

            def global_plan(self, request, response):
                self.globals.append({'id':request.id, 'bound_mode':request.bound_mode,
                    'not_check_frontier':request.not_check_frontier, 'ignore_time':request.ignore_time,
                    'automatic_repositioning':request.automatic_repositioning,
                    'x':self.x, 'active_action':self.active is not None})
                require(not request.automatic_repositioning, 'Explicit global request became automatic repositioning')
                response.path = [pose(self.x), pose(self.x+2.)]
                response.homing = False
                return response

            def validate(self, request, response):
                response.valid, response.reason, response.map_revision = True, 'protocol_fixture', self.revision
                return response

            async def execute(self, handle):
                if self.active is not None:
                    self.overlapping_actions += 1
                entry = {'handle':handle, 'done':Future(), 'generation':0,
                         'end':handle.request.path[-1].pose.position.x}
                self.actions.append({'id':[int(v) for v in handle.goal_id.uuid], 'end':entry['end'],
                    'bound_mode':handle.request.bound_mode, 'vel_max':handle.request.vel_max})
                self.active = entry
                success = await entry['done']
                if success:
                    handle.succeed()
                else:
                    handle.canceled()
                if self.active is entry:
                    self.active = None
                result = PathFollower.Result()
                result.success = success
                return result

            def cancel(self, handle):
                self.cancel_count += 1
                entry = self.active
                timer = None
                def finish_cancel():
                    timer.cancel()
                    if entry is not None and not entry['done'].done():
                        entry['done'].set_result(False)
                timer = self.create_timer(.03, finish_cancel, callback_group=self.group)
                return CancelResponse.ACCEPT

            def ahead(self, request, response):
                response.success, response.reason, response.map_revision = True, 'protocol_fixture', self.revision
                response.path = [copy.deepcopy(request.root_pose), pose(request.root_pose.position.x+1.)]
                return response

            async def continue_path(self, request, response):
                entry = self.active
                require(entry is not None and list(request.goal_id)==list(entry['handle'].goal_id.uuid),
                        'Continuation does not belong to active action')
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
                    if process.poll() is not None:
                        raise RuntimeError('Controller exited: '+str(process.returncode))
                    if time.monotonic()>=deadline:
                        raise TimeoutError(label)
                    rclpy.spin_once(self, timeout_sec=.01)

            def call(self, name, request, kind=Trigger):
                if name not in self.service_clients:
                    self.service_clients[name] = self.create_client(kind, name, callback_group=self.group)
                client = self.service_clients[name]
                self.until(client.service_is_ready, name+' discovery')
                future = client.call_async(request)
                self.until(future.done, name+' response')
                return future.result()

            def status(self):
                return json.loads(self.call('control_state', Trigger.Request()).message)

            def state(self, expected):
                self.until(lambda:self.status()['state']==expected, expected)
                return self.status()

            def trigger(self, automatic=False, bound=3):
                request = PciTrigger.Request()
                request.set_auto, request.bound_mode, request.vel_max = automatic, bound, .3
                require(self.call('pci_trigger', request, PciTrigger).success, 'Local trigger rejected')

            def homing(self, preview=False, standard=False):
                if standard:
                    result = self.call('planner_control_interface/std_srvs/homing_trigger', Trigger.Request())
                else:
                    result = self.call('pci_homing_trigger', PciHomingTrigger.Request(not_exe_path=preview), PciHomingTrigger)
                require(result.success, 'Homing trigger rejected')

            def global_request(self, vertex, bound=3, preview=False, velocity=.3, not_check=False):
                request = PciGlobal.Request()
                request.id, request.bound_mode, request.vel_max = vertex, bound, float(velocity)
                request.not_exe_path, request.not_check_frontier = preview, not_check
                request.ignore_time, request.set_auto = True, False
                require(self.call('pci_global', request, PciGlobal).success, 'Global trigger rejected')

            def freeze_map(self):
                self.emit_maps = False
                self.until(lambda:self.status()['map_revision']==self.revision, 'Frozen map snapshot accepted')

            def new_action(self, count):
                self.until(lambda:len(self.actions)==count and self.status()['state']=='EXECUTING', 'Action '+str(count))
                return self.status()

            def finish(self):
                require(self.active is not None, 'No action to complete')
                self.until(lambda:self.status()['follower_tracks_route'], 'Follower route authority')
                entry = self.active
                self.x = entry['end']
                stamp = self.publish()
                self.until(lambda:self.status()['odometry_freshness']['last_accepted_source_stamp_s']>=stamp,
                           'Endpoint odometry accepted')
                entry['done'].set_result(True)

            def snapshot(self):
                return {'status':self.status(), 'local_calls':len(self.plans), 'homing_calls':copy.deepcopy(self.homes),
                        'global_calls':copy.deepcopy(self.globals),
                        'actions':copy.deepcopy(self.actions), 'commits':copy.deepcopy(self.commits),
                        'cancel_requests':self.cancel_count, 'overlapping_actions':self.overlapping_actions}

        probe = Probe()
        command = [sys.executable, str(source), '--ros-args', '-r', '__ns:='+probe.ns,
                   '-p', 'planner_trigger_lead_time:=0.5', '-p', 'max_cycles:=20',
                   '-p', 'request_timeout:=5.0', '-p', 'execution_timeout:=15.0']
        summary['command'] = command
        with output.with_suffix('.log').open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       env=dict(os.environ), start_new_session=True)
        probe.until(lambda:probe.odom_pub.get_subscription_count()>0, 'Controller discovery')
        probe.until(lambda:probe.status()['known_cells']>0 and probe.status()['odometry_freshness']['fresh'], 'Readiness')

        # Queued settings never alter the current action. The second request
        # replaces the one slot, including preview and target selection flags.
        probe.trigger()
        probe.new_action(1)
        probe.global_request(11, bound=0, preview=True, velocity=.6)
        probe.global_request(22, bound=2, velocity=0., not_check=True)
        queued = probe.status()
        slot = queued['pending_global']
        require(queued['state']=='EXECUTING' and queued['bound_mode']==3 and
                abs(queued['vel_max']-.3)<1e-6 and not slot['preview'] and
                slot['parameters']=={'id':22, 'not_check_frontier':True, 'ignore_time':True, 'bound_mode':2} and
                abs(slot['vel_max']-.6)<1e-6 and len(probe.globals)==0,
                'Latest global slot changed current route or lost parameters')
        probe.freeze_map()
        probe.finish()
        applied = probe.new_action(2)
        require(applied['mode']=='global' and not applied['auto'] and applied['pending_global'] is None and
                applied['completed_actions']==1 and applied['observation_revision']==probe.revision and
                len(probe.globals)==1 and probe.globals[-1]['id']==22 and probe.globals[-1]['bound_mode']==2 and
                probe.globals[-1]['not_check_frontier'] and abs(probe.actions[-1]['vel_max']-.6)<1e-6 and
                probe.actions[-1]['bound_mode']==2 and len(probe.plans)==1 and probe.cancel_count==0,
                'Latest global request did not start after the current action')
        probe.finish()
        probe.state('DONE')
        summary['cases']['latest_request_and_motion_boundary'] = {'queued':queued, 'applied':applied, **probe.snapshot()}

        # Auto/local work has priority. An already sent continuation keeps
        # ownership until its ACK, even when its Action result arrives first.
        probe.emit_maps = True
        before_actions, before_globals = len(probe.actions), len(probe.globals)
        probe.allow_ahead = probe.hold_commit = True
        probe.trigger(True)
        probe.new_action(before_actions+1)
        probe.until(lambda:probe.commit_wait is not None, 'Held continuation acknowledgment')
        probe.global_request(33, bound=1, velocity=.5)
        probe.freeze_map()
        probe.finish()
        probe.until(lambda:probe.status()['plan_ahead']['result_deferred'], 'Action result deferred by commit')
        deferred = probe.status()
        require(deferred['pending_global'] is not None and deferred['auto'] and
                len(probe.globals)==before_globals, 'Global request bypassed continuation ACK')
        # The manual trigger is the next run_en_ request; its local round must
        # complete before the lower-priority global slot can be consumed.
        probe.trigger(False)
        probe.allow_ahead = False
        probe.commit_wait.set_result(True)
        probe.emit_maps = True
        local = probe.new_action(before_actions+2)
        require(local['mode']=='local' and not local['auto'] and local['completed_actions']==1 and
                local['cycles']==2 and local['plan_ahead']['continuations']==1 and
                local['pending_global']['parameters']['id']==33 and len(probe.globals)==before_globals,
                'Queued local did not retain priority or acknowledged continuation counts')
        probe.freeze_map()
        probe.finish()
        applied = probe.new_action(before_actions+3)
        require(applied['mode']=='global' and applied['completed_actions']==2 and applied['cycles']==3 and
                applied['plan_ahead']['continuations']==1 and applied['applied_global']['cycles']==3 and
                len(probe.globals)==before_globals+1 and probe.globals[-1]['id']==33,
                'Global request lost priority boundary or completed-action counts')
        probe.finish()
        probe.state('DONE')
        summary['cases']['local_priority_and_committed_continuation'] = {
            'deferred':deferred, 'local_first':local, 'applied':applied, **probe.snapshot()}

        # Homing owns the next idle PCI boundary, while the global slot waits.
        before_actions, before_globals, before_homes = len(probe.actions), len(probe.globals), len(probe.homes)
        probe.trigger()
        probe.new_action(before_actions+1)
        probe.global_request(44)
        probe.homing()
        probe.finish()
        home = probe.new_action(before_actions+2)
        require(home['mode']=='homing' and home['pending_global']['parameters']['id']==44 and
                len(probe.homes)==before_homes+1 and len(probe.globals)==before_globals,
                'Global ran before queued homing or was discarded by it')
        probe.finish()
        applied = probe.new_action(before_actions+3)
        require(applied['mode']=='global' and applied['completed_actions']==2 and
                len(probe.globals)==before_globals+1 and probe.globals[-1]['id']==44,
                'Global did not follow completed homing')
        probe.finish()
        probe.state('DONE')
        summary['cases']['homing_priority'] = {'homing_first':home, 'applied':applied, **probe.snapshot()}

        # Explicit stop cancels motion and discards lower-priority queued work.
        before_actions, before_globals = len(probe.actions), len(probe.globals)
        probe.trigger()
        probe.new_action(before_actions+1)
        probe.global_request(55)
        require(probe.call('pci_stop', PciStop.Request(), PciStop).success, 'Stop rejected')
        stopped = probe.state('STOPPED')
        require(stopped['pending_global'] is None and len(probe.globals)==before_globals,
                'Stop retained or executed queued global work')
        summary['cases']['stop_clears_global'] = probe.snapshot()
        require(probe.overlapping_actions==0 and
                not any(item['active_action'] for item in probe.homes+probe.globals),
                'Queued planner request overlapped unfinished motion')
        summary['checks'] = {name:True for name in summary['cases']}
        summary['checks']['no_parallel_motion_or_global_rpc'] = True
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = type(error).__name__+': '+str(error)
        if probe is not None:
            summary['last_status_event'] = probe.states[-1] if probe.states else None
            summary['fixture_at_failure'] = {'map_revision':probe.revision, 'emit_maps':probe.emit_maps,
                'local_calls':len(probe.plans), 'homing_calls':len(probe.homes),
                'global_calls':len(probe.globals), 'actions':len(probe.actions)}
    finally:
        if process is not None:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=4.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2.)
            summary['control_exit'] = process.returncode
            if process.returncode!=0:
                summary['status'] = 'failed'
                summary.setdefault('error', 'Controller did not exit cleanly')
        if probe is not None:
            probe.server.destroy()
            probe.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        summary['elapsed_s'] = time.monotonic()-started
        output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'status':summary['status'], 'checks':summary['checks'], 'output':str(output),
                      'elapsed_s':summary['elapsed_s'], 'error':summary.get('error')}))
    return 0 if summary['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())
