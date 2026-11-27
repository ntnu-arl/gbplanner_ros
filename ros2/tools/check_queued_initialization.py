#!/usr/bin/env python3
"""Check queued initialization through real ROS services and Action transport.

Deterministic planner, validator and follower peers exercise the source PCI.
This checks request scheduling and commanded poses, not planning geometry or
physical vehicle motion. No simulator or external robot is used.
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
    parser.add_argument('--output', type=Path, default=workspace/'results/queued_initialization/native.json')
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
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import Odometry
        from planner_msgs.action import PathFollower
        from planner_msgs.srv import (ContinuePath, PciHomingTrigger, PciInitialization,
                                      PciStop, PciTrigger, PlanAhead, PlannerHoming,
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
            result = Pose()
            result.position.x, result.orientation.w = float(x), 1.
            return result

        class Probe(Node):
            def __init__(self):
                self.ns = '/queued_initialization_'+str(os.getpid())
                super().__init__('fixture', namespace=self.ns)
                self.group = ReentrantCallbackGroup()
                self.x, self.revision, self.emit_maps = 0., 1, True
                self.plans, self.homes, self.actions, self.states, self.commits = [], [], [], [], []
                self.rpc_clients, self.active = {}, None
                self.overlaps = self.cancels = 0
                self.hold_empty_plan = self.allow_ahead = self.hold_commit = False
                self.plan_wait = self.commit_wait = None
                self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)
                self.map_pub = self.create_publisher(String, 'map_updates', durable)
                self.route_pub = self.create_publisher(String, 'execution_status', durable)
                self.create_subscription(String, 'control_status', lambda m:self.states.append(json.loads(m.data)), durable)
                self.create_service(PlannerSrv, 'planner', self.plan, callback_group=self.group)
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
                self.map_pub.publish(String(data=json.dumps({'map_revision':self.revision,
                    'observation_revision':self.revision, 'known_cells':100, 'stamp_sec':0, 'stamp_nanosec':0})))
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
                return message.header.stamp.sec+message.header.stamp.nanosec*1e-9

            async def plan(self, request, response):
                self.plans.append({'bound_mode':request.bound_mode, 'x':self.x})
                response.planning_bound_mode, response.status = request.bound_mode, PlannerSrv.Response.K_FORWARD
                if self.hold_empty_plan:
                    self.hold_empty_plan = False
                    self.plan_wait = Future()
                    await self.plan_wait
                    response.path = []
                else:
                    response.path = [pose(self.x), pose(self.x+1.)]
                return response

            def home(self, request, response):
                self.homes.append({'x':self.x, 'active':self.active is not None})
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
                self.actions.append({'points':points, 'bound_mode':handle.request.bound_mode,
                                     'manual':handle.request.manual_path})
                self.active = entry
                outcome = await entry['done']
                if outcome=='success':
                    handle.succeed()
                elif outcome=='cancel':
                    handle.canceled()
                else:
                    handle.abort()
                if self.active is entry:
                    self.active = None
                result = PathFollower.Result()
                result.success = outcome=='success'
                return result

            def cancel(self, handle):
                self.cancels += 1
                entry = self.active
                timer = None
                def complete():
                    timer.cancel()
                    if entry is not None and not entry['done'].done():
                        entry['done'].set_result('cancel')
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
                    if process.poll() is not None:
                        raise RuntimeError('Controller exited '+str(process.returncode))
                    if time.monotonic()>=deadline:
                        raise TimeoutError(label)
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

            def initialize(self):
                require(self.call('pci_initialization_trigger', PciInitialization, PciInitialization.Request()).success,
                        'Busy initialization rejected')

            def concurrent_initializations(self, count):
                name = 'pci_initialization_trigger'
                if name not in self.rpc_clients:
                    self.rpc_clients[name] = self.create_client(PciInitialization, name, callback_group=self.group)
                client = self.rpc_clients[name]
                self.until(client.service_is_ready, 'Concurrent initialization discovery')
                futures = [client.call_async(PciInitialization.Request()) for _ in range(count)]
                self.until(lambda:all(future.done() for future in futures), 'Concurrent initialization replies')
                require(all(future.result().success for future in futures), 'Concurrent busy initialization rejected')

            def stop(self):
                require(self.call('pci_stop', PciStop, PciStop.Request()).success, 'Stop rejected')
                return self.state('STOPPED')

            def action(self, count):
                self.until(lambda:len(self.actions)==count and self.status()['state']=='EXECUTING', 'Action '+str(count))
                return self.status()

            def finish(self, outcome='success'):
                require(self.active is not None, 'No active action')
                self.until(lambda:self.status()['follower_tracks_route'], 'Follower owns route validation')
                entry = self.active
                if outcome=='success':
                    self.x = entry['end']
                    stamp = self.publish()
                    self.until(lambda:self.status()['odometry_freshness']['last_accepted_source_stamp_s']>=stamp,
                               'Fresh endpoint odometry')
                entry['done'].set_result(outcome)

            def snapshot(self):
                return {'status':self.status(), 'actions':copy.deepcopy(self.actions),
                        'plans':copy.deepcopy(self.plans), 'homes':copy.deepcopy(self.homes),
                        'commits':copy.deepcopy(self.commits), 'cancels':self.cancels}

        probe = Probe()
        command = [sys.executable, str(source), '--ros-args', '-r', '__ns:='+probe.ns,
                   '-p', 'robot_type:=kGround', '-p', 'init_motion.x_forward:=0.5',
                   '-p', 'planner_trigger_lead_time:=0.5', '-p', 'max_cycles:=20',
                   '-p', 'request_timeout:=5.0', '-p', 'execution_timeout:=15.0']
        summary['command'] = command
        with output.with_suffix('.log').open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       env=dict(os.environ), start_new_session=True)
        probe.until(lambda:probe.odom_pub.get_subscription_count()>0, 'Controller discovery')
        probe.until(lambda:probe.status()['known_cells']>0 and probe.status()['odometry_freshness']['fresh'], 'Readiness')

        # Concurrent reentrant service calls coalesce in the default-group
        # handoff. The one emitted route uses the completed endpoint.
        probe.trigger()
        probe.action(1)
        probe.concurrent_initializations(8)
        queued = probe.status()
        require(queued['pending_initialization']['requests']==8 and len(probe.actions)==1 and
                queued['state']=='EXECUTING', 'Initialization changed current action or failed to coalesce')
        probe.finish()
        applied = probe.action(2)
        require(probe.actions[-1]['manual'] and probe.actions[-1]['points']==[1.,1.5] and
                probe.actions[-1]['bound_mode']==3 and applied['pending_initialization'] is None,
                'Initialization did not use the fresh completed pose')
        probe.finish();probe.state('DONE')
        require(len(probe.actions)==2 and probe.cancels==0, 'Repeated pending requests executed more than once')
        summary['cases']['coalesced_motion_boundary'] = {'queued':queued, **probe.snapshot()}

        # Homing wins the same boundary, and disables the preceding automatic
        # task. Initialization remains pending until the homing action ends.
        probe.trigger(True);probe.action(3);probe.initialize()
        require(probe.call('pci_homing_trigger', PciHomingTrigger, PciHomingTrigger.Request()).success, 'Homing rejected')
        probe.finish()
        home = probe.action(4)
        require(home['mode']=='homing' and home['pending_initialization'] is not None, 'Initialization bypassed homing')
        probe.finish()
        initialized_status = probe.action(5)
        require(initialized_status['mode']=='initialization' and probe.actions[-1]['points']==[-1.,-.5] and
                not initialized_status['initialization']['resume']['auto'], 'Homing lost priority or disabled auto mode')
        probe.finish();probe.state('DONE')
        summary['cases']['homing_priority'] = probe.snapshot()

        probe.trigger();probe.action(6);probe.initialize()
        stopped = probe.stop()
        require(stopped['pending_initialization'] is None and len(probe.actions)==6, 'Stop retained initialization')
        probe.trigger();probe.action(7);probe.initialize();probe.finish('abort')
        failed = probe.state('ERROR')
        require(failed['pending_initialization'] is None and len(probe.actions)==7, 'Error retained initialization')
        summary['cases']['stop_and_error_clear'] = probe.snapshot()

        # Automatic exploration survives the inserted motion; its next route
        # still needs an endpoint observation, and init is not an extra cycle.
        probe.trigger(True, bound=1);probe.action(8);probe.initialize();probe.finish();probe.action(9)
        probe.emit_maps = False
        probe.until(lambda:probe.status()['map_revision']==probe.revision, 'Freeze endpoint observation')
        probe.finish()
        waiting = probe.state('WAITING_OBSERVATION')
        require(waiting['mode']=='initialization' and waiting['initialization']['resume']['auto'] and
                waiting['cycles']==1 and waiting['completed_actions']==2 and len(probe.actions)==9,
                'Initialization lost automatic intent, cycle count or observation gate')
        probe.emit_maps = True
        resumed = probe.action(10)
        require(resumed['auto'] and resumed['mode']=='local' and probe.actions[-1]['bound_mode']==1 and
                not probe.actions[-1]['manual'], 'Original automatic planning settings were not restored')
        probe.stop()
        summary['cases']['automatic_resume_after_observation'] = {'waiting':waiting, **probe.snapshot()}

        # A pending init does not preempt the RPC. Once that RPC reports an
        # empty path, WAITING_REPLAN is a boundary; its Relaxed pass survives.
        probe.hold_empty_plan = True
        probe.trigger(bound=0)
        probe.until(lambda:probe.plan_wait is not None, 'Held no-path planner RPC')
        probe.initialize()
        require(probe.status()['state']=='PLANNING' and len(probe.actions)==10, 'Initialization preempted planner RPC')
        probe.plan_wait.set_result(True)
        retry_init = probe.action(11)
        require(retry_init['mode']=='initialization' and retry_init['initialization']['resume']['retry_pending'] and
                retry_init['initialization']['resume']['retry_bound_mode']==1, 'No-path retry blocked or lost initialization')
        probe.finish()
        retry = probe.action(12)
        require(retry['mode']=='local' and not retry['auto'] and probe.actions[-1]['bound_mode']==1,
                'Initialization lost the remaining single-planning retry')
        probe.finish();probe.state('DONE')
        summary['cases']['no_path_retry_boundary'] = {'initialization':retry_init, **probe.snapshot()}

        # A committed continuation can change the endpoint before its ACK.
        # Initialization must wait for that ACK and use the final endpoint.
        probe.allow_ahead = probe.hold_commit = True
        probe.trigger(True);probe.action(13)
        probe.until(lambda:probe.commit_wait is not None, 'Held continuation acknowledgement')
        probe.initialize();probe.finish()
        probe.until(lambda:probe.status()['plan_ahead']['result_deferred'], 'Deferred action result')
        deferred = probe.status()
        require(deferred['pending_initialization'] is not None and len(probe.actions)==13,
                'Initialization bypassed continuation acknowledgement')
        probe.allow_ahead = False
        final_endpoint = probe.commits[-1]['end']
        probe.commit_wait.set_result(True)
        continued_init = probe.action(14)
        require(continued_init['mode']=='initialization' and continued_init['plan_ahead']['continuations']==1 and
                probe.actions[-1]['points']==[final_endpoint,final_endpoint+.5], 'Initialization used obsolete continuation endpoint')
        probe.stop()
        summary['cases']['continuation_ack_boundary'] = {'deferred':deferred, **probe.snapshot()}

        # A frozen map must not turn the initialization-resume wait into an
        # unbounded early return from tick. Odometry remains fresh throughout.
        probe.trigger(True);probe.action(15);probe.initialize();probe.finish();probe.action(16)
        probe.emit_maps = False
        probe.until(lambda:probe.status()['map_revision']==probe.revision, 'Freeze timeout observation')
        probe.finish()
        waiting = probe.state('WAITING_OBSERVATION')
        timeout_started = time.monotonic()
        timed_out = probe.state('ERROR')
        require(timed_out['reason']=='waiting_observation_timeout' and
                timed_out['initialization']['resume'] is None and timed_out['pending_initialization'] is None and
                len(probe.actions)==16 and timed_out['odometry_freshness']['fresh'],
                'Frozen observation failed to terminate through the ordinary deadline')
        summary['cases']['frozen_observation_timeout'] = {'waiting':waiting,
            'wait_s':time.monotonic()-timeout_started, **probe.snapshot()}

        # The idle service still awaits the Action ACK without holding the
        # default callback group; it must not deadlock its own tick/validator.
        probe.emit_maps = True
        probe.initialize()
        idle_init = probe.action(17)
        require(idle_init['mode']=='initialization' and probe.actions[-1]['manual'], 'Idle initialization ACK lost')
        probe.finish();probe.state('DONE')
        summary['cases']['idle_action_ack'] = probe.snapshot()
        require(probe.overlaps==0 and not any(item['active'] for item in probe.homes), 'Concurrent actions were dispatched')
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = type(error).__name__+': '+str(error)
        if probe is not None:
            summary['last_state'] = probe.states[-1] if probe.states else None
            summary['actions_at_failure'] = probe.actions
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
    print(json.dumps({'status':summary['status'], 'cases':list(summary['cases']), 'output':str(output),
                      'elapsed_s':summary['elapsed_s'], 'error':summary.get('error')}))
    return 0 if summary['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())
