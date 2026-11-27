#!/usr/bin/env python3
"""Exercise queued PCI triggers through real ROS services and Actions.

Runs the source control_node.py unchanged. This deterministic protocol fixture
supplies planner/validator replies, map notices and measured-pose messages; it
does not claim real planner geometry, vehicle dynamics or sensor validation.
Delayed replies exercise the two different PlanAhead commit boundaries.
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


def require(value, message):
    if not value:
        raise AssertionError(message)


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/queued_trigger/native.json')
    parser.add_argument('--timeout', type=float, default=65.)
    args = parser.parse_args()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', '173')
    started = time.monotonic()
    deadline = started+args.timeout
    source = workspace/'src/gbplanner_local/scripts/control_node.py'
    summary = {'status': 'failed', 'scope': __doc__, 'checks': {}, 'cases': {},
               'control_source_sha256': hashlib.sha256(source.read_bytes()).hexdigest()}
    process = None
    probe = None
    initialized = False
    try:
        import rclpy
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import Odometry
        from planner_msgs.action import PathFollower
        from planner_msgs.srv import PciStop, PciTrigger, PlannerSrv, PlanAhead, ContinuePath, ValidatePath
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
            result.position.x = float(x)
            result.orientation.w = 1.
            return result

        class Probe(Node):
            def __init__(self):
                self.ns = '/queued_trigger_'+str(os.getpid())
                super().__init__('fixture', namespace=self.ns)
                self.group = ReentrantCallbackGroup()
                self.x, self.revision, self.known, self.emit_maps = 0., 1, 100, True
                self.plans, self.validations, self.actions, self.ahead_calls, self.commits = [], [], [], [], []
                self.states, self.service_clients = [], {}
                self.active = None
                self.cancel_count = 0
                self.hold_plan = self.hold_ahead = self.hold_commit = False
                self.plan_wait = self.ahead_wait = self.commit_wait = None
                self.empty_plans = 0
                self.map_pub = self.create_publisher(String, 'map_updates', durable)
                self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)
                self.route_pub = self.create_publisher(String, 'execution_status', durable)
                self.create_subscription(String, 'control_status',
                    lambda m:self.states.append(json.loads(m.data)), durable)
                self.create_service(PlannerSrv, 'planner', self.plan, callback_group=self.group)
                self.create_service(ValidatePath, 'validate_path', self.validate, callback_group=self.group)
                self.create_service(PlanAhead, 'planner_ahead', self.ahead, callback_group=self.group)
                self.create_service(ContinuePath, 'continue_path', self.continue_path, callback_group=self.group)
                self.server = ActionServer(self, PathFollower, 'path_follower',
                    execute_callback=self.execute, cancel_callback=self.cancel, callback_group=self.group)
                self.timer = self.create_timer(.03, self.publish, callback_group=self.group)

            def publish(self):
                odom = Odometry()
                odom.header.frame_id, odom.child_frame_id = 'map', 'base_link'
                odom.header.stamp = self.get_clock().now().to_msg()
                odom.pose.pose = pose(self.x)
                self.odom_pub.publish(odom)
                if self.emit_maps:
                    self.revision += 1
                    self.map_pub.publish(String(data=json.dumps({'map_revision':self.revision,
                        'known_cells':self.known, 'observation_revision':self.revision,
                        'stamp_sec':0, 'stamp_nanosec':0})))
                if self.active:
                    entry = self.active
                    self.route_pub.publish(String(data=json.dumps({'state':'EXECUTING', 'active':True,
                        'output_in_flight':False, 'output_unconfirmed':False,
                        'validates_remaining_path':True, 'validated_revision':self.revision,
                        'goal_id':[int(v) for v in entry['handle'].goal_id.uuid], 'generation':entry['generation'],
                        'endpoint':[entry['end'],0.,0.,0.]})))
                    feedback = PathFollower.Feedback()
                    feedback.remaining_waypoints = 1
                    feedback.dist_to_goal = float(abs(entry['end']-self.x))
                    feedback.estimated_time_remaining = .2
                    entry['handle'].publish_feedback(feedback)

            async def plan(self, request, response):
                entry = {'bound_mode':request.bound_mode, 'x':self.x}
                self.plans.append(entry)
                response.planning_bound_mode = request.bound_mode
                response.status = PlannerSrv.Response.K_FORWARD
                if self.empty_plans:
                    self.empty_plans -= 1
                else:
                    response.path = [pose(self.x), pose(self.x+1.)]
                if self.hold_plan:
                    self.hold_plan = False
                    self.plan_wait = Future()
                    await self.plan_wait
                return response

            def validate(self, request, response):
                self.validations.append({'bound_mode':request.bound_mode,
                    'points':len(request.path), 'observation':request.first_pose_is_observation})
                response.valid, response.reason, response.map_revision = True, 'protocol_fixture', self.revision
                return response

            async def execute(self, handle):
                request = handle.request
                entry = {'handle':handle, 'done':Future(), 'generation':0,
                         'end':request.path[-1].pose.position.x}
                self.actions.append({'id':[int(v) for v in handle.goal_id.uuid], 'bound_mode':request.bound_mode,
                                     'vel_max':request.vel_max, 'end':entry['end']})
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
                # Finish on the following timer turn, after cancel admission.
                entry = self.active
                timer = None
                def finish():
                    timer.cancel()
                    if entry and not entry['done'].done():
                        entry['done'].set_result(False)
                timer = self.create_timer(.03, finish, callback_group=self.group)
                return CancelResponse.ACCEPT

            async def ahead(self, request, response):
                self.ahead_calls.append({'bound_mode':request.bound_mode, 'root':request.root_pose.position.x})
                response.success, response.reason = True, 'protocol_fixture'
                response.map_revision = self.revision
                response.path = [copy.deepcopy(request.root_pose), pose(request.root_pose.position.x+1.)]
                if self.hold_ahead:
                    self.hold_ahead = False
                    self.ahead_wait = Future()
                    await self.ahead_wait
                return response

            async def continue_path(self, request, response):
                require(self.active is not None, 'Continuation without active fixture action')
                entry = self.active
                require(list(request.goal_id)==list(entry['handle'].goal_id.uuid), 'Wrong continuation goal')
                require(request.expected_generation==entry['generation'], 'Wrong continuation generation')
                entry['generation'] += 1
                entry['end'] = request.path[-1].position.x
                self.commits.append({'generation':entry['generation'], 'end':entry['end']})
                response.accepted, response.reason = True, 'protocol_fixture'
                response.generation, response.map_revision = entry['generation'], self.revision
                response.path = [pose(self.x), pose(entry['end'])]
                # Match the native transport ordering: route may be published
                # and its Action may finish before ContinuePath's ACK arrives.
                if self.hold_commit:
                    self.hold_commit = False
                    self.commit_wait = Future()
                    await self.commit_wait
                return response

            def until(self, predicate, label, seconds=8.):
                limit = min(deadline, time.monotonic()+seconds)
                while not predicate():
                    if process.poll() is not None:
                        raise RuntimeError('Controller exited: '+str(process.returncode))
                    if time.monotonic()>limit:
                        raise TimeoutError(label)
                    rclpy.spin_once(self, timeout_sec=.01)

            def window(self, seconds=.15):
                end = time.monotonic()+seconds
                self.until(lambda:time.monotonic()>=end, 'observation window', seconds+1.)

            def call(self, name, request, kind=Trigger):
                if name not in self.service_clients:
                    self.service_clients[name] = self.create_client(kind, name, callback_group=self.group)
                client = self.service_clients[name]
                self.until(client.service_is_ready, name+' discovery')
                pending = client.call_async(request)
                self.until(pending.done, name+' response')
                return pending.result()

            def status(self):
                return json.loads(self.call('control_state', Trigger.Request()).message)

            def state(self, expected):
                self.until(lambda:self.status()['state']==expected, expected)
                return self.status()

            def trigger(self, automatic=False, bound=3, velocity=.3, preview=False):
                request = PciTrigger.Request()
                request.set_auto, request.not_exe_path = automatic, preview
                request.bound_mode, request.vel_max = bound, float(velocity)
                require(self.call('pci_trigger', request, PciTrigger).success, 'Trigger rejected')

            def finish(self):
                require(self.active is not None, 'No fixture action to complete')
                self.until(lambda:self.status()['follower_tracks_route'], 'route authority handoff')
                entry = self.active
                self.x = entry['end']
                self.publish()
                self.window(.06)
                entry['done'].set_result(True)

            def new_action(self, count):
                self.until(lambda:len(self.actions)==count and self.status()['state']=='EXECUTING', 'action '+str(count))
                return self.status()

            def snapshot(self):
                return {'status':self.status(), 'plans':copy.deepcopy(self.plans),
                        'actions':copy.deepcopy(self.actions), 'continuations':copy.deepcopy(self.commits),
                        'cancel_requests':self.cancel_count, 'validation_count':len(self.validations)}

        probe = Probe()
        command = [sys.executable, str(source), '--ros-args', '-r', '__ns:='+probe.ns,
                   '-p', 'planner_trigger_lead_time:=0.5', '-p', 'max_cycles:=12',
                   '-p', 'request_timeout:=5.0', '-p', 'execution_timeout:=10.0']
        summary['command'] = command
        with output.with_suffix('.log').open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       env=dict(os.environ), start_new_session=True)
        probe.until(lambda:probe.odom_pub.get_subscription_count()>0, 'controller input discovery')
        probe.until(lambda:probe.status()['known_cells']>0 and
                    probe.status()['odometry_freshness']['fresh'], 'controller readiness')

        # Last trigger wins, while zero velocity preserves the prior valid setting.
        probe.trigger()
        probe.new_action(1)
        probe.trigger(True, 0, .6)
        probe.trigger(False, 2, 0.)
        queued = probe.status()
        require(queued['bound_mode']==3 and not queued['auto'] and abs(queued['vel_max']-.3)<1e-6,
                'Queued request changed the running route')
        require(queued['pending_trigger']['bound_mode']==2 and not queued['pending_trigger']['auto'] and
                abs(queued['pending_trigger']['vel_max']-.6)<1e-6, 'Latest-slot/velocity semantics changed')
        probe.emit_maps = False
        probe.window(.08)
        probe.finish()
        probe.state('WAITING_OBSERVATION')
        probe.window()
        require(len(probe.plans)==1 and len(probe.actions)==1, 'Queue bypassed new endpoint observation')
        probe.emit_maps = True
        second = probe.new_action(2)
        require(second['completed_actions']==1 and second['cycles']==1 and not second['auto'] and
                probe.actions[-1]['bound_mode']==2 and abs(probe.actions[-1]['vel_max']-.6)<1e-6,
                'Queued settings/counters not preserved')
        probe.finish()
        final = probe.state('DONE')
        require(final['completed_actions']==2 and final['cycles']==2 and probe.cancel_count==0,
                'Manual switch stopped motion or reset counters')
        summary['cases']['latest_request'] = {'queued':queued, **probe.snapshot()}

        # A late, uncommitted ahead response must not splice the old settings.
        before_actions, before_commits = len(probe.actions), len(probe.commits)
        probe.hold_ahead = True
        probe.trigger(True)
        probe.new_action(before_actions+1)
        probe.until(lambda:probe.ahead_wait is not None, 'held speculative plan')
        probe.trigger(False, 1, .5)
        probe.ahead_wait.set_result(True)
        probe.window(.2)
        require(len(probe.commits)==before_commits, 'Stale speculative plan was committed')
        probe.finish()
        probe.new_action(before_actions+2)
        probe.finish()
        probe.state('DONE')
        require(probe.cancel_count==0, 'Speculative replacement canceled an action')
        summary['cases']['uncommitted_ahead'] = probe.snapshot()

        # An already sent commit remains authoritative even if the Action
        # result arrives first. The queued request waits for the new endpoint.
        before_actions, before_commits = len(probe.actions), len(probe.commits)
        probe.hold_commit = True
        probe.trigger(True)
        probe.new_action(before_actions+1)
        probe.until(lambda:probe.commit_wait is not None, 'held continuation ACK')
        probe.trigger(False, 0, .4)
        probe.emit_maps = False
        probe.finish()
        probe.window(.15)
        deferred = probe.status()
        require(deferred['state']=='EXECUTING' and deferred['completed_actions']==0 and
                deferred['pending_trigger'] is not None, 'Result bypassed commit acknowledgment')
        probe.commit_wait.set_result(True)
        installed = probe.state('WAITING_OBSERVATION')
        require(installed['plan_ahead']['generation']==1 and installed['cycles']==2 and
                installed['completed_actions']==1 and installed['plan_ahead']['continuations']==1,
                'Committed generation or completed counts were lost')
        probe.emit_maps = True
        applied = probe.new_action(before_actions+2)
        require(applied['completed_actions']==1 and applied['cycles']==2 and
                applied['applied_trigger']['cycles']==2, 'Queued consumption reset committed counters')
        probe.finish()
        final = probe.state('DONE')
        require(final['cycles']==3 and final['completed_actions']==2 and
                len(probe.commits)==before_commits+1 and probe.cancel_count==0,
                'Committed continuation created extra requests or cancellation')
        summary['cases']['committed_ahead'] = {'before_ack':deferred, 'after_ack':installed, **probe.snapshot()}

        # Stop and map reset clear accepted work before any stale callbacks.
        for label, reset in [('stop', False), ('map_reset', True)]:
            before_actions, before_plans = len(probe.actions), len(probe.plans)
            probe.trigger()
            probe.new_action(before_actions+1)
            probe.trigger(False, 1, .7)
            if reset:
                probe.known = 0
            else:
                require(probe.call('pci_stop', PciStop.Request(), PciStop).success, 'Stop rejected')
            state = probe.state('ERROR' if reset else 'STOPPED')
            probe.window()
            require(state['pending_trigger'] is None and len(probe.plans)==before_plans+1,
                    label+' retained or executed queued work')
            summary['cases'][label] = probe.snapshot()
            probe.known = 100
            probe.until(lambda:probe.status()['known_cells']>0, 'map recovery')
            # Original setVelocity persists an accepted setting across reset,
            # even if the corresponding next-round trigger was discarded.
            probe.trigger(velocity=0., preview=True)
            retained = probe.state('PREVIEW')
            require(abs(retained['vel_max']-.7)<1e-6 and retained['pending_trigger'] is None,
                    label+' lost the previously accepted velocity setting')
            summary['cases'][label]['velocity_after_reset'] = retained['vel_max']

        # Preview completion has no Action result; it still drains the slot.
        before_actions, before_plans = len(probe.actions), len(probe.plans)
        probe.hold_plan = True
        probe.trigger(preview=True)
        probe.until(lambda:probe.plan_wait is not None, 'held preview plan')
        probe.trigger(False, 1, .5, preview=True)
        probe.plan_wait.set_result(True)
        probe.until(lambda:len(probe.plans)==before_plans+2 and probe.status()['state']=='PREVIEW',
                    'queued preview')
        require([p['bound_mode'] for p in probe.plans[-2:]]==[3,1] and
                len(probe.actions)==before_actions and probe.status()['completed_actions']==0,
                'Preview queue executed motion or reset its request')
        summary['cases']['preview'] = probe.snapshot()

        # The current three-bound pass finishes before the next request wins.
        before_actions, before_plans = len(probe.actions), len(probe.plans)
        probe.empty_plans = 3
        probe.hold_plan = True
        probe.plan_wait = None
        probe.trigger(False, 0)
        probe.until(lambda:probe.plan_wait is not None, 'held first empty plan')
        probe.trigger(False, 3, .5)
        probe.plan_wait.set_result(True)
        probe.new_action(before_actions+1)
        require([p['bound_mode'] for p in probe.plans[before_plans:]]==[0,1,2,3],
                'Queue interrupted bound retries or was lost at empty manual result')
        probe.finish()
        final = probe.state('DONE')
        require(final['cycles']==1 and final['completed_actions']==1,
                'Empty attempts changed execution counts')
        summary['cases']['empty_round'] = probe.snapshot()
        summary['checks'] = {name:True for name in summary['cases']}
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = type(error).__name__+': '+str(error)
        if probe is not None:
            summary['last_status_event'] = probe.states[-1] if probe.states else None
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
            if process.returncode != 0:
                summary['status'] = 'failed'
                summary.setdefault('error', 'Control process did not exit cleanly')
        if probe is not None:
            probe.server.destroy()
            probe.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        summary['elapsed_s'] = time.monotonic()-started
        output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'status':summary['status'], 'checks':summary['checks'],
                      'elapsed_s':summary['elapsed_s'], 'error':summary.get('error'), 'output':str(output)}))
    return 0 if summary['status']=='passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
