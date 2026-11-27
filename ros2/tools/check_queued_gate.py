#!/usr/bin/env python3
"""Check gate scheduling with real ROS services and Action transport.

Runs the production source control_node.py in a separate process against
controlled planning, ValidatePath and PathFollower peers. This verifies queue
ownership and interface ordering, not gate geometry, flight or physical motion.
No simulator or robot is started. --prefix selects the ROS dependency overlay;
the controller under test always comes from this workspace's source tree.
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
    parser.add_argument('--prefix', type=Path, default=Path('install'),
                        help='ROS install overlay, relative to the workspace (default: install)')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=45.)
    args = parser.parse_args()
    require(args.timeout > 0., 'Timeout must be positive')
    prefix = (args.prefix if args.prefix.is_absolute() else workspace/args.prefix).resolve()
    setup = prefix/'setup.bash'
    require(setup.is_file(), 'ROS overlay setup is missing: '+str(setup))
    # Re-exec after sourcing so Python's import path and native ROS libraries
    # both use the requested overlay. Positional shell arguments preserve paths.
    if os.environ.get('GBPLANNER_GATE_CHECK_OVERLAY') != str(prefix):
        environment = dict(os.environ, GBPLANNER_GATE_CHECK_OVERLAY=str(prefix),
                           PYTHONDONTWRITEBYTECODE='1')
        os.execvpe('bash', ['bash', '-c', 'source "$1" >&2 && shift && exec "$@"',
                   'queued-gate-overlay', str(setup), sys.executable, str(Path(__file__).resolve()),
                   *sys.argv[1:]], environment)
    output = args.output.resolve()
    log_path = output.with_suffix('.log')
    require(output != log_path and not output.exists() and not log_path.exists(),
            'Use a new result filename with a non-.log extension')
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    source = workspace/'src/gbplanner_local/scripts/control_node.py'
    started = time.monotonic()
    deadline = started+args.timeout
    summary = {'status':'failed', 'scope':__doc__, 'cases':{}, 'prefix':str(prefix),
               'control_source':str(source),
               'control_source_sha256':hashlib.sha256(source.read_bytes()).hexdigest()}
    process = probe = None
    initialized = False
    try:
        import rclpy
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import Odometry
        from planner_msgs.action import PathFollower
        from planner_msgs.srv import PciStop, PciTrigger, PlannerRequestPath, PlannerSrv, ValidatePath
        from rclpy.action import ActionServer, CancelResponse, GoalResponse
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

        class Probe(Node):
            def __init__(self):
                self.ns = '/queued_gate_'+str(os.getpid())
                super().__init__('fixture', namespace=self.ns)
                self.group = ReentrantCallbackGroup()
                self.x, self.revision = 0., 1
                self.plans, self.gates, self.validations, self.actions, self.events, self.states = [], [], [], [], [], []
                self.client_cache, self.active = {}, None
                self.cancels = self.overlaps = 0
                self.gate_validation_wait = None
                self.hold_gate_validation = True
                self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)
                self.map_pub = self.create_publisher(String, 'map_updates', durable)
                self.route_pub = self.create_publisher(String, 'execution_status', durable)
                self.create_subscription(String, 'control_status',
                    lambda message:self.states.append(json.loads(message.data)), durable)
                self.create_service(PlannerSrv, 'planner', self.plan, callback_group=self.group)
                self.create_service(PlannerRequestPath, 'gbplanner/passing_gate', self.gate, callback_group=self.group)
                self.create_service(ValidatePath, 'validate_path', self.validate, callback_group=self.group)
                self.server = ActionServer(self, PathFollower, 'path_follower', execute_callback=self.execute,
                    goal_callback=self.accept, cancel_callback=self.cancel, callback_group=self.group)
                self.create_timer(.03, self.publish, callback_group=self.group)

            def event(self, name, **data):
                entry = {'index':len(self.events), 'elapsed_s':time.monotonic()-started, 'event':name, **data}
                self.events.append(entry)
                return entry['index']

            def publish(self):
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                message.header.stamp = self.get_clock().now().to_msg()
                message.pose.pose = pose(self.x)
                self.odom_pub.publish(message)
                # A stable synthetic map avoids incidental preflight retries;
                # this check concerns scheduling, not map-change validation.
                self.map_pub.publish(String(data=json.dumps({'map_revision':self.revision,
                    'observation_revision':self.revision, 'known_cells':100, 'stamp_sec':0, 'stamp_nanosec':0})))
                if self.active is not None:
                    entry = self.active
                    self.route_pub.publish(String(data=json.dumps({'state':'EXECUTING', 'active':True,
                        'output_in_flight':False, 'output_unconfirmed':False,
                        'validates_remaining_path':True, 'validated_revision':self.revision,
                        'goal_id':[int(v) for v in entry['handle'].goal_id.uuid],
                        'generation':0, 'endpoint':[entry['end'],0.,0.,0.]})))
                return message.header.stamp.sec+message.header.stamp.nanosec*1.e-9

            def plan(self, request, response):
                self.plans.append({'source_x':self.x, 'bound_mode':request.bound_mode,
                                   'event':self.event('local_plan', active_action=self.active is not None)})
                response.planning_bound_mode, response.status = request.bound_mode, PlannerSrv.Response.K_FORWARD
                response.path = [pose(self.x), pose(self.x+1.)]
                return response

            def gate(self, request, response):
                response.bound.mode = 1
                response.path = [pose(self.x), pose(self.x+2.)]
                self.gates.append({'source_x':self.x, 'path':[p.position.x for p in response.path],
                    'bound_mode':response.bound.mode, 'active_action':self.active is not None,
                    'event':self.event('gate_plan_return', active_action=self.active is not None)})
                return response

            async def validate(self, request, response):
                entry = {'path':[p.position.x for p in request.path], 'bound_mode':request.bound_mode,
                    'frame':request.header.frame_id, 'first_pose_is_observation':request.first_pose_is_observation,
                    'event':self.event('validation_received', bound_mode=request.bound_mode)}
                self.validations.append(entry)
                if request.bound_mode == 1 and self.hold_gate_validation:
                    self.hold_gate_validation = False
                    self.gate_validation_wait = Future()
                    await self.gate_validation_wait
                response.valid, response.reason, response.map_revision = True, 'protocol_fixture', self.revision
                entry['returned_event'] = self.event('validation_return', bound_mode=request.bound_mode)
                return response

            def accept(self, request):
                self.event('action_goal_requested', bound_mode=request.bound_mode,
                           path=[p.pose.position.x for p in request.path])
                return GoalResponse.ACCEPT

            async def execute(self, handle):
                if self.active is not None:self.overlaps += 1
                points = [p.pose.position.x for p in handle.request.path]
                entry = {'handle':handle, 'done':Future(), 'end':points[-1]}
                record = {'id':[int(v) for v in handle.goal_id.uuid], 'points':points,
                    'bound_mode':handle.request.bound_mode, 'manual_path':handle.request.manual_path,
                    'event':self.event('action_started', number=len(self.actions)+1)}
                self.actions.append(record)
                self.active = entry
                outcome = await entry['done']
                if outcome == 'success':handle.succeed()
                elif outcome == 'cancel':handle.canceled()
                else:handle.abort()
                record['outcome'], record['finished_event'] = outcome, self.event('action_finished', outcome=outcome)
                if self.active is entry:self.active = None
                result = PathFollower.Result()
                result.success = outcome == 'success'
                return result

            def cancel(self, handle):
                self.cancels += 1
                entry = self.active
                timer = None
                def complete():
                    timer.cancel()
                    if entry is not None and not entry['done'].done():entry['done'].set_result('cancel')
                # Let ActionServer enter CANCELING before its execute callback
                # completes; a same-callback cancellation races that transition.
                timer = self.create_timer(.03, complete, callback_group=self.group)
                return CancelResponse.ACCEPT

            def until(self, predicate, label):
                while not predicate():
                    require(process.poll() is None, 'Controller exited before '+label)
                    if time.monotonic() >= deadline:raise TimeoutError(label)
                    rclpy.spin_once(self, timeout_sec=.01)

            def observe(self, duration):
                end = time.monotonic()+duration
                self.until(lambda:time.monotonic()>=end, 'Observation interval')

            def call(self, name, kind, request):
                if name not in self.client_cache:
                    self.client_cache[name] = self.create_client(kind, name, callback_group=self.group)
                client = self.client_cache[name]
                self.until(client.service_is_ready, name+' discovery')
                future = client.call_async(request)
                self.until(future.done, name+' response')
                return future.result()

            def status(self):
                response = self.call('control_state', Trigger, Trigger.Request())
                require(response.success, 'Status service rejected')
                return json.loads(response.message)

            def state(self, wanted):
                self.until(lambda:self.status()['state']==wanted, wanted)
                return self.status()

            def trigger(self):
                request = PciTrigger.Request(set_auto=False, not_exe_path=False, bound_mode=3, vel_max=.3)
                require(self.call('pci_trigger', PciTrigger, request).success, 'Local trigger rejected')

            def pass_gate(self):
                return self.call('planner_control_interface/std_srvs/pass_gate', Trigger, Trigger.Request())

            def action(self, count, mode):
                self.until(lambda:len(self.actions)>=count and self.status()['state']=='EXECUTING', 'Action '+str(count))
                require(len(self.actions)==count and self.active is not None, 'Unexpected Action count/ownership')
                state = self.status()
                require(state['mode']==mode, 'Wrong Action controller mode')
                return state

            def finish(self):
                require(self.active is not None, 'No active Action')
                self.until(lambda:self.status()['follower_tracks_route'], 'Follower route acknowledgement')
                entry = self.active
                self.x = entry['end']
                stamp = self.publish()
                self.until(lambda:self.status()['odometry_freshness']['last_accepted_source_stamp_s']>=stamp,
                           'Fresh endpoint odometry')
                entry['done'].set_result('success')

        probe = Probe()
        command = [sys.executable, str(source), '--ros-args', '-r', '__ns:='+probe.ns,
                   '-p', 'robot_type:=kGround', '-p', 'planner_trigger_lead_time:=0.0',
                   '-p', 'request_timeout:=5.0', '-p', 'execution_timeout:=20.0']
        summary['command'] = command
        with log_path.open('x') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       env=dict(os.environ), start_new_session=True)
        probe.until(lambda:probe.odom_pub.get_subscription_count()>0, 'Controller odometry subscription')
        probe.until(lambda:probe.status()['known_cells']>0 and probe.status()['odometry_freshness']['fresh'], 'Readiness')

        probe.trigger();probe.action(1, 'local')
        first_action = probe.active
        for _ in range(2):
            reply = probe.pass_gate()
            require(reply.success and reply.message=='passing_gate_queued', 'Busy gate did not acknowledge its slot')
        queued = probe.status()
        slot = copy.deepcopy(queued['pending_gate'])
        require(slot is not None and slot['requests']==2 and queued['state']=='EXECUTING' and
                probe.active is first_action and len(probe.actions)==1 and not probe.gates and probe.cancels==0,
                'Gate enqueue changed the active motion or did not coalesce')
        summary['cases']['busy_queue'] = queued

        require(probe.call('pci_stop', PciStop, PciStop.Request()).success, 'Stop rejected')
        stopped = probe.state('STOPPED')
        probe.observe(.35)
        stopped = probe.status()
        require(stopped['state']=='STOPPED' and stopped['pending_gate']==slot and probe.active is None and
                len(probe.actions)==1 and not probe.gates and probe.cancels==1,
                'Stop lost the pending gate or restarted it without a new explicit task')
        summary['cases']['stop_preserves_without_dispatch'] = stopped

        probe.trigger();resumed = probe.action(2, 'local')
        require(resumed['pending_gate']==slot and not probe.gates, 'Explicit local restart lost/dispatched pending gate')
        probe.finish()
        probe.until(lambda:probe.gate_validation_wait is not None, 'Gate ValidatePath request')
        validating = probe.status()
        require(len(probe.gates)==1 and not probe.gates[0]['active_action'] and
                probe.gates[0]['source_x']==1. and probe.gates[0]['bound_mode']==1 and
                probe.gates[0]['event']>probe.actions[1]['finished_event'] and len(probe.actions)==2 and
                validating['state']=='VALIDATING' and validating['mode']=='passing_gate' and
                validating['pending_gate'] is None and validating['passing_gate_success'] and
                validating['applied_gate']['sequence']==slot['sequence'] and
                validating['applied_gate']['completed_actions']==1,
                'Gate did not consume its retained slot at the completed Action boundary')
        duplicate = probe.pass_gate()
        require(not duplicate.success and duplicate.message=='passing_gate_already_activated',
                'Nonempty gate planner response did not latch before validation/Action')
        require(len(probe.actions)==2, 'Gate Action bypassed the held ValidatePath response')
        summary['cases']['ready_consumption_and_early_success_latch'] = validating

        probe.gate_validation_wait.set_result(True)
        executing = probe.action(3, 'passing_gate')
        validation = next(entry for entry in probe.validations if entry['bound_mode']==1)
        gate_goal = next(entry for entry in probe.events
                         if entry['event']=='action_goal_requested' and entry['bound_mode']==1)
        require(validation['frame']=='map' and validation['first_pose_is_observation'] and
                validation['path']==[1.,1.,3.] and validation['returned_event']<gate_goal['index'] and
                probe.actions[2]['points']==[1.,3.] and probe.actions[2]['bound_mode']==1 and
                not probe.actions[2]['manual_path'] and probe.overlaps==0,
                'Gate did not preserve validated reconnection, original Action path or bound mode')
        summary['cases']['validation_then_action_ack'] = executing
        probe.finish();completed = probe.state('DONE')
        duplicate = probe.pass_gate()
        require(not duplicate.success and duplicate.message=='passing_gate_already_activated' and
                completed['reason']=='passing_gate_complete' and completed['passing_gate_success'] and
                completed['pending_gate'] is None and len(probe.gates)==1 and len(probe.actions)==3,
                'Completed gate was repeated or lost its success latch')
        summary['cases']['completed_duplicate_rejected'] = completed
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'], summary['traceback'] = type(error).__name__+': '+str(error), traceback.format_exc()
    finally:
        if probe is not None:
            # Release fixture coroutines before teardown, including on an
            # assertion failure while validation or an Action is deliberately held.
            if probe.gate_validation_wait is not None and not probe.gate_validation_wait.done():
                probe.gate_validation_wait.set_result(True)
            if probe.active is not None and not probe.active['done'].done():
                probe.active['done'].set_result('abort')
            if initialized and rclpy.ok():
                for _ in range(5):rclpy.spin_once(probe, timeout_sec=.01)
            summary.update(plans=probe.plans, gate_calls=probe.gates, validations=probe.validations,
                           actions=probe.actions, events=probe.events, cancels=probe.cancels,
                           overlaps=probe.overlaps, last_state=probe.states[-1] if probe.states else None)
        if process is not None:
            forced = False
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:process.wait(timeout=4.)
                except subprocess.TimeoutExpired:
                    forced = True;os.killpg(process.pid, signal.SIGKILL);process.wait(timeout=2.)
            summary['cleanup'] = {'exit_code':process.returncode, 'forced':forced}
            if forced or process.returncode!=0:
                summary['status'] = 'failed'
                summary.setdefault('error', 'Controller did not exit cleanly')
        if probe is not None:
            probe.server.destroy();probe.destroy_node()
        if initialized and rclpy.ok():rclpy.shutdown()
        summary['elapsed_s'] = time.monotonic()-started
        output.write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'status':summary['status'], 'cases':list(summary['cases']), 'output':str(output),
                      'elapsed_s':summary['elapsed_s'], 'error':summary.get('error')}))
    return 0 if summary['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())
