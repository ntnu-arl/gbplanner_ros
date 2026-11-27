"""Run the production gate scheduler/validation/Action handoff without ROS.

Only ROS transport, message containers, readiness input and publication are
substituted. Compile the actual controller methods and initialization fields,
as in test_mission_freshness.py; no scheduler decisions are copied here.
Run: python3 -m unittest discover -s src/gbplanner_local/test -p test_gate_requests.py -v
"""
import ast
import copy
from concurrent.futures import Future as ConcurrentFuture
import json
import math
from pathlib import Path as FilePath
import runpy
import threading
import time
from types import SimpleNamespace as NS
import unittest


SOURCE = FilePath(__file__).resolve().parents[1]/'scripts/control_node.py'


class Future(ConcurrentFuture):
    def __await__(self):
        if not self.done():
            yield self
        return self.result()


def pose(x=0.):
    return NS(position=NS(x=float(x), y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))


class Message:
    K_EXACT_BOUND = 3

    def __init__(self):
        self.header = NS(frame_id='', stamp=NS(sec=0, nanosec=0))
        self.pose = pose()
        self.root_pose = pose()
        self.path = []
        self.poses = []
        self.first_pose_is_observation = False


class Client:
    def __init__(self):
        self.available = True
        self.calls, self.futures = [], []

    def service_is_ready(self):
        return self.available

    server_is_ready = service_is_ready

    def call_async(self, request):
        self.calls.append(copy.deepcopy(request))
        future = Future()
        self.futures.append(future)
        return future

    def send_goal_async(self, request, feedback_callback=None):
        return self.call_async(request)


class Goal:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.goal_id = NS(uuid=list(range(16)))
        self.result = Future()
        self.cancellations = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancellations += 1


def controller():
    tree = ast.parse(SOURCE.read_text())
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name=='PlannerControl')
    init = next(item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name=='__init__')
    # Initialize the real task fields, excluding parameter/ROS construction.
    assignments = []
    for item in init.body:
        if isinstance(item, ast.Assign):
            if any(isinstance(target, ast.Attribute) and target.attr=='tf_buffer' for target in item.targets):
                break
            if not any(isinstance(part, ast.Attribute) and part.attr=='declare_parameter' for part in ast.walk(item)):
                assignments.append(item)
    cls.bases = []
    service = NS(Request=Message, Response=NS(K_HOMING=2, K_REPOSITIONING=3, K_BACKWARD=1))
    namespace = dict(copy=copy, json=json, math=math, threading=threading, time=time, Future=Future,
        Path=Message, PoseStamped=Message, Bool=Message,
        PathProgress=runpy.run_path(str(SOURCE.with_name('path_progress.py')))['PathProgress'],
        LandingPolicy=lambda *args: NS(latched=False, enabled=False),
        GoalStatus=NS(STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6),
        PathFollower=NS(Goal=Message))
    for name in ('PlannerSrv', 'PlannerGlobal', 'PlannerHoming', 'PlannerSearch',
                 'PlannerGoToWaypoint', 'PlannerRequestPath', 'ValidatePath'):
        namespace[name] = service
    body = [item for item in tree.body if isinstance(item, ast.FunctionDef)] + [cls]
    exec(compile(ast.Module(body=body, type_ignores=[]), str(SOURCE), 'exec'), namespace)
    node = namespace['PlannerControl'].__new__(namespace['PlannerControl'])
    for key, value in dict(frame='map', request_timeout=10., execution_timeout=60.,
                          endpoint_tolerance=.1, robot_type='kAerial', configured_vel_max=0.,
                          lead_time=0., adaptive_lead=False, max_cycles=100,
                          path_invalidation_retries=0).items():
        setattr(node, key, value)
    exec(compile(ast.Module(body=assignments, type_ignores=[]), str(SOURCE), 'exec'), namespace, {'self':node})
    node.readiness = True
    node.ready = lambda: node.readiness
    node.mission_error = lambda: None
    node.advance_landing = lambda: False
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000, to_msg=lambda:NS(sec=10, nanosec=0)))
    node.get_parameter = lambda name: NS(value=False)
    node.geofence_calls = []
    node.odom = pose()
    node.known_cells = 10
    node.map_revision = 1
    node.published = []
    node.path_pub = NS(publish=node.published.append)
    node.stop_request_pub = NS(publish=node.published.append)
    def transition(state, reason):
        node.state, node.reason = state, reason
        node.events.append({'state':state, 'reason':reason, 'mode':node.mode})
    node.transition = transition
    for name in ('passing_gate_planner', 'validate', 'follower', 'local', 'global_plan',
                 'homing', 'search_planner', 'waypoint_planner'):
        setattr(node, name, Client())
    return node


def enqueue(node, preview=False):
    future = Future()
    with node.gate_inbox_lock:
        node.gate_inbox.append((preview, future))
    return future


def request(node, preview=False):
    future = enqueue(node, preview)
    node.receive_gate_requests()
    return future


def planned(node, path=None, bound=3):
    node.passing_gate_planner.futures[-1].set_result(NS(path=[pose(), pose(1.)] if path is None else path,
                                                      bound=NS(mode=bound)))


def validated(node, valid=True):
    node.validate.futures[-1].set_result(NS(valid=valid, reason='test_geometry', map_revision=node.map_revision))


class GateRequestsTest(unittest.TestCase):
    def test_reentrant_callback_only_uses_inbox(self):
        node = controller()
        node.state = 'EXECUTING'
        coroutine = node.start_gate(NS(), False)
        future = coroutine.send(None)
        self.assertIsNone(node.pending_gate)
        self.assertFalse(future.done())
        node.receive_gate_requests()
        with self.assertRaises(StopIteration) as done:
            coroutine.send(None)
        self.assertTrue(done.exception.value.success)
        self.assertEqual(done.exception.value.message, 'passing_gate_queued')

    def test_busy_requests_coalesce_without_touching_motion(self):
        for state in ('PLANNING', 'VALIDATING', 'SENDING', 'EXECUTING', 'WAITING_REPLAN', 'WAITING_OBSERVATION'):
            with self.subTest(state=state):
                node = controller()
                node.state, node.goal, node.pending_goal, node.auto = state, Goal(), True, True
                original = (node.state, node.epoch, node.goal, node.pending_goal, node.auto, node.bound_mode)
                for _ in range(3):
                    self.assertEqual(request(node).result(), (True, 'passing_gate_queued'))
                self.assertEqual(node.pending_gate, {'sequence':3, 'requests':3})
                self.assertEqual(original, (node.state, node.epoch, node.goal, node.pending_goal, node.auto, node.bound_mode))
                self.assertFalse(node.consume_gate())
                self.assertEqual(len(node.passing_gate_planner.calls), 0)

    def test_priority_and_single_consumption_preserve_waypoint(self):
        node = controller()
        node.state = 'EXECUTING'
        request(node)
        node.state = 'DONE'
        waypoint = {'queued':True}
        node.pending_waypoint = waypoint
        for field in ('pending_homing', 'pending_initialization', 'pending_trigger', 'pending_search', 'pending_global'):
            with self.subTest(priority=field):
                setattr(node, field, {'queued':True})
                self.assertFalse(node.consume_gate())
                setattr(node, field, None)
        node.auto = True
        self.assertFalse(node.consume_gate())
        node.auto = False
        node.ahead = {'stage':'committing'}
        self.assertFalse(node.consume_gate())
        node.ahead = None
        self.assertFalse(node.consume_waypoint())
        node.completed, node.cycles = 2, 3
        node.tick()
        self.assertEqual(node.mode, 'passing_gate')
        self.assertEqual(node.state, 'PLANNING')
        self.assertIsNone(node.pending_gate)
        self.assertIs(node.pending_waypoint, waypoint)
        self.assertEqual(node.applied_gate['completed_actions'], 2)
        self.assertEqual(node.applied_gate['cycles'], 3)
        self.assertFalse(node.consume_gate())
        self.assertEqual(len(node.passing_gate_planner.calls), 1)

    def test_global_is_dispatched_before_gate(self):
        node = controller()
        node.state = 'EXECUTING'
        request(node)
        node.state = 'DONE'
        node.pending_global = {'sequence':1, 'parameters':{'id':1, 'not_check_frontier':False,
            'ignore_time':True, 'bound_mode':3}, 'preview':True, 'requested_vel_max':0., 'vel_max':0.}
        node.tick()
        self.assertEqual(node.mode, 'global')
        self.assertIsNotNone(node.pending_gate)
        self.assertEqual(len(node.global_plan.calls), 1)
        self.assertEqual(len(node.passing_gate_planner.calls), 0)

    def test_consumption_rechecks_dependencies_and_does_not_retry(self):
        for dependency in ('readiness', 'passing_gate_planner', 'validate', 'follower'):
            with self.subTest(dependency=dependency):
                node = controller()
                node.state = 'EXECUTING'
                request(node)
                node.state = 'DONE'
                if dependency=='readiness':
                    node.readiness = False
                else:
                    getattr(node, dependency).available = False
                self.assertTrue(node.consume_gate())
                self.assertEqual(node.state, 'ERROR')
                self.assertIsNone(node.pending_gate)
                self.assertFalse(node.passing_gate_success)
                self.assertEqual(len(node.passing_gate_planner.calls), 0)
                node.tick()
                self.assertEqual(len(node.passing_gate_planner.calls), 0)

    def test_empty_path_and_rpc_failure_allow_explicit_retry(self):
        for rpc_error in (False, True):
            with self.subTest(rpc_error=rpc_error):
                node = controller()
                reply = request(node)
                if rpc_error:
                    node.passing_gate_planner.futures[-1].set_exception(RuntimeError('planner unavailable'))
                else:
                    planned(node, [])
                self.assertEqual(node.state, 'ERROR')
                self.assertFalse(reply.result()[0])
                self.assertFalse(node.passing_gate_success)
                self.assertFalse(request(node).done())
                self.assertEqual(len(node.passing_gate_planner.calls), 2)

    def test_nonempty_path_latches_before_validation_and_action(self):
        node = controller()
        reply = request(node)
        self.assertEqual(request(node).result(), (True, 'passing_gate_in_progress'))
        self.assertIsNone(node.pending_gate)
        planned(node)
        self.assertTrue(node.passing_gate_success)
        self.assertEqual(node.state, 'VALIDATING')
        self.assertFalse(reply.done())
        self.assertEqual(len(node.validate.calls), 1)
        self.assertEqual(len(node.follower.calls), 0)
        self.assertEqual(request(node).result(), (False, 'passing_gate_already_activated'))
        validated(node)
        self.assertEqual(node.state, 'SENDING')
        self.assertFalse(reply.done())
        self.assertEqual(len(node.follower.calls), 1)
        handle = Goal()
        node.follower.futures[-1].set_result(handle)
        self.assertEqual(reply.result(), (True, 'passing_gate_execution_started'))
        self.assertEqual(node.state, 'EXECUTING')
        handle.result.set_result(NS(status=4, result=NS(success=True)))
        node.odom = pose(1.)
        node.tick()
        self.assertEqual(node.state, 'DONE')
        self.assertEqual(node.reason, 'passing_gate_complete')
        self.assertEqual(request(node).result(), (False, 'passing_gate_already_activated'))
        self.assertEqual(len(node.passing_gate_planner.calls), 1)

    def test_validation_rejection_and_action_rejection_keep_success_latch(self):
        for reject_validation in (True, False):
            with self.subTest(reject_validation=reject_validation):
                node = controller()
                reply = request(node)
                planned(node)
                validated(node, not reject_validation)
                if not reject_validation:
                    node.follower.futures[-1].set_result(Goal(accepted=False))
                self.assertEqual(node.state, 'ERROR')
                self.assertFalse(reply.result()[0])
                self.assertIn('path_invalidated' if reject_validation else 'executor_rejected', reply.result()[1])
                self.assertTrue(node.passing_gate_success)
                self.assertEqual(request(node).result(), (False, 'passing_gate_already_activated'))

    def test_execution_failure_keeps_success_latch(self):
        node = controller()
        request(node)
        planned(node)
        validated(node)
        handle = Goal()
        node.follower.futures[-1].set_result(handle)
        handle.result.set_result(NS(status=6, result=NS(success=False)))
        self.assertEqual(node.state, 'ERROR')
        self.assertTrue(node.passing_gate_success)
        self.assertEqual(request(node).result(), (False, 'passing_gate_already_activated'))

    def test_preview_does_not_latch_or_replace_queued_execution(self):
        node = controller()
        reply = request(node, preview=True)
        planned(node)
        self.assertFalse(node.passing_gate_success)
        validated(node)
        self.assertEqual(reply.result(), (True, 'passing_gate_path_ready'))
        self.assertEqual(len(node.follower.calls), 0)
        self.assertFalse(request(node).done())
        node.cancel('user_stop', 'STOPPED')
        node.state, node.mode = 'EXECUTING', 'local'
        request(node)
        queued = node.pending_gate
        self.assertEqual(request(node, preview=True).result(), (False, 'controller_busy'))
        self.assertIs(node.pending_gate, queued)

    def test_preview_can_be_followed_by_queued_execution(self):
        node = controller()
        preview = request(node, preview=True)
        self.assertEqual(request(node).result(), (True, 'passing_gate_queued'))
        planned(node)
        validated(node)
        self.assertTrue(preview.result()[0])
        self.assertFalse(node.passing_gate_success)
        node.tick()
        self.assertEqual(len(node.passing_gate_planner.calls), 2)
        self.assertFalse(node.preview)

    def test_stop_failure_map_reset_and_begin_preserve_gate_flags(self):
        for reset in ('stop', 'failure', 'map'):
            with self.subTest(reset=reset):
                node = controller()
                node.state = 'PLANNING'
                request(node)
                queued = node.pending_gate
                unaccepted = enqueue(node)
                if reset=='stop':
                    node.stop(None, NS())
                elif reset=='failure':
                    node.fail('ordinary_task_failed')
                else:
                    node.receive_map(NS(data=json.dumps(dict(map_revision=2, known_cells=0, stamp_sec=0,
                                                            stamp_nanosec=0, observation_revision=2))))
                self.assertIs(node.pending_gate, queued)
                self.assertFalse(unaccepted.result()[0])
                self.assertTrue(node.idle())
                node.tick()
                self.assertEqual(len(node.passing_gate_planner.calls), 0)
                node.known_cells = 10
                node.begin('local', False, True)
                self.assertIs(node.pending_gate, queued)
                self.assertEqual(len(node.local.calls), 1)
                # A normal completed task supplies the next READY boundary.
                node.state = 'DONE'
                node.tick()
                self.assertEqual(len(node.passing_gate_planner.calls), 1)
        node = controller()
        request(node)
        planned(node)
        node.stop(None, NS())
        self.assertTrue(node.passing_gate_success)
        node.receive_map(NS(data=json.dumps(dict(map_revision=2, known_cells=0, stamp_sec=0, stamp_nanosec=0))))
        node.begin('local', False, True)
        self.assertTrue(node.passing_gate_success)
        self.assertEqual(request(node).result(), (False, 'passing_gate_already_activated'))

    def test_explicit_gate_restart_consumes_suspended_slot(self):
        node = controller()
        node.state = 'PLANNING'
        request(node)
        node.stop(None, NS())
        self.assertFalse(request(node).done())
        self.assertIsNone(node.pending_gate)
        self.assertEqual(len(node.passing_gate_planner.calls), 1)

    def test_map_reset_suspends_gate_at_ready_boundary(self):
        for state in ('DONE', 'PREVIEW'):
            for rollback in (False, True):
                with self.subTest(state=state, rollback=rollback):
                    node = controller()
                    node.state = 'EXECUTING'
                    request(node)
                    queued = node.pending_gate
                    node.state, node.map_revision = state, 5
                    node.receive_map(NS(data=json.dumps(dict(map_revision=4 if rollback else 6,
                        known_cells=10 if rollback else 0, stamp_sec=0, stamp_nanosec=0))))
                    self.assertEqual(node.state, 'ERROR')
                    node.tick()
                    self.assertIs(node.pending_gate, queued)
                    self.assertTrue(node.idle())
                    self.assertEqual(len(node.passing_gate_planner.calls), 0)

    def test_stale_nonempty_planner_reply_cannot_latch_after_stop(self):
        node = controller()
        reply = request(node)
        stale, epoch = node.pending, node.epoch
        # Model an RPC already completed in transport, with its callback held.
        delayed = NS(result=lambda:NS(path=[pose(), pose(1.)], bound=NS(mode=3)))
        node.stop(None, NS())
        self.assertFalse(reply.result()[0])
        node.planned(delayed, epoch, 'passing_gate')
        self.assertTrue(stale.cancelled())
        self.assertFalse(node.passing_gate_success)
        self.assertEqual(len(node.validate.calls), 0)
        self.assertEqual(len(node.follower.calls), 0)

    def test_landing_rejects_unaccepted_inbox(self):
        node = controller()
        reply = enqueue(node)
        node.advance_landing = lambda: True
        node.tick()
        self.assertEqual(reply.result(), (False, 'landing_in_progress'))

    def test_one_pose_gate_still_requires_validation(self):
        node = controller()
        request(node)
        planned(node, [pose()])
        self.assertTrue(node.passing_gate_success)
        self.assertEqual(node.state, 'VALIDATING')
        self.assertEqual(len(node.validate.calls), 1)
        self.assertEqual(len(node.follower.calls), 0)


if __name__=='__main__':
    unittest.main()
