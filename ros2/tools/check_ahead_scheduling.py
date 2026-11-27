#!/usr/bin/env python3
"""Exercise production PCI scheduling callbacks without ROS or robot motion."""
import argparse
import ast
import copy
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest


WORKSPACE = Path(__file__).resolve().parents[1]
SOURCE = WORKSPACE/'src/gbplanner_local/scripts/control_node.py'
CLOCK = NS(wall=100., ros=20.)


def pose(x=0., y=0., z=0.):
    return NS(position=NS(x=x, y=y, z=z), orientation=NS(x=0., y=0., z=0., w=1.))


class Message:
    def __init__(self):
        self.header = NS(frame_id='', stamp=None)
        self.poses = []


class Pending:
    def add_done_callback(self, callback):
        self.callback = callback

    def result(self):
        return self.response

    def finish(self, response):
        self.response = response
        self.callback(self)


class Client:
    def __init__(self):
        self.available, self.calls = True, []

    def service_is_ready(self):
        return self.available

    def call_async(self, request):
        future = Pending()
        self.calls.append((request, future))
        return future


# Run the actual scheduling/response code. Only transport and clocks are
# in-memory substitutes; this is deliberately not a native planning result.
tree = ast.parse(SOURCE.read_text())
control = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'PlannerControl')
names = {'maybe_plan_ahead', 'ahead_planned', 'continued', 'begin',
         'effective_lead_time', 'record_latency', 'timing_start'}
methods = [n for n in control.body if isinstance(n, ast.FunctionDef) and n.name in names]
namespace = {'math': math, 'copy': copy, 'time': NS(monotonic=lambda: CLOCK.wall),
             'position': lambda p: (p.position.x, p.position.y, p.position.z),
             'PlanAhead': NS(Request=Message), 'ContinuePath': NS(Request=Message),
             'Path': Message, 'PoseStamped': Message}
exec(compile(ast.Module(body=methods, type_ignores=[]), str(SOURCE), 'exec'), namespace)


class Harness:
    def __init__(self, early=True):
        self.state, self.reason, self.mode = 'EXECUTING', 'local_executing', 'local'
        self.early_start, self.auto, self.adaptive_lead = early, True, True
        self.lead_time, self.frame, self.request_timeout, self.execution_timeout = .4, 'map', 30., 600.
        self.epoch, self.route_generation, self.route_token = 3, 0, 1
        self.goal = NS(goal_id=NS(uuid=list(range(16))))
        self.execution_route = {'goal_id': list(self.goal.goal_id.uuid), 'generation': 0,
                                'endpoint': [4., 1., 2., .5]}
        self.ahead, self.ahead_schedule = None, None
        self.pending_trigger, self.pending_homing = None, None
        self.pending_initialization = None
        self.ahead_home_required = self.ahead_global_required = False
        self.cycles, self.max_cycles, self.ahead_count = 0, 100, 0
        self.plan_observation, self.observation_revision = 8, 9
        self.feedback = {'estimated_time_remaining': 8.}
        self.feedback_received = CLOCK.wall
        self.bound_mode, self.commanded = 0, [pose(99.)]
        self.ahead_events, self.events = [], []
        self.planning_latencies = [{'trajectory_s': 1.}]
        self.handoff_latencies = []
        self.ahead_planner, self.continue_client = Client(), Client()
        self.path_pub = NS(publish=lambda message: None)
        self.configured_vel_max = 0.
        self.recovery_events, self.execution_outcomes = [], {}

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=int(CLOCK.ros*1.e9), to_msg=lambda: CLOCK.ros))

    def get_parameter(self, name):
        return NS(value=True)

    def transition(self, state, reason):
        self.state, self.reason = state, reason
        self.events.append((state, reason))

    def ready(self):
        return True

    def fail(self, reason):
        raise AssertionError(reason)

    def drain_result(self):
        pass

    def request_plan(self, mode):
        self.requested_mode = mode


for name in names:
    setattr(Harness, name, namespace[name])


def refusal(reason='future_root_not_free'):
    return NS(success=False, homing_required=False, global_required=False,
              reason=reason, map_revision=12)


class SchedulingChecks(unittest.TestCase):
    def setUp(self):
        CLOCK.wall, CLOCK.ros = 100., 20.

    def test_ros_uint8_uuid_status_is_json_serializable(self):
        # Humble's UUID storage uses numpy.uint8, unlike a plain Python list.
        import numpy as np
        node = Harness()
        node.goal.goal_id.uuid = np.arange(16, dtype=np.uint8)
        node.maybe_plan_ahead()
        data = json.loads(json.dumps({'schedule': node.ahead_schedule,
                                     'attempts': node.ahead_events}))
        self.assertEqual(data['attempts'][0]['goal_id'], list(range(16)))
        self.assertEqual(data['schedule']['key'][1], list(range(16)))

    def test_legacy_window_and_early_start(self):
        normal = Harness(early=False)
        normal.maybe_plan_ahead()
        self.assertEqual(len(normal.ahead_planner.calls), 0)
        normal.feedback['estimated_time_remaining'] = 1.
        normal.maybe_plan_ahead()
        self.assertEqual(len(normal.ahead_planner.calls), 1)
        early = Harness()
        early.maybe_plan_ahead()
        request, _ = early.ahead_planner.calls[0]
        self.assertEqual(request.arrival_time, 8.)
        self.assertEqual((request.root_pose.position.x, request.root_pose.position.y,
                          request.root_pose.position.z), (4., 1., 2.))
        self.assertEqual(early.ahead['event']['trigger'], 'early')
        early.maybe_plan_ahead()
        self.assertEqual(len(early.ahead_planner.calls), 1)

    def test_existing_gates_still_block(self):
        changes = [dict(state='SENDING'), dict(auto=False), dict(mode='homing'),
                   dict(goal=None), dict(pending_trigger={}), dict(pending_homing={}),
                   dict(ahead_home_required=True), dict(ahead_global_required=True),
                   dict(cycles=99), dict(observation_revision=8), dict(feedback_received=99.),
                   dict(lead_time=0.)]
        for change in changes:
            with self.subTest(change=change):
                node = Harness(); node.__dict__.update(change); node.maybe_plan_ahead()
                self.assertEqual(node.ahead_planner.calls, [])
        for eta in (0., -1., math.inf, math.nan):
            node = Harness(); node.feedback['estimated_time_remaining'] = eta
            node.maybe_plan_ahead(); self.assertEqual(node.ahead_planner.calls, [])
        for field, value in (('goal_id', [1]*16), ('generation', 1)):
            node = Harness(); node.execution_route[field] = value
            node.maybe_plan_ahead(); self.assertEqual(node.ahead_planner.calls, [])

    def test_unavailable_service_does_not_consume_attempt(self):
        for client in ('ahead_planner', 'continue_client'):
            node = Harness(); getattr(node, client).available = False
            node.maybe_plan_ahead(); node.maybe_plan_ahead()
            self.assertEqual(node.ahead_schedule['attempts'], 0)
            self.assertEqual(len(node.events), 1)
            getattr(node, client).available = True; node.maybe_plan_ahead()
            self.assertEqual(node.ahead_schedule['attempts'], 1)

    def test_missing_map_retry_needs_new_scan_and_lead_window(self):
        for reason in ('future_root_not_free', 'map_missing'):
            node = Harness(); node.maybe_plan_ahead()
            node.observation_revision = 10
            node.ahead_planner.calls[0][1].finish(refusal(reason))
            node.feedback['estimated_time_remaining'] = 1.
            node.maybe_plan_ahead()  # A scan received before the reply is insufficient.
            self.assertEqual(len(node.ahead_planner.calls), 1)
            node.observation_revision = 11; node.feedback['estimated_time_remaining'] = 8.
            node.maybe_plan_ahead(); self.assertEqual(len(node.ahead_planner.calls), 1)
            node.feedback['estimated_time_remaining'] = 1.; node.maybe_plan_ahead()
            self.assertEqual(len(node.ahead_planner.calls), 2)
            self.assertEqual(node.ahead['event']['trigger'], 'retry')
            node.ahead_planner.calls[1][1].finish(refusal(reason))
            node.observation_revision = 12; node.maybe_plan_ahead()
            self.assertEqual(len(node.ahead_planner.calls), 2)

    def test_other_failures_and_legacy_mode_do_not_retry(self):
        for reason, early in (('no_gain', True), ('current_pose_not_free', True),
                              ('plan_ahead_error', True), ('map_missing', False)):
            node = Harness(early); node.feedback['estimated_time_remaining'] = 1.
            node.maybe_plan_ahead(); node.ahead_planner.calls[0][1].finish(refusal(reason))
            node.observation_revision += 1; node.maybe_plan_ahead()
            self.assertEqual(len(node.ahead_planner.calls), 1)

    def test_generation_and_uuid_have_separate_attempt_budgets(self):
        node = Harness(); node.maybe_plan_ahead()
        node.ahead_planner.calls[0][1].finish(refusal('no_gain'))
        node.route_generation = node.execution_route['generation'] = 1
        node.maybe_plan_ahead(); self.assertEqual(node.ahead['attempt'], 1)
        node.ahead_planner.calls[1][1].finish(refusal('no_gain'))
        node.route_generation = node.execution_route['generation'] = 0
        node.goal.goal_id.uuid = [5]*16; node.execution_route['goal_id'] = [5]*16
        node.maybe_plan_ahead(); self.assertEqual(node.ahead['attempt'], 1)
        self.assertEqual(len(node.ahead_planner.calls), 3)

    def test_delayed_old_action_reply_does_not_reopen_current_schedule(self):
        node = Harness(); node.maybe_plan_ahead()
        old, future = node.ahead, node.ahead_planner.calls[0][1]
        node.ahead = None; node.goal.goal_id.uuid = [6]*16
        node.execution_route['goal_id'] = [6]*16
        node.maybe_plan_ahead(); current = node.ahead
        future.finish(refusal())
        self.assertIs(node.ahead, current)
        self.assertIsNone(node.ahead_schedule['retry_after_observation'])
        self.assertEqual(old['event']['outcome'], 'stale_planning_reply')

    def test_commit_ack_resets_only_after_success(self):
        node = Harness(); node.maybe_plan_ahead()
        root = copy.deepcopy(node.ahead['root'])
        node.ahead_planner.calls[0][1].finish(NS(success=True, homing_required=False,
            global_required=False, reason='ok', map_revision=12, path=[root, pose(5., 1., 2.)],
            budget_enabled=False, required_time_after_root=0.))
        self.assertEqual(node.ahead['stage'], 'committing')
        node.maybe_plan_ahead(); self.assertEqual(len(node.ahead_planner.calls), 1)
        node.continue_client.calls[0][1].finish(NS(accepted=True, reason='trajectory_replaced',
            generation=1, path=[root, pose(5., 1., 2.)], map_revision=12))
        self.assertEqual(node.route_generation, 1)
        self.assertIsNone(node.ahead_schedule)
        self.assertEqual(node.feedback, {})
        node.maybe_plan_ahead(); self.assertEqual(len(node.ahead_planner.calls), 1)

    def finish_plan(self, node):
        root = copy.deepcopy(node.ahead['root'])
        node.ahead_planner.calls[-1][1].finish(NS(success=True, homing_required=False,
            global_required=False, reason='ok', map_revision=12, path=[root, pose(5.,1.,2.)],
            budget_enabled=False, required_time_after_root=0.))

    def test_rejected_splice_replans_once_after_new_observation_in_lead_window(self):
        node = Harness(); node.maybe_plan_ahead(); self.finish_plan(node)
        node.observation_revision = 11
        node.continue_client.calls[0][1].finish(NS(accepted=False, reason='extension_invalid:path_not_free'))
        self.assertEqual(node.ahead_schedule['retry_reason'], 'extension_invalid:path_not_free')
        self.assertEqual(node.ahead_schedule['retry_after_observation'], 11)
        self.assertEqual(node.route_generation, 0)
        self.assertEqual(node.cycles, 0)
        node.feedback['estimated_time_remaining'] = 1.; node.maybe_plan_ahead()
        self.assertEqual(len(node.ahead_planner.calls), 1)  # No new observation after reply.
        node.observation_revision = 12; node.feedback['estimated_time_remaining'] = 8.
        node.maybe_plan_ahead(); self.assertEqual(len(node.ahead_planner.calls), 1)
        node.feedback['estimated_time_remaining'] = 1.; node.maybe_plan_ahead()
        self.assertEqual(len(node.ahead_planner.calls), 2)
        self.assertEqual(node.ahead['event']['retry_reason'], 'extension_invalid:path_not_free')
        self.assertEqual(node.ahead['event']['trigger'], 'retry')
        self.finish_plan(node)
        node.continue_client.calls[1][1].finish(NS(accepted=False, reason='extension_invalid:path_not_free'))
        node.observation_revision = 13; node.maybe_plan_ahead()
        self.assertEqual(len(node.ahead_planner.calls), 2)
        self.assertIsNone(node.ahead_schedule['retry_after_observation'])

    def test_splice_refusal_retry_preserves_task_and_failure_boundaries(self):
        scenarios = [('extension_invalid:current_pose_not_free', {}),
                     ('extension_invalid:map_missing', {}),
                     ('continuation_time_insufficient', {}),
                     ('retiming_initial_state_infeasible', {}),
                     ('extension_invalid:path_not_free', {'early_start':False}),
                     ('extension_invalid:path_not_free', {'ahead_home_required':True}),
                     ('extension_invalid:path_not_free', {'ahead_global_required':True}),
                     ('extension_invalid:path_not_free', {'route_generation':1})]
        for reason, changes in scenarios:
            with self.subTest(reason=reason, changes=changes):
                node=Harness(); node.maybe_plan_ahead(); self.finish_plan(node)
                node.__dict__.update(changes)
                node.continue_client.calls[0][1].finish(NS(accepted=False, reason=reason))
                self.assertIsNone(node.ahead_schedule['retry_after_observation'])
        for stale in ('uuid','epoch','finished'):
            with self.subTest(stale=stale):
                node=Harness(); node.maybe_plan_ahead(); self.finish_plan(node)
                if stale=='uuid':node.goal.goal_id.uuid=[8]*16
                elif stale=='epoch':node.epoch+=1
                else:node.state='WAITING_OBSERVATION';node.goal=None
                node.continue_client.calls[0][1].finish(NS(accepted=False, reason='extension_invalid:path_not_free'))
                self.assertIsNone(node.ahead_schedule['retry_after_observation'])
    def test_begin_clears_old_task_schedule(self):
        node = Harness(); node.maybe_plan_ahead()
        node.begin('local', True, False)
        self.assertIsNone(node.ahead_schedule)
        self.assertIsNone(node.ahead)
        self.assertEqual(node.route_generation, 0)
        self.assertEqual(node.epoch, 4)
        self.assertEqual(node.ahead_events, [])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(SchedulingChecks))
    summary = {'status': 'passed' if result.wasSuccessful() else 'failed',
               'scope': __doc__, 'checks': result.testsRun,
               'source': str(SOURCE), 'sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest()}
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(summary, ensure_ascii=False))
    raise SystemExit(0 if result.wasSuccessful() else 1)
