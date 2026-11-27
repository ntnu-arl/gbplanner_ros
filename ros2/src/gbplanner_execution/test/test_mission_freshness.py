"""Execute the mission consumers' production methods without ROS transport.

The clock and landing adapter are the only substitutes. Methods and mission
field initialization are compiled directly from the two node source files, so
these checks cover their actual receive/freshness/budget decisions on macOS.
"""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest


SOURCES = Path(__file__).resolve().parents[2]


class Clock:
    def __init__(self, ros=10., steady=100.):
        self.ros, self.steady = ros, steady

    def monotonic(self):
        return self.steady

    def now(self):
        return SimpleNamespace(nanoseconds=round(self.ros*1e9))


def consumer(kind, simulated=True, ros=10.):
    """Load existing methods, rather than copying node decisions into a fake."""
    control = kind == 'control'
    path = SOURCES/('gbplanner_local/scripts/control_node.py' if control else
                    'gbplanner_execution/scripts/path_follower_node.py')
    cls = next(item for item in ast.parse(path.read_text()).body if isinstance(item, ast.ClassDef))
    wanted = {'receive_mission', 'observe_mission_clock', 'mission_freshness',
              'mission_error' if control else 'continuation_budget'}
    methods = [item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name in wanted]
    assert len(methods) == len(wanted)
    init = next(item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name == '__init__')
    assignments = [item for item in init.body if isinstance(item, ast.Assign) and
                   all(isinstance(target, ast.Attribute) and isinstance(target.value, ast.Name) and
                       target.value.id == 'self' and target.attr.startswith('mission') for target in item.targets)]
    clock = Clock(ros)
    namespace = {'json': json, 'math': math, 'time': clock}
    body = ast.ClassDef(name='Consumer', bases=[], keywords=[], body=methods, decorator_list=[])
    exec(compile(ast.fix_missing_locations(ast.Module(body=[body], type_ignores=[])), str(path), 'exec'), namespace)
    node = namespace['Consumer']()
    exec(compile(ast.Module(body=assignments, type_ignores=[]), str(path), 'exec'), namespace, {'self': node})
    node.get_clock = lambda: clock
    node.get_parameter = lambda name: SimpleNamespace(value=simulated)
    if control:
        node.landing = SimpleNamespace(latched=False, observe=lambda status, received: False)
        node.advance_landing = lambda: False
        node.require_mission = True
        node.state = 'IDLE'
        node.TERMINAL = {'IDLE'}
    else:
        node.trajectory_elapsed = lambda active, now: active['elapsed']
    return node, clock


def receive(node, stamp=10., remaining=60., legacy=False):
    status = {'enabled': True, 'valid': True, 'remaining_s': remaining, 'reason': 'ok'}
    if not legacy:
        status['stamp_s'] = stamp
    node.receive_mission(SimpleNamespace(data=json.dumps(status)))


class MissionFreshnessTest(unittest.TestCase):
    kinds = ('control', 'follower')

    def assert_decision(self, kind, node, fresh):
        if kind == 'control':
            self.assertEqual(node.mission_error() is None, fresh)
        else:
            request = SimpleNamespace(budget_enabled=True, required_time_after_root=20.)
            if fresh:
                node.continuation_budget(request, {'duration': 1., 'elapsed': 1.})
            else:
                with self.assertRaisesRegex(ValueError, '^continuation_budget_unavailable$'):
                    node.continuation_budget(request, {'duration': 1., 'elapsed': 1.})

    def test_paused_simulation_and_zero_receipt_then_ros_expiry(self):
        for kind in self.kinds:
            with self.subTest(kind=kind):
                node, clock = consumer(kind, ros=0.)
                receive(node, stamp=0.)
                clock.steady += 30.
                receive(node, stamp=0., remaining=600.)
                self.assertEqual(node.mission['remaining_s'], 60.)
                self.assertEqual(node.mission_received, 100.)
                self.assertEqual(node.mission_freshness()['source_diagnostic'], 'duplicate_stamp_s')
                self.assertTrue(node.mission_freshness()['fresh'])
                self.assert_decision(kind, node, True)
                clock.ros = 2.
                self.assertTrue(node.mission_freshness()['fresh'])
                clock.ros = 2.001
                self.assertAlmostEqual(node.mission_freshness()['receipt_ros_age_s'], 2.001)
                self.assertFalse(node.mission_freshness()['fresh'])
                self.assert_decision(kind, node, False)

    def test_mission6_slow_simulation_clock(self):
        for kind in self.kinds:
            with self.subTest(kind=kind):
                node, clock = consumer(kind, ros=23.026)
                receive(node, stamp=23.026)
                clock.steady += 2.6
                clock.ros = 23.108
                health = node.mission_freshness()
                self.assertAlmostEqual(health['source_age_s'], .082)
                self.assertGreater(health['receipt_steady_age_s'], 2.)
                self.assertTrue(health['fresh'])
                self.assert_decision(kind, node, True)

    def test_delayed_source_is_stale_even_when_just_received(self):
        for kind in self.kinds:
            with self.subTest(kind=kind):
                node, _ = consumer(kind)
                receive(node, stamp=7.5)
                health = node.mission_freshness()
                self.assertEqual(health['receipt_ros_age_s'], 0.)
                self.assertEqual(health['source_age_s'], 2.5)
                self.assertFalse(health['fresh'])
                self.assert_decision(kind, node, False)

    def test_clock_rewind_without_heartbeat_requires_new_snapshot(self):
        for kind in self.kinds:
            with self.subTest(kind=kind):
                node, clock = consumer(kind)
                receive(node, stamp=9.9)
                clock.ros = 10.5
                self.assertTrue(node.mission_freshness()['fresh'])
                clock.ros = 10.4
                health = node.mission_freshness()
                self.assertTrue(health['clock_regressed'])
                self.assertFalse(health['fresh'])
                self.assert_decision(kind, node, False)
                clock.ros = 9.
                self.assertLess(node.mission_freshness()['receipt_ros_age_s'], 0.)
                clock.ros = 10.5
                self.assertFalse(node.mission_freshness()['fresh'])
                receive(node, stamp=9.9)
                self.assertFalse(node.mission_freshness()['fresh'])
                receive(node, stamp=10.5)
                self.assertTrue(node.mission_freshness()['fresh'])

    def test_real_time_receipt_timeout(self):
        for kind in self.kinds:
            with self.subTest(kind=kind):
                node, clock = consumer(kind, simulated=False)
                receive(node)
                clock.steady += 2.001
                health = node.mission_freshness()
                self.assertEqual(health['source_age_s'], 0.)
                self.assertFalse(health['fresh'])
                self.assert_decision(kind, node, False)

    def test_legacy_fallback_is_only_for_missing_key(self):
        for kind in self.kinds:
            with self.subTest(kind=kind):
                node, clock = consumer(kind, ros=0.)
                receive(node, legacy=True)
                clock.steady += 30.
                self.assertTrue(node.mission_freshness()['fresh'])
                self.assertEqual(node.mission_freshness()['source_diagnostic'], 'legacy_missing_stamp_s')
                self.assertIsNone(node.mission_freshness()['source_age_s'])
                clock.ros = 2.01
                self.assertFalse(node.mission_freshness()['fresh'])
            for stamp in (None, -1., math.nan, math.inf, -math.inf, True, 'bad'):
                with self.subTest(kind=kind, stamp=stamp):
                    node, _ = consumer(kind)
                    receive(node, stamp=stamp)
                    self.assertIsNone(node.mission)
                    self.assertEqual(node.mission_source_diagnostic, 'invalid_stamp_s')

    def test_out_of_order_and_duplicate_never_replace_budget(self):
        for kind in self.kinds:
            with self.subTest(kind=kind):
                node, clock = consumer(kind)
                receive(node, remaining=25.)
                clock.ros = 10.1
                clock.steady += .1
                receive(node, stamp=9.95, remaining=600.)
                self.assertEqual(node.mission_source_diagnostic, 'out_of_order_stamp_s')
                self.assertEqual(node.mission['remaining_s'], 25.)
                receive(node, stamp=10., remaining=600.)
                self.assertEqual(node.mission_source_diagnostic, 'duplicate_stamp_s')
                self.assertEqual(node.mission['remaining_s'], 25.)
                self.assertEqual(node.mission_received, 100.)
                receive(node, legacy=True, remaining=600.)
                self.assertEqual(node.mission_source_diagnostic, 'missing_stamp_s_after_stamped')
                self.assertEqual(node.mission['remaining_s'], 25.)

    def test_future_source_tolerance(self):
        for kind in self.kinds:
            with self.subTest(kind=kind):
                node, _ = consumer(kind)
                receive(node, stamp=10.101)
                self.assertIsNone(node.mission)
                self.assertEqual(node.mission_source_diagnostic, 'future_stamp_s')
                receive(node, stamp=10.099)
                self.assertTrue(node.mission_freshness()['fresh'])
                self.assertLess(node.mission_freshness()['source_age_s'], 0.)
                self.assertEqual(node.mission_freshness()['age_s'], 0.)

    def test_continuation_deducts_source_age(self):
        node, _ = consumer('follower')
        receive(node, stamp=9., remaining=20.5)
        request = SimpleNamespace(budget_enabled=True, required_time_after_root=20.)
        self.assertTrue(node.mission_freshness()['fresh'])
        self.assertEqual(node.mission_freshness()['receipt_ros_age_s'], 0.)
        with self.assertRaisesRegex(ValueError, '^continuation_time_insufficient$'):
            node.continuation_budget(request, {'duration': 1., 'elapsed': 1.})
        receive(node, stamp=9.5, remaining=22.)
        node.continuation_budget(request, {'duration': 1., 'elapsed': 1.})

    def test_retimed_continuation_charges_new_schedule_before_commit(self):
        node, clock = consumer('follower')
        receive(node, remaining=26.)
        request = SimpleNamespace(budget_enabled=True, required_time_after_root=20.)
        active = {'duration': 10., 'elapsed': 9.}
        node.continuation_budget(request, active)
        with self.assertRaisesRegex(ValueError, '^continuation_time_insufficient$'):
            node.continuation_budget(request, active, replacement_duration=7.)
        clock.ros += .1
        receive(node, stamp=clock.ros, remaining=28.)
        node.continuation_budget(request, active, replacement_duration=7.)


if __name__ == '__main__':
    unittest.main()
