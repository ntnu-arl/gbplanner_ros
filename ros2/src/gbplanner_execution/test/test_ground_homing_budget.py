"""Exercise production homing admission and mission aging without ROS.

The mission receiver, freshness checks and mission_error come from the shared
AST consumer. Only message containers and the two clocks are substituted;
maneuver_budget_error is also compiled directly from control_node.py.
"""
import ast
import math
from pathlib import Path
from types import MethodType, SimpleNamespace
import unittest

from test_mission_freshness import consumer, receive


CONTROL = (Path(__file__).resolve().parents[2] /
           'gbplanner_local/scripts/control_node.py')


def budget_consumer(*, mode='homing', remaining=12., reference=6.,
                    encoded_budget=30., reserve=20., home_cost=7.):
    node, clock = consumer('control')
    cls = next(item for item in ast.parse(CONTROL.read_text()).body
               if isinstance(item, ast.ClassDef))
    method = next(item for item in cls.body
                  if isinstance(item, ast.FunctionDef) and
                  item.name == 'maneuver_budget_error')
    namespace = {'math': math, 'time': clock}
    exec(compile(ast.fix_missing_locations(ast.Module(
        body=[method], type_ignores=[])), str(CONTROL), 'exec'), namespace)
    node.maneuver_budget_error = MethodType(
        namespace['maneuver_budget_error'], node)
    node.mode = mode
    node.ground_maneuver_parameters = {'max_duration': 30.}
    node.maneuver_home_cost = home_cost
    node.maneuver_report = {'reference_duration_s': reference}
    node.maneuver_candidates = [SimpleNamespace(duration=reference)]
    node.maneuver_index = 0
    node.ground_maneuver = SimpleNamespace(
        max_duration=encoded_budget, reference_times=[0., reference])
    receive(node, remaining=remaining)
    node.mission['reserve_s'] = reserve
    return node, clock


class GroundHomingBudgetTest(unittest.TestCase):
    def test_return_can_use_reserve_and_counts_return_only_once(self):
        # A large unrelated home-cost estimate must not be charged again to
        # the complete route that already returns to home.
        for reserve in (20., 200.):
            with self.subTest(reserve=reserve):
                node, _ = budget_consumer(reserve=reserve, home_cost=1000.)
                self.assertIsNone(node.mission_error())
                self.assertIsNone(node.maneuver_budget_error())
                self.assertEqual(node.ground_maneuver.max_duration, 12.)
                self.assertEqual(node.maneuver_report['budget']['remaining_s'], 12.)
                self.assertEqual(node.maneuver_report['budget']['approved_execution_budget_s'], 12.)
                self.assertTrue(node.maneuver_report['budget']['reserve_consumable_for_homing'])
                self.assertEqual(node.maneuver_report['approved_duration_s'], 12.)
                self.assertEqual(node.ground_maneuver.reference_times, [0., 6.])
                self.assertEqual(node.ground_maneuver_parameters['max_duration'], 30.)

    def test_reference_must_fit_strictly_inside_remaining_execution_budget(self):
        for reference in (12., 15.):
            with self.subTest(reference=reference):
                node, _ = budget_consumer(reference=reference)
                self.assertIsNone(node.mission_error())
                self.assertIsNotNone(node.maneuver_budget_error())

    def test_validation_delay_shrinks_budget_and_later_balance_cannot_expand_it(self):
        node, clock = budget_consumer()
        self.assertIsNone(node.maneuver_budget_error())
        self.assertEqual(node.ground_maneuver.max_duration, 12.)

        self.assertEqual(node.maneuver_report['reference_slack_s'], 6.)

        # One ROS second passes during validation; this heartbeat is still
        # fresh, but only eleven of its twelve seconds remain.
        clock.ros += 1.
        clock.steady += 1.
        self.assertTrue(node.mission_freshness()['fresh'])
        self.assertIsNone(node.maneuver_budget_error())
        self.assertEqual(node.ground_maneuver.max_duration, 11.)
        self.assertEqual(node.maneuver_report['budget']['remaining_s'], 11.)
        self.assertEqual(node.maneuver_report['budget']['approved_execution_budget_s'], 11.)
        self.assertEqual(node.maneuver_report['reference_slack_s'], 5.)

        # Even a valid newer battery/status report cannot enlarge the budget
        # already encoded for this action after it has been narrowed.
        receive(node, stamp=clock.ros, remaining=40.)
        self.assertIsNone(node.maneuver_budget_error())
        self.assertEqual(node.ground_maneuver.max_duration, 11.)
        self.assertEqual(node.maneuver_report['budget']['approved_execution_budget_s'], 11.)
        self.assertEqual(node.maneuver_report['approved_duration_s'], 11.)
        self.assertEqual(node.maneuver_report['reference_slack_s'], 5.)
        self.assertEqual(node.ground_maneuver.reference_times, [0., 6.])

    def test_existing_encoded_budget_and_configuration_both_bound_admission(self):
        for remaining, encoded, expected in ((12., 8., 8.), (60., 40., 30.)):
            with self.subTest(remaining=remaining, encoded=encoded):
                node, _ = budget_consumer(remaining=remaining, encoded_budget=encoded)
                self.assertIsNone(node.maneuver_budget_error())
                self.assertEqual(node.ground_maneuver.max_duration, expected)
        node, _ = budget_consumer(reference=8., encoded_budget=8.)
        self.assertIsNotNone(node.maneuver_budget_error())

    def test_fresh_heartbeat_can_have_an_exhausted_aged_balance(self):
        for age in (.5, .75):
            with self.subTest(age=age):
                node, clock = budget_consumer(remaining=.5, reference=.1)
                clock.ros += age
                clock.steady += age
                self.assertTrue(node.mission_freshness()['fresh'])
                self.assertEqual(node.mission['remaining_s'], .5)
                self.assertEqual(node.mission['reason'], 'ok')
                self.assertEqual(node.mission_error(), 'mission_budget_exhausted')
                self.assertEqual(node.maneuver_budget_error(), 'mission_budget_exhausted')

    def test_local_keeps_outbound_inverse_home_and_reserve_budget(self):
        for reserve, remaining, admitted in (
                (20., 87., True), (20., 86.9, False),
                (40., 107., True), (40., 106.9, False)):
            with self.subTest(reserve=reserve, remaining=remaining):
                node, _ = budget_consumer(mode='local', reserve=reserve,
                                          remaining=remaining, home_cost=7.)
                self.assertEqual(node.maneuver_budget_error() is None, admitted)
                budget = node.maneuver_report['budget']
                self.assertEqual(budget['approved_outward_s'], 30.)
                self.assertEqual(budget['inverse_return_s'], 30.)
                self.assertEqual(budget['current_home_cost_s'], 7.)
                self.assertEqual(budget['reserve_s'], reserve)
                self.assertEqual(budget['total_cost_s'], 60. + 7. + reserve)
                self.assertEqual(node.ground_maneuver.max_duration, 30.)


if __name__ == '__main__':
    unittest.main()
