"""Focused wire-contract and actual follower lifecycle checks without ROS."""
import ast
import copy
from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace as Object
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from ground_maneuver import Limits, construct_maneuvers
from ground_maneuver_transport import TransportLimits, decode_maneuver, validate_start


def fixture():
    limits = Limits(max_speed=.3, max_duration=30., max_path_length=5.)
    candidate = construct_maneuvers((0., 0., .2), [(.5, .2, .8)], offset=.2159,
        limits=limits, valid_pose=lambda pose: True).candidates[0]
    message = Object(control_offset=candidate.control_offset,
        max_speed=limits.max_speed, max_yaw_rate=limits.max_yaw_rate,
        max_acceleration=limits.max_acceleration, max_yaw_acceleration=limits.max_yaw_acceleration,
        max_duration=30., reference_times=[s.time for s in candidate.samples],
        forward_velocities=[s.v for s in candidate.samples], yaw_rates=[s.w for s in candidate.samples],
        segments=[Object(kind=0 if p.kind == 'straight' else 1, signed_amount=p.signed_amount,
                         start_index=p.start_index, end_index=p.end_index, stop_at_end=True)
                  for p in candidate.primitives])
    states = [(s.x, s.y, 1., s.yaw) for s in candidate.samples]
    return candidate, states, message


def follower_methods(*names):
    source = Path(__file__).resolve().parents[1] / 'scripts/path_follower_node.py'
    cls = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.ClassDef))
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    body = ast.ClassDef(name='Follower', bases=[], keywords=[], body=methods, decorator_list=[])
    namespace = {'math': math, 'wrap': lambda v: math.remainder(v, 2.*math.pi),
                 'time': Object(monotonic=lambda: 50.),
                 'GoalStatus': Object(STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6)}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[body], type_ignores=[])), str(source), 'exec'), namespace)
    return namespace['Follower']()


class TransportChecks(unittest.TestCase):
    def test_reference_preserves_geometry_limits_and_stopped_start(self):
        candidate, states, message = fixture()
        decoded = decode_maneuver(states, message, caps=TransportLimits())
        self.assertEqual(decoded.path, tuple(states))
        self.assertEqual(decoded.times, tuple(message.reference_times))
        self.assertAlmostEqual(decoded.path_length, candidate.path_length)
        self.assertEqual(decoded.max_duration, 30.)
        validate_start(decoded, states[0], 0., 0.)
        for current, v, w in ((states[0], .021, 0.), (states[0], 0., .031),
                              ((states[0][0], states[0][1], 1., .24), 0., 0.)):
            with self.assertRaises(ValueError):
                validate_start(decoded, current, v, w)
        with self.assertRaisesRegex(ValueError, 'max_speed_exceeds_limit'):
            decode_maneuver(states, message, caps=replace(TransportLimits(), max_speed=.15))

    def test_malformed_reference_never_repaired(self):
        _, states, message = fixture()
        for field, edit in (
            ('reference_times', lambda values: [0.]*len(values)),
            ('forward_velocities', lambda values: [1.]+values[1:]),
            ('segments', lambda values: values[:-1]),
        ):
            changed = copy.deepcopy(message)
            setattr(changed, field, edit(getattr(changed, field)))
            with self.subTest(field=field), self.assertRaises(ValueError):
                decode_maneuver(states, changed, caps=TransportLimits())
        moved = list(states); moved[2] = (moved[2][0]+.001, *moved[2][1:])
        with self.assertRaisesRegex(ValueError, 'model_mismatch'):
            decode_maneuver(moved, message, caps=TransportLimits())

    def test_ordered_interior_crossings_are_preserved_and_not_clamped(self):
        _, states, message = fixture()
        index = next(i for i, s in enumerate(message.segments) if s.kind == 0)
        segment = message.segments[index]
        length = abs(segment.signed_amount)
        segment.pass_through_distances = [.25*length, .75*length]
        decoded = decode_maneuver(states, message, caps=TransportLimits())
        self.assertEqual(decoded.segments[index].pass_through_distances,
                         tuple(segment.pass_through_distances))
        self.assertTrue(decoded.segments[index].stop_at_end)
        for distances in ([0.], [length], [-.01], [math.nan],
                          [.75*length, .25*length], [.25*length, .25*length],
                          [length*i/66. for i in range(1, 66)]):
            with self.subTest(distances=distances):
                segment.pass_through_distances = distances
                with self.assertRaisesRegex(ValueError, 'invalid_ground_pass_through_distances'):
                    decode_maneuver(states, message, caps=TransportLimits())
        segment.pass_through_distances = []
        pivot = next(s for s in message.segments if s.kind == 1)
        pivot.pass_through_distances = [.01]
        with self.assertRaisesRegex(ValueError, 'invalid_ground_pass_through_distances'):
            decode_maneuver(states, message, caps=TransportLimits())

    def test_terminal_does_not_release_motion_until_new_stationary_observations(self):
        node = follower_methods('action_terminal', 'observe_ground_stop', 'ground_stop_confirmed')
        active = dict(ground=True, output_status=4, output_terminal=False, stop=None,
                      ground_settled=9, ground_settle_stamp=10., ground_settle_since=9.)
        node.active = active; node.output_in_flight = active; node.odom_stamp = 10.
        node.odom_timeout = .5; node.transition = lambda *args: None; node.fresh = lambda: True
        node.action_terminal(active, True, 'ok')
        self.assertIs(node.output_in_flight, active)
        self.assertFalse(node.ground_stop_confirmed(active))
        node.measured_speed = 0.; node.measured_yaw_rate = 0.
        for stamp in (10.05, 10.10, 10.15):
            node.odom_stamp = stamp; node.observe_ground_stop(active)
        self.assertTrue(node.ground_stop_confirmed(active))
        node.odom_stamp = 10.2; node.measured_speed = .03; node.observe_ground_stop(active)
        self.assertFalse(node.ground_stop_confirmed(active))

    def test_ground_timeout_uses_motion_budget_not_wall_or_reference_duration(self):
        node = follower_methods('tick_action')
        node.stop = lambda *args: calls.append(args)
        calls = []
        node.action_timeout = .1
        active = dict(ground=True, output_handle=object(), output_deadline=20., duration=30.,
                      output_success=False, start=0.)
        node.trajectory_elapsed = lambda *args: 2.
        node.tick_action(active, 500.)
        self.assertFalse(calls)
        node.trajectory_elapsed = lambda *args: 30.
        node.tick_action(active, 500.)
        self.assertEqual(calls, [('ground_maneuver_timeout', False)])


if __name__ == '__main__':
    unittest.main()
