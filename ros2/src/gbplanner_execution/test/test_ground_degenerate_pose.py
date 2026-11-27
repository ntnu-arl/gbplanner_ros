"""Finite-pose construction regressions; no native dynamics or map claims."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from ground_maneuver import Limits, _leg, construct_maneuvers
from ground_route_obligations import construct_route_obligations


class DegeneratePoseChecks(unittest.TestCase):
    # Exact saved inputs from autonomy13_braking_envelope.json, native13's
    # local report. Keeping them here makes the test independent of results/.
    start = (2.4462857184745417, -8.462232685281133e-08,
             .41192998536388825, -1.2716167996888958e-07)
    route = (
        (2.4462857184745417, -8.462232685281133e-08,
         .3943533093741325, -1.271616799688896e-07),
        (2.2522618814583506, -.048525856107937144,
         .39412714843443997, -2.896517887023268),
        (2.0582380444421595, -.09705162759354743,
         .39434784820518787, -2.896517887023268),
        (1.8642142074259684, -.14557739907915773,
         .39393025215580635, -2.896517887023268),
        (1.7152281478451203, -.18283912729226617,
         .39440261542420063, -2.8965178870232675))
    limits = Limits(max_speed=.3, max_duration=30., max_path_length=5.,
                    max_candidates=2, max_wall_time=.1)

    def test_native13_root_remains_a_nonzero_stopped_obligation(self):
        delta = math.remainder(self.route[0][3]-self.start[3], 2.*math.pi)
        self.assertEqual(delta, -2.6469779601696886e-23)
        result = construct_route_obligations(self.start, self.route, offset=.2159,
            limits=self.limits, valid_pose=lambda _: True,
            measured_speed=0., measured_yaw_rate=0.)
        self.assertTrue(result.candidates)
        self.assertEqual(result.diagnostics['zero_motion_root_indices'], [])
        self.assertEqual(result.anchor_route_indices, (0, 1, 4))
        self.assertEqual(result.pass_route_indices, (2, 3))
        self.assertEqual(result.original_route, self.route)
        self.assertEqual(result.diagnostics['original_endpoint'], self.route[-1])
        for candidate, proof in zip(result.candidates, result.candidate_obligations):
            root_parts = [p for p in candidate.primitives if p.target_index == 0]
            self.assertEqual(len(root_parts), 1)
            self.assertEqual((root_parts[0].kind, root_parts[0].signed_amount), ('pivot', delta))
            self.assertTrue(root_parts[0].stop_at_end)
            self.assertGreater(root_parts[0].duration, 0.)
            self.assertLessEqual(candidate.duration, 30.)
            self.assertLessEqual(candidate.path_length, 5.)
            self.assertEqual([p.original_index for p in proof.points], list(range(5)))
            root = proof.points[0]
            self.assertTrue(root.stop)
            self.assertFalse(root.zero_motion)
            self.assertFalse(root.at_start)
            self.assertEqual(root.original_pose, self.route[0])
            self.assertTrue(proof.points[-1].stop)
            for point in proof.points:
                self.assertEqual(point.original_pose, self.route[point.original_index])
                self.assertLessEqual(math.dist(point.reference_pose[:2], point.original_pose[:2]), 1.e-8)
                self.assertLessEqual(abs(math.remainder(point.reference_pose[3]-point.original_pose[3],
                                                        2.*math.pi)), 1.e-8)
            last = candidate.samples[-1]
            self.assertEqual((last.v, last.w), (0., 0.))
            self.assertLessEqual(math.dist(last.pose[:2], self.route[-1][:2]), 1.e-8)
            self.assertLessEqual(abs(math.remainder(last.yaw-self.route[-1][3], 2.*math.pi)), 1.e-9)

    def test_exact_forward_pivot_endpoint_for_both_signs_and_offsets(self):
        start = (.17, -.31, 3.)
        for angle in (-.4, .4):
            for offset in (-.2159, .2159):
                with self.subTest(angle=angle, offset=offset):
                    target_yaw = math.remainder(start[2]+angle, 2.*math.pi)
                    delta = math.remainder(target_yaw-start[2], 2.*math.pi)
                    end_yaw = math.remainder(start[2]+delta, 2.*math.pi)
                    target = (start[0]+offset*(math.cos(start[2])-math.cos(end_yaw)),
                              start[1]+offset*(math.sin(start[2])-math.sin(end_yaw)), target_yaw)
                    for direction in (-1, 1):
                        self.assertEqual(_leg(start, target, direction, offset), (('pivot', delta),))
                    built = construct_maneuvers(start, (target,), offset=offset,
                        limits=self.limits, valid_pose=lambda _: True)
                    self.assertEqual(len(built.candidates), 1)
                    candidate = built.candidates[0]
                    self.assertEqual(len(candidate.primitives), 1)
                    self.assertEqual(candidate.primitives[0].signed_amount, delta)
                    self.assertLessEqual(math.dist(candidate.samples[-1].pose[:2], target[:2]), 1.e-8)
                    self.assertLessEqual(abs(math.remainder(candidate.samples[-1].yaw-target_yaw,
                                                            2.*math.pi)), 1.e-9)

    def test_one_ulp_off_endpoint_does_not_match_the_pure_pivot(self):
        start = (.17, -.31, .7)
        offset = .2159
        yaw = math.remainder(start[2]+.4, 2.*math.pi)
        exact = (start[0]+offset*(math.cos(start[2])-math.cos(yaw)),
                 start[1]+offset*(math.sin(start[2])-math.sin(yaw)), yaw)
        changed = (exact[0], math.nextafter(exact[1], math.inf), yaw)
        self.assertNotEqual(changed[1], exact[1])
        for direction in (-1, 1):
            parts = _leg(start, changed, direction, offset)
            self.assertTrue(any(kind == 'straight' and amount != 0. for kind, amount in parts))

    def test_nonzero_same_xy_heading_and_exact_zero_are_distinct(self):
        start = (0., 0., 0.)
        target = (0., 0., 1.e-4)
        for direction in (-1, 1):
            parts = _leg(start, target, direction, .2159)
            self.assertTrue(parts)
            self.assertTrue(any(kind == 'straight' and amount != 0. for kind, amount in parts))
            net_turn = math.fsum(amount for kind, amount in parts if kind == 'pivot')
            self.assertAlmostEqual(math.remainder(net_turn-target[2], 2.*math.pi), 0., places=14)
            self.assertEqual(_leg(start, start, direction, .2159), ())
        built = construct_maneuvers(start, (start,), offset=.2159,
            limits=self.limits, valid_pose=lambda _: True)
        self.assertEqual(len(built.candidates), 1)
        self.assertEqual(built.candidates[0].primitives, ())
        self.assertEqual(built.candidates[0].duration, 0.)


if __name__ == '__main__':
    unittest.main()
