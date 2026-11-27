"""Small analytic checks for the inactive, finite ground candidate builder."""
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from ground_maneuver import Limits, construct_maneuvers


class GroundManeuverChecks(unittest.TestCase):
    def test_recorded_nearly_straight_forward_candidate_keeps_tiny_pivots(self):
        # autonomy5_maneuver: microscopic but nonzero yaw deltas generated a
        # false cruise phase of 7.37e-28 s and rejected the 0.65 m forward leg.
        start = (1.8001660930976917, 9.124366906423566e-09, -1.2384545457747568e-07)
        goal = (2.4501660930976867, -7.137517856893543e-08, -1.2384545457747576e-07)
        limits = Limits(max_speed=.3, max_duration=30., max_path_length=5.)
        result = construct_maneuvers(start, [goal], offset=.2159,
                                    limits=limits, valid_pose=lambda pose: True)
        self.assertEqual(result.rejections, {})
        self.assertEqual([c.directions for c in result.candidates], [(1,), (-1,)])
        forward = result.candidates[0]
        self.assertEqual([p.kind for p in forward.primitives], ['pivot', 'straight', 'pivot'])
        self.assertAlmostEqual(forward.duration, .65/.3+.3/.3, places=9)
        self.assertLess(math.dist(forward.samples[-1].pose[:2], goal[:2]), 1.e-12)
        self.assertLess(abs(math.remainder(forward.samples[-1].yaw-goal[2], 2.*math.pi)), 1.e-20)
        for before, after in zip(forward.samples, forward.samples[1:]):
            self.assertGreater(after.time, before.time)
        self.assertTrue(all(p.signed_amount != 0. for p in forward.primitives))

    def test_integrated_targets_controls_and_base_pivot_length(self):
        offset = .2159
        limits = Limits(integration_dt=.037)
        targets = ((.8, .3, 2.8), (.5, .6, -.4))
        result = construct_maneuvers((0., 0., .2), targets, offset=offset,
                                    limits=limits, valid_pose=lambda pose: True)
        self.assertEqual(result.status, 'success')
        self.assertEqual(len(result.candidates), 4)
        self.assertEqual([c.duration for c in result.candidates],
                         sorted(c.duration for c in result.candidates))
        for candidate in result.candidates:
            expected_length = sum(abs(p.signed_amount) * (offset if p.kind == 'pivot' else 1.)
                                  for p in candidate.primitives)
            self.assertAlmostEqual(candidate.path_length, expected_length, places=10)
            for i, target in enumerate(targets):
                end = [p for p in candidate.primitives if p.target_index == i][-1].end_index
                sample = candidate.samples[end]
                self.assertLess(math.dist(sample.pose[:2], target[:2]), 1.e-10)
                self.assertLess(abs(math.remainder(sample.yaw-target[2], 2.*math.pi)), 1.e-10)
                self.assertEqual((sample.v, sample.w), (0., 0.))
            for before, after in zip(candidate.samples, candidate.samples[1:]):
                dt = after.time-before.time
                self.assertGreater(dt, 0.)
                self.assertLessEqual(dt, limits.integration_dt+1.e-12)
                self.assertLessEqual(abs(after.v), limits.max_speed+1.e-12)
                self.assertLessEqual(abs(after.w), limits.max_yaw_rate+1.e-12)
                self.assertLessEqual(abs(after.v-before.v)/dt, limits.max_acceleration+1.e-10)
                self.assertLessEqual(abs(after.w-before.w)/dt, limits.max_yaw_acceleration+1.e-10)
            for primitive in candidate.primitives:
                if primitive.kind != 'pivot':
                    continue
                first = candidate.samples[primitive.start_index]
                q = (first.x+offset*math.cos(first.yaw), first.y+offset*math.sin(first.yaw))
                for sample in candidate.samples[primitive.start_index:primitive.end_index+1]:
                    self.assertLess(math.dist(q, (sample.x+offset*math.cos(sample.yaw),
                                                  sample.y+offset*math.sin(sample.yaw))), 1.e-10)

    def test_straight_profiles_and_limits(self):
        limits = Limits(max_speed=.4, max_acceleration=.2)
        for distance, expected_time in ((.1, 2.*math.sqrt(.1/.2)), (1., 1./.4+.4/.2)):
            result = construct_maneuvers((0., 0., 0.), [(distance, 0., 0.)], offset=.2,
                                        limits=limits, valid_pose=lambda pose: True)
            self.assertAlmostEqual(result.candidates[0].duration, expected_time, places=10)
            self.assertAlmostEqual(result.candidates[0].path_length, distance, places=10)
        for change in ({'max_duration': .1}, {'max_path_length': .1}, {'max_points': 2}):
            result = construct_maneuvers((0., 0., 0.), [(1., 0., 0.)], offset=.2,
                                        limits=replace(limits, **change), valid_pose=lambda pose: True)
            self.assertFalse(result.candidates)
            self.assertTrue(result.rejections)

    def test_checked_pivot_obstacle_and_finite_family_limit(self):
        # q is shared at (.2,0): rotating the base sweeps x from 0 to .4.
        visited = []
        def obstacle(pose):
            visited.append(pose)
            return pose[0] < .15
        result = construct_maneuvers((0., 0., 0.), [(.4, 0., math.pi)], offset=.2,
                                    limits=Limits(), valid_pose=obstacle)
        self.assertFalse(result.candidates)
        self.assertIn('invalid_pose', result.rejections)
        self.assertTrue(any(p[0] >= .15 for p in visited))
        limited = construct_maneuvers((0., 0., 0.), [(1., 0., 0.)], offset=.2,
                                     limits=Limits(max_candidates=1), valid_pose=lambda pose: True)
        self.assertEqual(limited.status, 'candidate_limit')
        self.assertEqual(len(limited.candidates), 1)
        now = [0.]
        def slow_validator(pose):
            now[0] += 1.
            return True
        with patch('ground_maneuver.time.monotonic', lambda: now[0]):
            expired = construct_maneuvers((0., 0., 0.), [(1., 0., 0.)], offset=.2,
                limits=Limits(max_wall_time=.5), valid_pose=slow_validator)
        self.assertEqual(expired.status, 'wall_time_limit')
        self.assertFalse(expired.candidates)


if __name__ == '__main__':
    unittest.main()
