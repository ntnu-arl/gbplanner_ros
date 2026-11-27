"""Recorded homing route and preservation checks; no ROS or map claims."""
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from ground_maneuver import Limits
from ground_return_route import construct_return_route


class ReturnRouteChecks(unittest.TestCase):
    start = (1.416462566711991, -.19995529744270438, .41192998795123176, -2.9827029779745895)
    route = ((1.416462566711991, -.19995529744270438, .3986235934439071, -2.9827029779745895),
             (1.5933739176596198, -.10679218210471812, .39874998340660195, .48470753400748773),
             (1.7961660929140917, 9.07845993176581e-09, .3883285800455938, .48470753400748756))
    limits = Limits(max_speed=.3, max_duration=30., max_path_length=5.)

    def construct(self, route=None, **kwargs):
        return construct_return_route(self.start, self.route if route is None else route,
            offset=.2159, limits=kwargs.pop('limits', self.limits),
            valid_pose=kwargs.pop('valid_pose', lambda pose: True),
            measured_speed=kwargs.pop('measured_speed', 0.), measured_yaw_rate=0., **kwargs)

    def test_recorded_route_retains_interior_and_home_stops(self):
        observed = []
        result = self.construct(valid_pose=lambda pose: observed.append(pose) or True,
                                route_source='autonomy8_recorded_command_suffix_proxy')
        self.assertEqual(result.status, 'success')
        self.assertEqual(result.target_route_indices, (1, 2))
        self.assertEqual(result.diagnostics['zero_motion_root_indices'], [0])
        self.assertEqual(result.original_route[-1], self.route[-1])
        self.assertEqual(len(result.candidates), 2)
        self.assertAlmostEqual(result.candidates[0].duration, 13.672272517791935, places=10)
        self.assertTrue(all(p[2] == self.start[2] for p in observed))
        for candidate in result.candidates:
            previous_index = -1
            for i, target in enumerate(result.targets):
                end = [p.end_index for p in candidate.primitives if p.target_index == i][-1]
                self.assertGreater(end, previous_index); previous_index = end
                sample = candidate.samples[end]
                self.assertLess(math.dist(sample.pose[:2], target[:2]), 1.e-10)
                self.assertLess(abs(math.remainder(sample.yaw-target[2], 2.*math.pi)), 1.e-10)
                self.assertEqual((sample.v, sample.w), (0., 0.))

    def test_all_route_height_and_whole_candidate_limits_apply(self):
        route = list(self.route)
        p = route[1]; route[1] = (p[0], p[1], self.start[2]+.041, p[3])
        rejected = self.construct(route)
        self.assertEqual(rejected.diagnostics['reason'], 'whole_route_height_change_unsupported')
        self.assertEqual(rejected.diagnostics['height_rejection_indices'], [1])
        self.assertFalse(rejected.candidates)
        self.assertFalse(self.construct(limits=replace(self.limits, max_duration=5.)).candidates)
        self.assertEqual(self.construct(measured_speed=.021).diagnostics['reason'], 'ground_start_not_stationary')
        self.assertEqual(self.construct(max_targets=1).diagnostics['reason'], 'ordered_target_limit')

    def test_near_root_turn_and_later_repeated_root_are_not_deleted(self):
        changed_root = (*self.start[:3], self.start[3]+1.e-4)
        result = self.construct((changed_root, *self.route[1:]))
        self.assertEqual(result.target_route_indices, (0, 1, 2))
        self.assertEqual(result.diagnostics['zero_motion_root_indices'], [])
        loop = self.construct((self.route[1], self.start, self.route[-1]))
        self.assertEqual(loop.target_route_indices, (0, 1, 2))
        self.assertEqual(loop.original_route[1], self.start)


if __name__ == '__main__':
    unittest.main()
