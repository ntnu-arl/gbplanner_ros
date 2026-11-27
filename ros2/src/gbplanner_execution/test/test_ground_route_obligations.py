"""Recorded route obligation checks; construction is not a map safety test."""
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from ground_maneuver import Limits
from ground_route_obligations import construct_route_obligations


class RouteObligationChecks(unittest.TestCase):
    start = (2.445230145436494, -8.840578727567243e-08, .41192998527483965, -1.3102025001267135e-07)
    route = ((2.445230145436494, -8.840578727567243e-08, .39460430050950124, -1.3102025001267135e-07),
        (2.245873887125841, -.0160338603698035, .3945578372396523, -3.0613376691933465),
        (2.0465176288151876, -.03206763233381972, .39433362780945425, -3.0613376691933465),
        (1.8471613705045344, -.04810140429783595, .3946268516599638, -3.0613376691933465),
        (1.5994350453077857, -.06802547109838214, .3945164276299742, -3.0613376691933465),
        (1.6815073337810225, .1143575106696742, .39474986671418355, 1.1479426437929054),
        (1.7635796222542592, .29674049243773054, .3956220447486234, 1.1479426437929054),
        (1.855800780437534, .50167653265859, .39691051544552414, 1.1479426437929052))
    limits = Limits(max_speed=.3, max_duration=30., max_path_length=5., max_candidates=2, max_wall_time=.1)

    def test_native9_preserves_heading_stations_and_proves_every_original_point(self):
        result = construct_route_obligations(self.start, self.route, offset=.2159, limits=self.limits,
            valid_pose=lambda pose: True, measured_speed=0., measured_yaw_rate=0.)
        self.assertEqual(result.anchor_route_indices, (1,4,5,7))
        self.assertEqual(result.pass_route_indices, (2,3,6))
        self.assertEqual(result.diagnostics['zero_motion_root_indices'], [0])
        self.assertEqual(len(result.candidates), 2)
        self.assertEqual(result.status, 'candidate_limit')
        for candidate, proof in zip(result.candidates, result.candidate_obligations):
            self.assertLessEqual(candidate.duration, 30.)
            self.assertLessEqual(candidate.path_length, 5.)
            self.assertEqual(len(proof.pass_through_distances), len(candidate.primitives))
            self.assertEqual([p.original_index for p in proof.points], list(range(8)))
            self.assertEqual([p.original_index for p in proof.points if not p.stop], [2,3,6])
            for mapping in proof.points:
                self.assertLessEqual(math.dist(mapping.reference_pose[:2], mapping.original_pose[:2]), 1.e-8)
                self.assertLessEqual(abs(math.remainder(mapping.reference_pose[3]-mapping.original_pose[3],2.*math.pi)),1.e-8)
            self.assertAlmostEqual(proof.pass_through_distances[3][0], .2, places=6)
            self.assertAlmostEqual(proof.pass_through_distances[3][1], .4, places=6)
            # Original terrain-route spacing is close to, but not exactly,
            # 0.2 m; the obligation retains that actual distance.
            self.assertAlmostEqual(proof.pass_through_distances[8][0],
                                   math.dist(self.route[5][:2],self.route[6][:2]),places=10)

    def test_repeated_points_and_reversal_remain_distinct_stopped_obligations(self):
        start=(0.,0.,1.,0.);route=(start,(.2,0.,1.,0.),(.2,0.,1.,0.),(.1,0.,1.,0.),(.3,0.,1.,0.))
        result=construct_route_obligations(start,route,offset=.1,limits=self.limits,
            valid_pose=lambda pose:True,measured_speed=0.,measured_yaw_rate=0.)
        self.assertEqual(result.anchor_route_indices,(1,2,3,4))
        self.assertFalse(result.pass_route_indices)
        self.assertTrue(result.candidates)
        for proof in result.candidate_obligations:
            self.assertEqual([p.original_index for p in proof.points],list(range(5)))
            self.assertTrue(all(p.stop for p in proof.points))
            self.assertTrue(proof.points[2].zero_motion)
            self.assertEqual((proof.points[1].primitive_index,proof.points[1].progress),
                             (proof.points[2].primitive_index,proof.points[2].progress))

    def test_whole_route_height_and_original_resource_limits_are_not_relaxed(self):
        route=list(self.route);p=route[2];route[2]=(p[0],p[1],self.start[2]+.041,p[3])
        bad=construct_route_obligations(self.start,route,offset=.2159,limits=self.limits,
            valid_pose=lambda pose:True,measured_speed=0.,measured_yaw_rate=0.)
        self.assertEqual(bad.diagnostics['reason'],'whole_route_height_change_unsupported')
        self.assertFalse(bad.candidates)
        short=construct_route_obligations(self.start,self.route,offset=.2159,
            limits=replace(self.limits,max_duration=5.),valid_pose=lambda pose:True,
            measured_speed=0.,measured_yaw_rate=0.)
        self.assertFalse(short.candidates)


if __name__ == '__main__':
    unittest.main()
