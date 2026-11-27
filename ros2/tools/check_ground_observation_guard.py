#!/usr/bin/env python3
"""Focused geometry and real follower group-callback checks, without ROS."""
import ast
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

sys.dont_write_bytecode = True
workspace = Path(__file__).resolve().parents[1]
execution = workspace/'src/gbplanner_execution/scripts'
sys.path[:0] = [str(execution), str(workspace/'src/gbplanner_local/scripts')]
from path_progress import PathProgress
from trajectory_tools import prepare, remaining_validation_routes, state_from_pose


def require(condition, message):
    if not condition:
        raise AssertionError(message)


observed = (-1.5002772808074951, -.0005807739798910916, .23999997973442078, .0004346209921699505)
root = (-1.5002986192703247, -.0005807850975543261, .22702097660896992, .00043474375431423057)
route = [root, (-1.6112646212069692, -.1669685762775086, .22890700043792778, -2.1589687046229318)]
current = (-1.5003834962844849, -.0008676583529449999, .2399994283914566, .016096317155591303)
connected, _ = prepare(route, observed, smooth_yaw=False)
frozen = tuple(tuple(point) for point in connected)
progress = PathProgress([point[:3] for point in connected])
length = math.dist(observed[:3], root[:3])
for distance in (0., progress.guard+length/2.):
    progress.progress = distance
    parts = remaining_validation_routes(progress, current, frozen)
    require(parts == [[current, observed], list(frozen)], 'Known observed seam or fixed vertices were changed')
progress.progress = progress.guard+length+.001
expected = [[(*point, current[3]) for point in progress.remaining(current[:3])]]
require(remaining_validation_routes(progress, current, frozen)==expected,
        'Guard past the first fixed vertex did not restore the ordinary single request')
progress.progress = 0.
expected = [[(*point, current[3]) for point in progress.remaining(current[:3])]]
require(remaining_validation_routes(progress, current, None)==expected,
        'A path without explicit observation provenance acquired split semantics')
boundary_prefix = ((0., 0., .125, 0.), (0., 0., 0., 0.), (1., 0., 0., 0.))
boundary_progress = PathProgress([point[:3] for point in boundary_prefix])
boundary_progress.progress = .375  # Exactly representable cutoff == first fixed vertex arc.
require(remaining_validation_routes(boundary_progress, boundary_prefix[1], boundary_prefix)==
        [[boundary_prefix[1], boundary_prefix[0]], list(boundary_prefix)],
        'A guard exactly on the first fixed vertex prematurely discarded observation provenance')

# Execute the production callback itself with finite in-memory futures. This
# checks callback ordering and route identity without mocking a ROS response as
# a native planning result or starting any node.
tree = ast.parse((execution/'path_follower_node.py').read_text())
follower = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name=='TrajectoryFollower')
callback = next(node for node in follower.body if isinstance(node, ast.FunctionDef) and node.name=='validated')
clock = SimpleNamespace(now=10.)
namespace = {'time': SimpleNamespace(monotonic=lambda:clock.now), 'math': math,
             'state_from_pose': state_from_pose}
exec(compile(ast.Module(body=[callback], type_ignores=[]), 'production_validated_callback', 'exec'), namespace)


def pose(point):
    return SimpleNamespace(position=SimpleNamespace(x=point[0], y=point[1], z=point[2]),
                           orientation=SimpleNamespace(x=0., y=0., z=math.sin(point[3]/2.), w=math.cos(point[3]/2.)))


def reply(valid=True, revision=7, reason='ok'):
    result = SimpleNamespace(valid=valid, map_revision=revision, reason=reason)
    return SimpleNamespace(result=lambda:result)


class Harness:
    validated = namespace['validated']

    def __init__(self):
        requests = [SimpleNamespace(path=[pose(point) for point in part], first_pose_is_observation=True)
                    for part in [[current, observed], list(frozen)]]
        self.group = {'sequence': 4, 'generation': 0, 'deadline': 11.,
                      'requests': requests, 'revisions': [None, None]}
        self.active = {'sequence': 4, 'generation': 0, 'stop': None, 'validation_group': self.group,
                       'pending': True, 'validated': 0, 'start': None, 'manual_path': False,
                       'samples': [current]}
        self.current, self.tolerance, self.revision = current, .05, 7
        self.validation_failure, self.output_action = None, object()
        self.dispatches, self.rechecks, self.stops = 0, [], []

    def fresh(self):
        return True

    def output_ready(self):
        return True

    def dispatch_action(self, active):
        self.dispatches += 1

    def validate(self, deadline=None):
        self.rechecks.append(deadline)

    def stop(self, reason, cancelled):
        self.stops.append(reason)
        self.active['stop'] = reason


probe = Harness()
probe.validated(reply(), probe.group, 1)
require(probe.active['pending'] and probe.active['validated']==0 and probe.dispatches==0,
        'Half a validation group updated ownership or dispatched')
probe.validated(reply(), probe.group, 0)
require(not probe.active['pending'] and probe.active['validated']==7 and probe.dispatches==1,
        'Both successful parts did not release the original dispatch path')

probe = Harness()
probe.revision = 8
probe.validated(reply(revision=8), probe.group, 0)
probe.validated(reply(revision=7), probe.group, 1)
require(probe.active['validated']==7 and probe.rechecks==[11.] and probe.dispatches==0,
        'Mixed map revisions lost their minimum or reset the absolute deadline')

probe = Harness()
probe.validated(reply(), probe.group, 0)
probe.validated(reply(False, 8, 'terrain_slope_exceeded'), probe.group, 1)
failure = probe.validation_failure
require(probe.stops==['path_invalidated:terrain_slope_exceeded'] and probe.dispatches==0
        and probe.active['validated']==0 and failure['map_revision']==8
        and failure['first_pose_is_observation'] and failure['path_points']==len(frozen)
        and not failure['path_truncated']
        and [point[:3] for point in failure['path']]==[list(point[:3]) for point in frozen],
        'Rejected part did not stop with its actual request and response revision')

for key, value in [('sequence', 5), ('generation', 1), ('validation_group', object())]:
    probe = Harness()
    probe.active[key] = value
    probe.validated(reply(False, 8, 'stale_reply'), probe.group, 0)
    require(not probe.stops and probe.validation_failure is None and probe.group['revisions']==[None, None],
            'An obsolete sequence, generation or group changed active state')

probe = Harness()
clock.now = 12.
probe.validated(reply(), probe.group, 0)
require(probe.stops==['validation_timeout'] and probe.dispatches==0,
        'Expired absolute deadline accepted a group response')
print(json.dumps({'status': 'passed', 'scope': __doc__,
                  'checks': ['run6_observation_seam_preserved', 'cutoff_inside_seam_retains_whole_segment',
                             'cutoff_past_seam_restores_single_request', 'no_invented_provenance',
                             'no_half_group_dispatch', 'minimum_revision_and_absolute_deadline',
                             'actual_failure_request_retained', 'stale_callbacks_ignored', 'deadline_enforced']}))
