"""Drive real follower validation callbacks without a ROS executor or simulator."""
import ast
import bisect
from concurrent.futures import Future
import copy
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))
sys.path.insert(0, str(PACKAGE.parent / 'gbplanner_local/scripts'))
from path_progress import PathProgress
from timed_continuation import prepare_timed_continuation
from trajectory_tools import (TimedSamples, continuation_within_checked,
                              manual_validation_route, prepare_continuation,
                              remaining_validation_routes, retime_polyline,
                              state_from_pose)


class Pose:
    def __init__(self):
        self.position = NS(x=0., y=0., z=0.)
        self.orientation = NS(x=0., y=0., z=0., w=1.)


class Request:
    def __init__(self):
        self.header = NS(frame_id='', stamp=NS(sec=0, nanosec=0))
        self.path = []
        self.first_pose_is_observation = False


class Client:
    def __init__(self):
        self.requests, self.futures = [], []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        future = Future()
        self.requests.append(request)
        self.futures.append(future)
        return future


class Clock:
    steady = 103.
    ns = 0

    def monotonic(self):
        return self.steady

    def now(self):
        return NS(nanoseconds=self.ns,
                  to_msg=lambda: NS(sec=self.ns // 10**9, nanosec=self.ns % 10**9))


def follower():
    source = PACKAGE / 'scripts/path_follower_node.py'
    tree = ast.parse(source.read_text())
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef))
    names = {'validate', 'validated', 'validate_continuation', 'continuation_validated',
             'reject_continuation', 'trajectory_elapsed', 'sample_duration', 'continuation_budget'}
    methods = [item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name in names]
    pose = next(item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name == 'pose')
    clock = Clock()
    namespace = dict(math=math, bisect=bisect, time=clock, Pose=Pose,
                     ValidatePath=NS(Request=Request), PathProgress=PathProgress,
                     TimedSamples=TimedSamples, prepare_continuation=prepare_continuation,
                     continuation_within_checked=continuation_within_checked,
                     remaining_validation_routes=remaining_validation_routes,
                     manual_validation_route=manual_validation_route, state_from_pose=state_from_pose)
    extracted = ast.Module(body=[pose, ast.ClassDef(name='Follower', bases=[], keywords=[],
                           body=methods, decorator_list=[])], type_ignores=[])
    exec(compile(ast.fix_missing_locations(extracted), str(source), 'exec'), namespace)
    node = namespace['Follower']()
    node.frame, node.dt, node.yaw_rate = 'map', .05, .3
    node.smooth, node.max_points, node.trajectory_profile = True, 20000, 'quintic_collinear'
    node.sequence, node.revision, node.validation_timeout = 7, 12, 2.
    node.validator, node.continuation_validator = Client(), Client()
    node.clock = clock
    node.get_clock = lambda: clock
    node.get_parameter = lambda _: NS(value=True)
    node.fresh = lambda: True
    node.transitions, node.stops, node.publications = [], [], []
    node.transition = lambda *args: node.transitions.append(args)
    node.stop = lambda *args: node.stops.append(args)
    node.stats = dict(trajectory_commands=1, hold_commands=0, continuations=0, continuation_rejections=0)
    node.commands = NS(get_subscription_count=lambda: 1)

    def publish(samples, *, origin_stamp_ns):
        node.publications.append((samples, origin_stamp_ns))
        node.stats['trajectory_commands'] += 1
        return origin_stamp_ns

    node.publish_trajectory = publish
    path = [(0., 0., 0., 0.), (.4, 0., 0., .08), (1.2, 0., 0., .24)]
    samples = retime_polyline(path, .8, .3, .05, .7, .2, merge_collinear=True)
    reference_index = len(samples) // 3
    node.current = samples[reference_index]
    node.measured_velocity = samples.velocities[reference_index]
    node.measured_speed = math.sqrt(sum(v*v for v in node.measured_velocity[:3]))
    origin = 10_123_456_789
    elapsed = samples.time_offsets[reference_index] + .1
    clock.ns = origin + round(elapsed * 1.e9)
    # Acquisition precedes callback time. Metadata must select this reference,
    # not the first sample or the newer ROS clock's reference.
    node.odom_stamp = (origin + round((samples.time_offsets[reference_index] + .001) * 1.e9)) * 1.e-9
    progress = PathProgress([p[:3] for p in path], tolerance=.12, guard=.3, max_step=.9)
    progress.update(node.current[:3])
    node.active = dict(sequence=7, generation=2, stop=False,
        handle=NS(is_cancel_requested=False, goal_id=NS(uuid=list(range(16)))),
        start=100., trajectory_start_ros_ns=origin, trajectory_uses_ros_time=True,
        samples=samples, connected=path, progress=progress, speed=.8,
        duration=samples.time_offsets[-1], pending=False, validated=12,
        validation_latest_revision=12, validation_group=None, manual_path=False,
        bound_mode=0, observed_prefix=None)
    full, combined, tail = prepare_timed_continuation(
        path, samples, [path[-1], (1.6, 0., 0., .32), (1.6, .4, 0., .5)],
        speed=.8, yaw_rate=.3, dt=.05, max_acceleration=.7, max_yaw_acceleration=.2,
        max_points=20000, smooth_yaw=True, merge_collinear=True)
    node.continuation = dict(sequence=7, deadline=108., extension=tail,
        request=NS(budget_enabled=True, required_time_after_root=20.),
        response=NS(accepted=False, reason=''), done=Future(), timed_extension=(full, combined),
        started_wall=102., started_ros=(clock.ns-100_000_000)*1.e-9, validation_attempts=0)
    # Enough for the remaining combined trajectory, but not its full history.
    node.mission = dict(enabled=True, valid=True, remaining_s=combined.time_offsets[-1]-elapsed+20.05)
    node.mission_freshness = lambda *args: dict(fresh=True, age_s=0.)
    node.budget_calls = []
    real_budget = node.continuation_budget

    def budget(request, active, replacement_duration=None):
        node.budget_calls.append(replacement_duration)
        return real_budget(request, active, replacement_duration)

    node.continuation_budget = budget
    return node


def complete(future, revision):
    future.set_result(NS(valid=True, map_revision=revision))


def assert_waiting(node, candidate):
    assert node.publications == [] and node.stops == []
    assert node.stats['trajectory_commands'] == 1
    assert node.active['generation'] == 2
    assert not candidate['done'].done()


def prefix_bytes(samples, count):
    values = [value for rows in (samples[:count], samples.velocities[:count],
                                samples.accelerations[:count]) for row in rows for value in row]
    values += samples.time_offsets[:count]
    return struct.pack('!' + 'd' * len(values), *values)


def test_append_preserves_action_progress_clock_and_complete_sample_prefix():
    node = follower()
    active, candidate = node.active, node.continuation
    old = copy.deepcopy(active)
    handle = active['handle']
    elapsed = node.trajectory_elapsed(active)
    node.validate_continuation()
    complete(node.continuation_validator.futures[-1], 12)

    response = candidate['done'].result(timeout=0)
    assert response.accepted and response.reason == 'trajectory_replaced'
    assert node.active is active and active['handle'] is handle
    assert active['sequence'] == 8 and active['generation'] == response.generation == 3
    assert active['start'] == old['start']
    assert active['trajectory_start_ros_ns'] == old['trajectory_start_ros_ns']
    assert active['trajectory_uses_ros_time']
    assert active['progress'].progress == old['progress'].progress > 0.
    assert active['progress'].previous == old['progress'].previous
    for name in ('tolerance', 'guard', 'max_step'):
        assert getattr(active['progress'], name) == getattr(old['progress'], name)
    assert active['progress'].points[:len(old['progress'].points)] == old['progress'].points
    assert active['progress'].arc[:len(old['progress'].arc)] == old['progress'].arc
    assert len(active['samples']) > len(old['samples'])
    assert prefix_bytes(active['samples'], len(old['samples'])) == prefix_bytes(old['samples'], len(old['samples']))
    assert active['samples'].vertex_indices[:len(old['connected'])] == old['samples'].vertex_indices
    assert node.budget_calls == [None, pytest.approx(active['duration'] - elapsed)]
    assert node.mission['remaining_s'] < active['duration'] + 20.
    assert node.publications == [(active['samples'], old['trajectory_start_ros_ns'])]
    assert active['validated'] == active['validation_latest_revision'] == response.map_revision == 12
    reference = node.last_continuation['reference_at_commit']
    index = len(old['samples']) // 3
    assert reference['sample_index'] == index
    assert reference['state'] == list(old['samples'][index])
    assert reference['velocity'] == list(old['samples'].velocities[index])
    assert reference['acceleration'] == list(old['samples'].accelerations[index])
    assert reference['offset_s'] == old['samples'].time_offsets[index]
    assert reference['old_and_new_identical']
    assert node.continuation is None and node.stops == []
    assert node.stats['hold_commands'] == 0 and node.stats['continuations'] == 1


def test_pending_old_route_guard_resumes_waiting_extension_from_validated_callback():
    node = follower()
    candidate = node.continuation
    node.validate(deadline=candidate['deadline'])
    group = node.active['validation_group']
    node.validate_continuation()
    extension_future = node.continuation_validator.futures[-1]
    complete(extension_future, 12)
    assert candidate['waiting_active_validation'] is extension_future
    assert_waiting(node, candidate)
    assert group['deadline'] == candidate['deadline']
    complete(node.validator.futures[-1], 12)
    assert candidate['done'].result(timeout=0).accepted
    assert len(node.publications) == 1 and node.stops == []
    assert 'waiting_active_validation' not in candidate


def test_extension_reply_ahead_of_map_notification_revalidates_old_guard_first():
    node = follower()
    candidate = node.continuation
    node.validate_continuation()
    complete(node.continuation_validator.futures[-1], 13)
    assert_waiting(node, candidate)
    assert node.active['pending'] and len(node.validator.requests) == 1
    assert node.active['validation_group']['deadline'] == candidate['deadline']
    assert node.active['validation_deadline'] == candidate['deadline']
    # The old-route guard includes geometry behind the measured cursor.
    assert min(p.position.x for p in node.validator.requests[-1].path) < node.current[0]
    complete(node.validator.futures[-1], 13)
    assert candidate['done'].result(timeout=0).accepted
    assert node.active['validated'] == node.active['validation_latest_revision'] == 13
    assert len(node.continuation_validator.requests) == 1 and node.stops == []


def test_newer_old_route_guard_requires_a_new_extension_reply():
    node = follower()
    node.active['validated'] = node.active['validation_latest_revision'] = 14
    candidate = node.continuation
    node.validate_continuation()
    complete(node.continuation_validator.futures[-1], 12)
    assert_waiting(node, candidate)
    assert len(node.continuation_validator.requests) == 2
    assert node.validator.requests == [] and candidate['deadline'] == 108.
    complete(node.continuation_validator.futures[-1], 14)
    assert candidate['done'].result(timeout=0).accepted
    assert node.active['validated'] == 14 and node.stops == []


def test_split_guard_revisions_converge_before_extension_commit_with_same_deadline():
    node = follower()
    node.active['observed_prefix'] = list(node.active['connected'])
    candidate = node.continuation
    node.validate(deadline=candidate['deadline'])
    first = list(node.validator.futures)
    assert len(first) == 2
    node.validate_continuation()
    complete(node.continuation_validator.futures[-1], 12)
    complete(first[0], 12)
    assert_waiting(node, candidate)
    complete(first[1], 13)
    assert_waiting(node, candidate)
    assert node.active['validated'] == 12 and node.active['validation_latest_revision'] == 13
    assert len(node.validator.futures) == 4
    assert node.active['validation_group']['deadline'] == candidate['deadline']
    second = node.validator.futures[2:]
    complete(second[0], 13)
    assert_waiting(node, candidate)
    complete(second[1], 13)
    assert_waiting(node, candidate)
    assert len(node.continuation_validator.futures) == 2
    complete(node.continuation_validator.futures[-1], 13)
    assert candidate['done'].result(timeout=0).accepted
    assert len(node.publications) == 1 and node.stops == []


@pytest.mark.parametrize('late,reason', [('deadline', 'continuation_timeout'),
                                      ('endpoint', 'continuation_root_too_close')])
def test_late_reply_keeps_existing_trajectory_untouched(late, reason):
    node = follower()
    active, candidate = node.active, node.continuation
    samples, handle, progress = active['samples'], active['handle'], active['progress']
    node.validate_continuation()
    if late == 'deadline':
        node.clock.steady = candidate['deadline'] + .001
    else:
        node.clock.ns = active['trajectory_start_ros_ns'] + round(active['duration'] * 1.e9)
    complete(node.continuation_validator.futures[-1], 12)
    response = candidate['done'].result(timeout=0)
    assert not response.accepted and response.reason == reason
    assert node.continuation is None and node.active is active
    assert active['samples'] is samples and active['handle'] is handle and active['progress'] is progress
    assert active['generation'] == 2 and active['sequence'] == 7 and active['start'] == 100.
    assert node.stats['continuation_rejections'] == 1
    assert node.stats['trajectory_commands'] == 1 and node.stats['hold_commands'] == 0
    assert node.publications == [] and node.stops == []
