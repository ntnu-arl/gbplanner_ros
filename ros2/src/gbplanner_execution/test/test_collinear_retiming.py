"""Exact polyline knots, shared arc time and nonuniform ROS timing."""
import math
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from trajectory_tools import RetimingError, retime_polyline, wrap


def assert_retained(path, samples):
    assert [samples[i] for i in samples.vertex_indices] == path
    assert len(samples) == len(samples.velocities) == len(samples.accelerations) == len(samples.time_offsets)
    stamps = [round(t * 1e9) for t in samples.time_offsets]
    assert stamps[0] > 0
    assert all(b > a for a, b in zip(stamps, stamps[1:]))
    for a, b, first, last in zip(path, path[1:], samples.vertex_indices, samples.vertex_indices[1:]):
        delta = tuple(y - x for x, y in zip(a[:3], b[:3]))
        distance = math.hypot(*delta)
        if distance == 0:
            assert all(s[:3] == a[:3] for s in samples[first:last+1])
            continue
        direction = tuple(value / distance for value in delta)
        previous = 0.
        for state in samples[first:last+1]:
            travel = math.fsum((p - origin) * axis for p, origin, axis in zip(state[:3], a[:3], direction))
            assert previous - 1e-13 <= travel <= distance + 1e-13
            projected = tuple(origin + axis * travel for origin, axis in zip(a[:3], direction))
            assert math.dist(projected, state[:3]) < 1e-13
            previous = travel


def test_rest_run_inserts_exact_knots_and_matches_one_analytic_time_law():
    path = [(0., 0., 0., 0.), (.23, 0., 0., .046), (1.1, 0., 0., .22), (1.7, 0., 0., .34)]
    kwargs = dict(speed=.8, yaw_rate=.2, dt=.05, max_acceleration=.7, max_yaw_acceleration=.15)
    old = retime_polyline(path, **kwargs)
    new = retime_polyline(path, **kwargs, merge_collinear=True)
    assert_retained(path, new)
    assert old.time_offsets == [(i+1) * .05 for i in range(len(old))]
    assert new.time_offsets[-1] < old.time_offsets[-1]
    assert all(new.velocities[i][0] > 0. for i in new.vertex_indices[1:-1])
    duration = new.time_offsets[-1] - new.time_offsets[0]
    for state, velocity, acceleration, at in zip(new, new.velocities, new.accelerations, new.time_offsets):
        u = (at - .05) / duration
        position = 1.7 * (10*u**3 - 15*u**4 + 6*u**5)
        speed = 1.7 / duration * 30*u**2*(1-u)**2
        accel = 1.7 / duration**2 * 60*u*(1-u)*(1-2*u)
        assert state[0] == pytest.approx(position, abs=2e-14)
        assert state[3] == pytest.approx(position * .2, abs=2e-14)
        assert velocity[0] == pytest.approx(speed, abs=2e-14)
        assert velocity[3] == pytest.approx(.2*speed, abs=2e-14)
        assert acceleration[0] == pytest.approx(accel, abs=2e-14)
        assert acceleration[3] == pytest.approx(.2*accel, abs=2e-14)
    # The maxima occur between grid samples: certify the whole continuous law.
    assert 1.875*1.7/duration <= .8+1e-14
    assert (10*math.sqrt(3)/3)*1.7/duration**2 <= .7+1e-14
    assert 1.875*.34/duration <= .2+1e-14
    assert (10*math.sqrt(3)/3)*.34/duration**2 <= .15+1e-14


def test_corner_reversal_duplicate_rotation_and_changed_yaw_slope_remain_stops():
    path = [(0., 0., 0., 0.), (1., 0., 0., 0.), (2., 0., 0., 0.),
            (2., 1., 0., 0.), (2., 0., 0., 0.), (2., 0., 0., .2),
            (3., 0., 0., .3), (4., 0., 0., .6), (5., 0., 0., .9)]
    samples = retime_polyline(path, 1., .5, .05, merge_collinear=True)
    assert_retained(path, samples)
    for vertex in (0, 2, 3, 4, 5, 6, 8):
        assert samples.velocities[samples.vertex_indices[vertex]] == (0.,)*4
    for vertex in (1, 7):
        assert math.hypot(*samples.velocities[samples.vertex_indices[vertex]][:3]) > 0.


def test_resolvable_tiny_corner_is_not_replaced_by_a_chord():
    path = [(0., 0., 0., 0.), (1., 0., 0., 0.), (2., 1e-10, 0., 0.)]
    samples = retime_polyline(path, 1., .2, .05, merge_collinear=True)
    assert_retained(path, samples)
    assert samples.velocities[samples.vertex_indices[1]] == (0.,)*4


def test_moving_shared_start_retains_projection_and_signed_yaw_derivative():
    path = [(0., 0., 0., .4), (.3, 0., 0., .34), (1.5, 0., 0., .1)]
    samples = retime_polyline(path, 1., .3, .02, initial_velocity=(.4, .1, -.2, -.08), merge_collinear=True)
    assert_retained(path, samples)
    assert samples.velocities[0] == (.4, 0., 0., -.08)
    assert samples.velocities[samples.vertex_indices[1]][0] > 0.
    duration = samples.time_offsets[-1] - samples.time_offsets[0]
    alpha = .4 * duration / 1.5
    for k in range(10001):
        u = k / 10000
        velocity = 1.5/duration*(1-u)**2*(alpha+2*alpha*u+(30-15*alpha)*u*u)
        acceleration = 1.5/duration**2*((60-36*alpha)*u+(-180+96*alpha)*u*u+(120-60*alpha)*u*u*u)
        assert -1e-12 <= velocity <= 1.+1e-12
        assert abs(acceleration) <= 1.+1e-12
        assert abs(-.2*velocity) <= .3+1e-12
        assert abs(-.2*acceleration) <= .3+1e-12

    # Individually feasible but differently normalized initial yaw/XYZ rates
    # cannot share one polynomial while keeping the first original knot.
    mismatch = retime_polyline(path, 1., .3, .02, initial_velocity=(.1, 0., 0., -.01), merge_collinear=True)
    assert_retained(path, mismatch)
    assert mismatch.velocities[mismatch.vertex_indices[1]] == (0.,)*4


def test_wrapped_yaw_preserves_more_than_half_turn_across_original_knots():
    path = [(float(i), 0., 0., wrap(2.8 + .8*i)) for i in range(7)]
    samples = retime_polyline(path, 1., .4, .05, merge_collinear=True)
    assert_retained(path, samples)
    accumulated = sum(wrap(b[3]-a[3]) for a, b in zip(samples, samples[1:]))
    assert accumulated == pytest.approx(4.8, abs=1e-12)
    assert all(samples.velocities[i][3] > 0. for i in samples.vertex_indices[1:-1])


def test_nanosecond_collision_retains_distinct_vertices_and_point_budget():
    path = [(0., 0., 0., 0.), (.5, 0., 0., 0.), (.5+1e-15, 0., 0., 0.), (1., 0., 0., 0.)]
    samples = retime_polyline(path, 1., .2, .05, merge_collinear=True)
    assert_retained(path, samples)
    assert samples.velocities[samples.vertex_indices[1]] == (0.,)*4
    with pytest.raises(ValueError, match='trajectory_point_budget'):
        retime_polyline(path, 1., .2, .05, max_points=4, merge_collinear=True)


def test_grid_collision_prefers_original_vertex_and_default_stays_compatible():
    path = [(0.,)*4, (.5, 0., 0., 0.), (1., 0., 0., 0.)]
    new = retime_polyline(path, 1., .2, .05, merge_collinear=True)
    assert_retained(path, new)
    half = (new.time_offsets[-1]+new.time_offsets[0])/2.
    assert sum(abs(t-half)<1e-12 for t in new.time_offsets) == 1
    explicit = retime_polyline(path, 1., .2, .05, merge_collinear=False)
    default = retime_polyline(path, 1., .2, .05)
    assert default == explicit and default.time_offsets == explicit.time_offsets
    assert default.velocities == explicit.velocities


@pytest.mark.parametrize('path,initial,branch,end,parameterization', [
    ([(0.,)*4, (.005, 0., 0., 0.), (.01, 0., 0., 0.)], (1., 0., 0., 0.),
     'empty_step_interval', 2, 'shared_collinear'),
    ([(0.,)*4, (.01, 0., 0., 0.), (.02, 1e-10, 0., 0.)], (1., 0., 0., 0.),
     'empty_step_interval', 1, 'segment'),
    ([(0.,)*4, (1., 0., 0., .2), (2., 0., 0., .4)], (1., 0., 0., .2),
     'shared_initial_yaw_speed_exceeds_limit', 2, 'shared_collinear'),
])
def test_rejection_preserves_global_run_context(path, initial, branch, end, parameterization):
    with pytest.raises(RetimingError, match='^retiming_initial_state_infeasible$') as caught:
        retime_polyline(path, 1., .15, .05, initial_velocity=initial, merge_collinear=True)
    details = caught.value.details
    assert json.loads(json.dumps(details, allow_nan=False)) == details
    assert details['branch'] == branch
    context = details['context']
    assert context['profile'] == 'quintic_collinear'
    assert context['run_index'] == 0
    assert context['segment_index'] == (0 if end == 1 else None)
    assert context['start_vertex_index'] == 0 and context['end_vertex_index'] == end
    assert context['start'] == list(path[0]) and context['end'] == list(path[end])
    assert context['initial_velocity'] == list(initial)
    assert context['parameterization'] == parameterization


def test_shared_rejection_distinguishes_solver_limits_from_physical_limits():
    path = [(0.,)*4, (.05, 0., 0., .01), (.1, 0., 0., .02)]
    with pytest.raises(RetimingError) as caught:
        retime_polyline(path, 1., .15, .05, max_yaw_acceleration=.01,
                        initial_velocity=(.4, 0., 0., .08), merge_collinear=True)
    details = caught.value.details
    assert details['branch'] == 'empty_step_interval'
    assert details['angle'] == 0. and details['initial_yaw_speed'] == 0.
    assert details['limits']['speed'] == pytest.approx(.75)
    assert details['limits']['acceleration'] == pytest.approx(.05)
    assert details['limits']['yaw_rate'] == 1.
    assert details['context']['physical_limits']['yaw_rate'] == .15
    assert details['context']['physical_limits']['yaw_acceleration'] == .01
    assert details['context']['initial_velocity'][3] == .08
