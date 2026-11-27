"""Geometry, continuous limits and moving-start checks for aerial retiming."""
import math
import ast
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from trajectory_tools import RetimingError, TimedSamples, retime_polyline, wrap


def follower_profile():
    """Run the production profile methods without importing ROS on the host."""
    source = Path(__file__).resolve().parents[1]/'scripts/path_follower_node.py'
    cls = next(item for item in ast.parse(source.read_text()).body if isinstance(item, ast.ClassDef))
    methods = [item for item in cls.body if isinstance(item, ast.FunctionDef) and
               item.name in {'profile_start_velocity', 'profile_samples'}]
    body = ast.ClassDef(name='Follower', bases=[], keywords=[], body=methods, decorator_list=[])
    namespace = {'math': math, 'retime_polyline': retime_polyline}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[body], type_ignores=[])), str(source), 'exec'), namespace)
    node = namespace['Follower']()
    node.trajectory_profile, node.yaw_rate, node.dt = 'quintic', .15, .05
    node.max_acceleration, node.max_yaw_acceleration, node.max_points = 1., .3, 20000
    node.measured_speed, node.measured_yaw_rate, node.measured_velocity = 0., 0., (0.,)*4
    return node


def test_follower_requires_rest_for_new_profile_but_keeps_moving_continuation():
    node = follower_profile()
    path = [(0.,0.,0.,0.), (1.,0.,0.,0.)]
    node.measured_speed, node.measured_velocity = .4, (.4,0.,0.,0.)
    with pytest.raises(ValueError, match='^retiming_start_not_stationary$'):
        node.profile_samples(path, path, 1.)
    samples = node.profile_samples(path, path, 1., continuation=True)
    assert samples.velocities[0] == (.4,0.,0.,0.)
    node.measured_speed, node.measured_yaw_rate = .04, .04
    assert node.profile_start_velocity() is None
    node.measured_yaw_rate = .06
    with pytest.raises(ValueError, match='^retiming_start_not_stationary$'):
        node.profile_start_velocity()


def test_follower_does_not_interpret_missing_continuation_velocity_as_rest():
    node = follower_profile()
    for unavailable in (None, (math.nan,0.,0.,0.)):
        node.measured_velocity = unavailable
        with pytest.raises(ValueError, match='^retiming_initial_velocity_unavailable$'):
            node.profile_samples([(0.,)*4,(1.,0.,0.,0.)], [], 1., continuation=True)
    node.trajectory_profile = 'upstream'
    sentinel = [(0.,)*4]
    assert node.profile_samples(sentinel, sentinel, 1., continuation=True) is sentinel


def test_fixed_recorded_reversal_retains_vertices_and_order():
    # The first failed native RMF command, including its short observed seam.
    path = [
        (5.594620088244031e-6, -2.682474194360665e-6, 1.012840333557665, 0.),
        (5.559588638410861e-6, -2.6686445736486396e-6, 1.01305518783996, 0.),
        (.443989800726905, -.1642752429538746, 1.1739667006371266, -.07),
        (.8686533556025909, -.32139923275116794, 1.3278758909040593, -.14),
        (.4194572310201484, -.19672465326097954, 1.147102300772482, -.08),
        (-.02973889356229405, -.07205007377079116, .9663287106409049, -.02),
    ]
    samples = retime_polyline(path, 1., .15, .05)
    assert isinstance(samples, TimedSamples)
    assert len(samples) == len(samples.velocities) == len(samples.accelerations)
    assert [samples[index] for index in samples.vertex_indices] == path
    assert all(b > a for a, b in zip(samples.vertex_indices, samples.vertex_indices[1:]))
    for index in samples.vertex_indices:
        assert samples.velocities[index] == (0.,)*4
        assert samples.accelerations[index] == (0.,)*4
    for start, end, first, last in zip(path, path[1:], samples.vertex_indices, samples.vertex_indices[1:]):
        delta = tuple(b-a for a, b in zip(start[:3], end[:3]))
        length = math.hypot(*delta)
        direction = tuple(value/length for value in delta)
        previous = -1e-12
        for state in samples[first:last+1]:
            displacement = tuple(b-a for a, b in zip(start[:3], state[:3]))
            progress = math.fsum(a*b for a, b in zip(displacement, direction))
            assert previous <= progress+1e-12 <= length+2e-12
            projected = tuple(a+progress*d for a, d in zip(start[:3], direction))
            assert math.dist(projected, state[:3]) < 1e-12
            previous = progress


def test_rest_profile_respects_continuous_peaks_between_samples():
    # Coarse samples do not themselves certify the acceleration peak at
    # u=(3-sqrt(3))/6. Check its analytic value using the selected duration.
    distance, angle, dt = .7, .13, .4
    samples = retime_polyline([(0., 0., 0., 0.), (distance, 0., 0., angle)],
                              .6, .2, dt, max_acceleration=.65, max_yaw_acceleration=.12)
    duration = (samples.vertex_indices[-1]-samples.vertex_indices[0])*dt
    assert 1.875*distance/duration <= .6+1e-12
    assert 1.875*angle/duration <= .2+1e-12
    assert 10.*math.sqrt(3.)/3.*distance/duration**2 <= .65+1e-12
    assert 10.*math.sqrt(3.)/3.*angle/duration**2 <= .12+1e-12
    earlier = duration-dt
    assert max(1.875*distance/earlier/.6,
               1.875*angle/earlier/.2,
               10.*math.sqrt(3.)/3.*distance/earlier**2/.65,
               10.*math.sqrt(3.)/3.*angle/earlier**2/.12) > 1.


def test_derivatives_agree_with_pose_and_velocity_finite_differences():
    dt = .002
    samples = retime_polyline([(0., 0., .3, 3.12), (.4, -.3, .8, -3.02)],
                              .8, .3, dt)
    for index in range(1, len(samples)-1):
        before, after = samples[index-1], samples[index+1]
        numerical_velocity = tuple((b-a)/(2.*dt) for a, b in zip(before[:3], after[:3]))
        numerical_yaw_rate = wrap(after[3]-before[3])/(2.*dt)
        assert math.dist(numerical_velocity, samples.velocities[index][:3]) < 1e-5
        assert abs(numerical_yaw_rate-samples.velocities[index][3]) < 1e-5
        numerical_acceleration = tuple((b-a)/(2.*dt) for a, b in
            zip(samples.velocities[index-1], samples.velocities[index+1]))
        assert math.dist(numerical_acceleration, samples.accelerations[index]) < 2e-5


def test_moving_start_retains_projection_and_signed_yaw_rate():
    samples = retime_polyline([(0., 0., 0., .3), (1., 0., 0., -.2), (1., 1., 0., -.2)],
                              .9, .3, .02, initial_velocity=(.5, .2, -.1, -.1))
    # The transverse .2/- .1 components are intentionally not claimed matched.
    assert samples.velocities[0] == (.5, 0., 0., -.1)
    assert samples.accelerations[0] == (0.,)*4
    first_end = samples.vertex_indices[1]
    assert all(b[0] >= a[0]-1e-12 for a, b in zip(samples[:first_end], samples[1:first_end+1]))
    assert all(wrap(b[3]-a[3]) <= 1e-12 for a, b in zip(samples[:first_end], samples[1:first_end+1]))
    assert samples[first_end] == (1., 0., 0., -.2)
    for index in samples.vertex_indices[1:]:
        assert samples.velocities[index] == (0.,)*4
        assert samples.accelerations[index] == (0.,)*4
    assert max(math.hypot(*v[:3]) for v in samples.velocities) <= .9+1e-12
    assert max(abs(v[3]) for v in samples.velocities) <= .3+1e-12
    assert max(math.hypot(*a[:3]) for a in samples.accelerations) <= 1.+1e-12
    assert max(abs(a[3]) for a in samples.accelerations) <= .3+1e-12


@pytest.mark.parametrize('endpoint,initial,branch', [
    ((.01, 0., 0., 0.), (1., 0., 0., 0.), 'empty_step_interval'),
    ((1., 0., 0., 0.), (-.1, 0., 0., 0.), 'initial_speed_negative'),
    ((.1, 0., 0., 1.), (.5, 0., 0., 0.), 'empty_step_interval'),
    ((1., 0., 0., .2), (0., 0., 0., -.1), 'initial_yaw_speed_negative'),
    ((1., 0., 0., 0.), (0., 0., 0., .1), 'moving_zero_angle'),
    ((0., 0., 0., .2), (.1, 0., 0., 0.), 'moving_zero_length_segment'),
    ((1., 0., 0., 0.), (1.1, 0., 0., 0.), 'initial_speed_exceeds_limit'),
    ((1., 0., 0., .2), (0., 0., 0., .2), 'initial_yaw_speed_exceeds_limit'),
])
def test_infeasible_start_is_rejected_without_changing_geometry(endpoint, initial, branch):
    path = [(0., 0., 0., 0.), endpoint]
    with pytest.raises(RetimingError, match='^retiming_initial_state_infeasible$') as caught:
        retime_polyline(path, 1., .15, .05, initial_velocity=initial)
    assert isinstance(caught.value, ValueError)
    details = caught.value.details
    assert json.loads(json.dumps(details, allow_nan=False)) == details
    assert details['schema_version'] == 1 and details['branch'] == branch
    assert details['context']['profile'] == 'quintic'
    assert details['context']['segment_index'] == 0
    assert details['context']['start_vertex_index'] == 0
    assert details['context']['end_vertex_index'] == 1
    assert details['context']['start'] == list(path[0])
    assert details['context']['end'] == list(endpoint)
    assert details['context']['initial_velocity'] == list(initial)
    assert path == [(0., 0., 0., 0.), endpoint]


@pytest.mark.parametrize('distance,initial,branch,first,last,checked', [
    (.01, 1., 'empty_step_interval', 20, 0, 0),
    (.1, .4, 'candidate_steps_exhausted', 8, 12, 5),
])
def test_rejection_distinguishes_empty_grid_from_checked_infeasible_candidates(
        distance, initial, branch, first, last, checked):
    with pytest.raises(RetimingError) as caught:
        retime_polyline([(0.,)*4, (distance, 0., 0., 0.)], 1., .15, .05,
                        initial_velocity=(initial, 0., 0., 0.))
    details = caught.value.details
    assert details['branch'] == branch
    assert details['stage'] == 'candidate_search'
    assert details['distance'] == distance and details['initial_speed'] == initial
    assert details['duration_interval']['lower'] == initial
    assert details['duration_interval']['upper'] == 2.5*distance/initial
    assert details['step_interval']['first_candidate'] == first
    assert details['step_interval']['last_candidate'] == last
    assert details['step_interval']['evaluated_candidates'] == checked


def test_overflowed_rejection_bound_remains_strict_json():
    # Finite inputs can produce an infinite required braking duration. Logging
    # must retain the original ValueError rather than fail while encoding it.
    with pytest.raises(RetimingError) as caught:
        retime_polyline([(0.,)*4, (.1, 0., 0., 0.)], 1., .15, .05,
                        max_acceleration=1e-320, initial_velocity=(1., 0., 0., 0.))
    details = caught.value.details
    assert details['branch'] == 'duration_lower_exceeds_upper'
    assert details['duration_interval']['lower'] is None
    assert details['duration_interval']['lower_finite'] is False
    assert details['step_interval']['first_candidate'] is None
    assert json.loads(json.dumps(details, allow_nan=False)) == details


def test_moving_start_derivatives_are_hermite_consistent():
    dt = .002
    samples = retime_polyline([(0., 0., 0., 0.), (1., 0., 0., .3)],
                              1., .4, dt, initial_velocity=(.6, 0., 0., .1))
    duration = (len(samples)-1)*dt
    # Independent dense evaluation includes between-grid extrema, and also
    # catches an enlarged T that would overshoot then return to the endpoint.
    for displacement, initial, maximum_v, maximum_a, dimension in [
            (1., .6, 1., 1., 0), (.3, .1, .4, .3, 3)]:
        alpha = initial*duration/displacement
        for step in range(10001):
            u = step/10000.
            velocity = displacement/duration*(1.-u)**2*(alpha+2.*alpha*u+(30.-15.*alpha)*u*u)
            acceleration = displacement/duration**2*(
                (60.-36.*alpha)*u+(-180.+96.*alpha)*u*u+(120.-60.*alpha)*u*u*u)
            assert -1e-12 <= velocity <= maximum_v+1e-12
            assert abs(acceleration) <= maximum_a+1e-12
        for index in range(1, len(samples)-1):
            numerical_velocity = (samples[index+1][dimension]-samples[index-1][dimension])/(2.*dt)
            assert abs(numerical_velocity-samples.velocities[index][dimension]) < 1e-5
            numerical_acceleration = (samples.velocities[index+1][dimension]-samples.velocities[index-1][dimension])/(2.*dt)
            assert abs(numerical_acceleration-samples.accelerations[index][dimension]) < 2e-5


def test_short_duplicate_and_pure_yaw_segments_are_not_dropped():
    path = [(1., 2., 3., 3.1), (1., 2., 3., 3.1),
            (1.+1e-12, 2., 3., 3.1), (1.+1e-12, 2., 3., -3.1)]
    samples = retime_polyline(path, 1., .2, .05)
    assert [samples[index] for index in samples.vertex_indices] == path
    assert samples.vertex_indices[:3] == [0, 1, 2]
    assert all(state[:3] == path[-1][:3] for state in samples[samples.vertex_indices[2]:])
    assert all(abs(wrap(b[3]-a[3])) <= .2*.05+1e-12 for a, b in zip(samples, samples[1:]))


def test_single_vertex_and_exact_point_budget():
    state = (1., 2., 3., .4)
    singleton = retime_polyline([state], 1., .2, .05, max_points=1)
    assert singleton == [state]
    assert singleton.vertex_indices == [0]
    assert singleton.velocities == [(0.,)*4]
    assert len(retime_polyline([state]*7, 1., .2, .05, max_points=7)) == 7
    with pytest.raises(ValueError, match='^trajectory_point_budget$'):
        retime_polyline([state]*7, 1., .2, .05, max_points=6)
    with pytest.raises(ValueError, match='^trajectory_point_budget$'):
        retime_polyline([(0., 0., 0., 0.), (1., 0., 0., 0.)], 1., .2, 1e-8)
    with pytest.raises(RetimingError, match='^retiming_initial_state_infeasible$') as caught:
        retime_polyline([state], 1., .2, .05, initial_velocity=(.1, 0., 0., 0.))
    assert caught.value.details['branch'] == 'moving_single_vertex'
    assert caught.value.details['context']['segment_index'] is None
    assert caught.value.details['context']['start'] == list(state)


@pytest.mark.parametrize('options', [
    {'speed': 0.}, {'yaw_rate': math.nan}, {'dt': 0.},
    {'max_acceleration': 0.}, {'max_yaw_acceleration': math.inf},
    {'max_points': 20001}, {'max_points': 1.5},
    {'initial_velocity': (0., 0., 0.)},
    {'initial_velocity': (math.nan, 0., 0., 0.)},
])
def test_invalid_parameters(options):
    parameters = dict(speed=1., yaw_rate=.2, dt=.05)
    parameters.update(options)
    with pytest.raises(ValueError):
        retime_polyline([(0., 0., 0., 0.), (1., 0., 0., 0.)], **parameters)
