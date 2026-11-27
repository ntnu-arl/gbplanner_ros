"""Retain the exact rejected call and keep the existing action running."""
import ast
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from trajectory_tools import retime_polyline


def follower():
    source = SCRIPTS / 'path_follower_node.py'
    cls = next(item for item in ast.parse(source.read_text()).body if isinstance(item, ast.ClassDef))
    names = {'profile_start_velocity', 'profile_samples', 'capture_retiming_rejection',
             'reject_continuation', 'continuation_validated'}
    methods = [item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name in names]
    body = ast.ClassDef(name='Follower', bases=[], keywords=[], body=methods, decorator_list=[])
    namespace = {'math': math, 'time': time, 'retime_polyline': retime_polyline}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[body], type_ignores=[])),
                 str(source), 'exec'), namespace)
    node = namespace['Follower']()
    node.trajectory_profile, node.yaw_rate, node.dt = 'quintic_collinear', .15, .05
    node.smooth = True
    node.max_acceleration, node.max_yaw_acceleration, node.max_points = 1., .3, 20000
    node.measured_velocity = [-.2, 0., 0., 0.]
    node.measured_speed, node.measured_yaw_rate = .2, 0.
    node.current, node.odom_stamp, node.frame = [.2, 0., 0., 0.], 3., 'map'
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=3_010_000_000))
    return node, namespace


def test_snapshot_replays_actual_call_after_live_input_changes():
    node, _ = follower()
    connected = [[.2, 0., 0., 0.], [1., 0., 0., 0.]]
    with pytest.raises(ValueError) as rejected:
        node.profile_samples(connected, [], 1., continuation=True)
    call = rejected.value.retiming_input
    connected[0][0] = 99.
    node.measured_velocity[0] = 99.
    encoded = json.dumps(call, allow_nan=False)
    restored = json.loads(encoded)
    assert restored['path'][0][0] == .2
    assert restored['parameters']['initial_velocity'][0] == -.2
    with pytest.raises(ValueError) as replayed:
        retime_polyline(restored['path'], **restored['parameters'])
    assert str(replayed.value) == str(rejected.value) == 'retiming_initial_state_infeasible'


def test_retiming_rejection_is_reported_without_replacing_or_stopping_action():
    node, namespace = follower()
    connected = [[.2, 0., 0., 0.], [1., 0., 0., 0.], [2., 0., 0., 0.]]
    # This test targets the validated callback and its error handling. The
    # geometric preparation/validation itself already has separate checks.
    namespace['prepare_continuation'] = lambda *args: (connected, [])
    namespace['continuation_within_checked'] = lambda *args: True
    handle = SimpleNamespace(is_cancel_requested=False, goal_id=SimpleNamespace(uuid=[1] * 16))
    active = {'sequence': 7, 'stop': False, 'handle': handle, 'generation': 2,
              'connected': [[0., 0., 0., 0.], [1., 0., 0., 0.]],
              'progress': SimpleNamespace(progress=.2), 'speed': 1.}
    completions, publications = [], []
    response = SimpleNamespace(accepted=False, reason='')
    candidate = {'sequence': 7, 'deadline': time.monotonic() + 20., 'request': object(),
                 'extension': [[1., 0., 0., 0.], [2., 0., 0., 0.]], 'checked_path': connected,
                 'validation_stamp_ns': 3_000_000_000, 'validation_attempts': 1,
                 'done': SimpleNamespace(set_result=completions.append), 'response': response}
    node.active, node.continuation, node.revision = active, candidate, 12
    node.state, node.reason = 'EXECUTING', 'trajectory_published'
    node.stats = {'continuation_rejections': 0}
    node.fresh = lambda: True
    node.continuation_budget = lambda *args: None
    node.transition = lambda state, reason: publications.append((state, reason))
    node.publish_trajectory = lambda *args: pytest.fail('Rejected continuation must not publish a trajectory')
    future = SimpleNamespace(result=lambda: SimpleNamespace(valid=True, map_revision=12))
    node.continuation_validated(future, candidate)
    assert node.active is active and not active['stop'] and active['generation'] == 2
    assert node.continuation is None and completions == [response]
    assert response.reason == 'retiming_initial_state_infeasible' and not response.accepted
    assert node.stats['continuation_rejections'] == 1
    assert publications == [('EXECUTING', 'trajectory_published')]
    captured = node.last_continuation_rejection
    assert captured['stage'] == 'retiming' and captured['map_revision'] == 12
    assert captured['retiming_input']['path'] == connected
    assert captured['measured_velocity'] == [-.2, 0., 0., 0.]
    active['connected'][0][0] = 88.
    candidate['checked_path'][0][0] = 77.
    node.current[0] = 66.
    assert captured['previous_path'][0][0] == 0.
    assert captured['checked_path'][0][0] == .2
    assert captured['position'][0] == .2
    json.dumps(captured, allow_nan=False)
