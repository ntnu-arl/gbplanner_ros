import math
import os
from pathlib import Path
import random
import subprocess
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from trajectory_tools import allocate_yaw, interpolate, reconnect, prepare, prepare_continuation, wrap


def reference(mode, path, current=(0.,0.,0.,0.), speed=1., yaw_rate=.15, dt=.05):
    payload = [str(mode), str(speed), str(yaw_rate), str(dt), str(len(path))]
    payload.extend(str(v) for state in [current]+path for v in state)
    output = subprocess.run([os.environ['PCI_REFERENCE']], input=' '.join(payload),
        text=True, capture_output=True, check=True, timeout=5).stdout.splitlines()
    if int(output[0]) == -1:
        return None
    result = [tuple(map(float, line.split())) for line in output[1:]]
    assert len(result) == int(output[0]); return result


def same(a, b):
    assert math.dist(a[:3], b[:3]) < 1e-10
    assert abs(wrap(a[3]-b[3])) < 1e-10


def test_yaw_allocation_matches_extracted_original():
    for seed in range(30):
        rng = random.Random(seed)
        path = [tuple(rng.uniform(-3,3) for _ in range(4)) for _ in range(6)]
        for speed, rate in [(1.,.15), (.4,.7)]:
            port = allocate_yaw(path, speed, rate)
            original = reference(0, path, speed=speed, yaw_rate=rate)
            for a,b in zip(port,original):same(a,b)


def test_interpolation_preserves_original_interior_and_repairs_endpoint():
    repaired = 0
    for seed in range(30):
        rng=random.Random(seed)
        path=[tuple(rng.uniform(-2,2) for _ in range(4)) for _ in range(2)]
        original=reference(1,path,speed=.8,yaw_rate=.6,dt=.07)
        port=interpolate(path,.8,.6,.07)
        assert len(port) in [len(original),len(original)+1]
        for a,b in zip(port,original):same(a,b)
        same(port[-1],path[-1]);repaired+=len(port)>len(original)
    assert repaired>0


def test_reconnect_matches_original_short_distance_branches():
    path=[(0.,0.,0.,0.),(1.,0.,0.,.3),(1.,1.,0.,.5)]
    for current in [(-.05,0.,0.,.2),(-.8,0.,0.,.2),(.8,0.,0.,.2),(0.,.7,0.,.2),(-2.,0.,0.,0.)]:
        original=reference(2,path,current=current)
        if original is None:
            with pytest.raises(ValueError):reconnect(path,current)
        else:
            port=reconnect(path,current);assert len(port)==len(original)
            for a,b in zip(port,original):same(a,b)


def test_segment_endpoints_and_speed_bounds():
    path=[(0.,0.,0.,3.1),(.233,.144,-.08,-3.05),(.81,.2,.3,2.8)]
    samples=interpolate(path,.7,.3,.05)
    for endpoint in path:assert any(math.dist(p,endpoint)<1e-10 for p in samples)
    for a,b in zip(samples,samples[1:]):
        assert math.dist(a[:3],b[:3])/.05 <= .7+1e-10
        assert abs(wrap(b[3]-a[3]))/.05 <= .3+1e-10


def test_zero_length_and_pure_rotation():
    same_state=(1.,2.,3.,.4)
    samples=interpolate([same_state,same_state],1.,.2,.05)
    assert all(p==same_state for p in samples)
    rotation=interpolate([(0.,0.,0.,3.1),(0.,0.,0.,-3.1)],1.,.2,.05)
    same(rotation[-1],(0.,0.,0.,-3.1))
    assert all(p[:3]==(0.,0.,0.) for p in rotation)
    assert all(abs(wrap(b[3]-a[3]))<=.010000001 for a,b in zip(rotation,rotation[1:]))


def test_invalid_inputs_and_resource_budget():
    path=[(0.,0.,0.,0.),(1.,0.,0.,0.)]
    for args in [(0.,.2,.05),(1.,0.,.05),(1.,.2,-1.),(math.nan,.2,.05)]:
        with pytest.raises(ValueError):interpolate(path,*args)
    with pytest.raises(ValueError):interpolate(path,1.,.2,.000001,100)
    with pytest.raises(ValueError):interpolate([],1.,.2,.05)
    with pytest.raises(ValueError):prepare([(math.nan,0.,0.,0.)],path[0])


def test_original_yaw_relaxation_and_start_pose_preserved():
    current=(0.,0.,0.,1.)
    adjusted,samples=prepare([(0.,0.,0.,0.),(.5,0.,0.,-2.)],current)
    same(adjusted[0],current);same(samples[0],current)
    assert abs(wrap(adjusted[-1][3]-1.)) <= .075000001
    assert samples[-1][:3]==(.5,0.,0.)


def test_continuation_keeps_far_forward_corners_and_exact_endpoint():
    previous=[(0.,0.,0.,0.),(3.,0.,0.,.2),(3.,2.,0.,.4)]
    path=[previous[-1],(4.,2.,1.,.6)]
    current=(.8,0.,0.,.1)
    assert math.dist(current[:3],path[0][:3])>1.5
    connected,samples=prepare_continuation(previous,.8,path,current)
    assert [p[:3] for p in connected]==[current[:3],(3.,0.,0.),(3.,2.,0.),(4.,2.,1.)]
    same(samples[0],current)
    for corner in connected:
        assert any(math.dist(p[:3],corner[:3])<1e-10 for p in samples)
    assert samples[-1][:3]==path[-1][:3]
    for a,b in zip(samples,samples[1:]):
        assert math.dist(a[:3],b[:3]) <= .05+1e-10
        assert abs(wrap(b[3]-a[3])) <= .0075+1e-10


def test_continuation_does_not_command_behind_cursor_guard():
    previous=[(0.,0.,0.,0.),(1.,0.,0.,0.),(2.,0.,0.,0.)]
    current=(1.2,0.,0.,0.)
    connected,samples=prepare_continuation(previous,1.2,[previous[-1],(3.,0.,0.,0.)],current)
    assert connected==[current,previous[-1],(3.,0.,0.,0.)]
    assert all(p[0]>=1.2 for p in samples)
    assert all(b[0]>=a[0] for a,b in zip(samples,samples[1:]))


def test_continuation_keeps_order_at_self_crossing_and_deduplicates_xyz():
    previous=[(0.,0.,0.,0.),(0.,0.,0.,.1),(2.,0.,0.,.2),
              (2.,2.,0.,.3),(0.,2.,0.,.4),(0.,0.,0.,.5)]
    connected,_=prepare_continuation(previous,0.,[previous[-1],(-1.,0.,0.,.6)],previous[0])
    assert [p[:3] for p in connected]==[p[:3] for p in previous[1:]]+[(-1.,0.,0.)]
    assert len(connected)==6


def test_continuation_at_old_endpoint_moves_only_into_new_segment():
    previous=[(0.,0.,0.,0.),(2.,0.,0.,0.)]
    current=(2.,0.,0.,.2)
    connected,_=prepare_continuation(previous,2.,[previous[-1],(3.,0.,0.,.3)],current)
    assert [p[:3] for p in connected]==[(2.,0.,0.),(3.,0.,0.)]
    same(connected[0],current)


def test_continuation_rejects_mismatched_root_and_stale_cursor():
    previous=[(0.,0.,0.,0.),(2.,0.,0.,0.)]
    # Equal XY is insufficient: vertical endpoint separation must be rejected.
    with pytest.raises(ValueError,match='continuation_root_mismatch'):
        prepare_continuation(previous,.5,[(2.,0.,.01,0.),(3.,0.,0.,0.)],(.5,0.,0.,0.))
    path=[previous[-1],(3.,0.,0.,0.)]
    with pytest.raises(ValueError,match='continuation_cursor_stale'):
        prepare_continuation(previous,.5,path,(1.,0.,0.,0.))
    for progress in [-.1,2.1,math.nan,math.inf]:
        with pytest.raises(ValueError,match='invalid_continuation_progress'):
            prepare_continuation(previous,progress,path,(.5,0.,0.,0.))


def test_continuation_commit_stays_in_checked_connector():
    from trajectory_tools import continuation_within_checked
    checked=[(0.,0.,0.,0.),(1.,0.,0.,0.),(1.,1.,0.,0.)]
    assert continuation_within_checked(checked,[(.4,.015,0.,.1)]+checked[1:],.02)
    assert not continuation_within_checked(checked,[(.4,.03,0.,.1)]+checked[1:],.02)
    assert not continuation_within_checked(checked,[(1.,.1,0.,.1),checked[-1]],.02)
    changed=[(.4,0.,0.,0.),(1.,.01,0.,0.),checked[-1]]
    assert not continuation_within_checked(checked,changed,.02)
