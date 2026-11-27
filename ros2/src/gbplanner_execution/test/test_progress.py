import math
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'gbplanner_local/scripts'))
from path_progress import PathProgress


def test_measured_progress_trims_only_traversed_geometry():
    path=PathProgress([(0,0,0),(1,0,0),(2,0,0),(3,0,0)])
    for x in [.5,1.,1.5,2.]:path.update((x,0,0))
    remaining=path.remaining((2,0,0))
    assert remaining[:4]==[(2,0,0),(3,0,0),(2,0,0),(1.75,0,0)]
    assert remaining[-1]==(3,0,0) and (1,0,0) not in remaining


def test_stationary_or_lagging_robot_does_not_advance_on_time():
    path=PathProgress([(0,0,0),(3,0,0)])
    for _ in range(100):path.update((0,0,0))
    assert path.progress==0 and path.remaining((0,0,0))[1]==(3,0,0)


def test_self_intersection_never_selects_distant_later_branch():
    path=PathProgress([(0,0,0),(2,0,0),(2,2,0),(0,2,0),(0,0,0),(3,0,0)])
    for _ in range(10):path.update((.05,0,0))
    assert path.progress<.1
    assert (2,2,0) in path.remaining((.05,0,0))


def test_guard_preserves_corner_and_full_future_route():
    path=PathProgress([(0,0,0),(1,0,0),(1,1,0),(2,1,0)])
    for point in [(0.5,0,0),(.95,0,0),(1,.1,0)]:path.update(point)
    remaining=path.remaining((1,.1,0))
    assert (1,0,0) in remaining and (1,1,0) in remaining and remaining[-1]==(2,1,0)


def test_offset_connector_does_not_cut_back_to_guard():
    path=PathProgress([(0,0,0),(1,0,0),(1,1,0),(2,1,0)])
    for point in [(0.5,0,0),(.95,0,0),(1.08,.1,0)]:path.update(point)
    route=path.remaining((1.08,.1,0))
    # The only off-polyline edge joins the next original forward vertex.
    # Both traversals of the corner and all future geometry are still checked.
    assert route[:3]==[(1.08,.1,0),(1,1,0),(1,0,0)]
    assert route.count((1,0,0))==2 and route[-2:]==[(1,1,0),(2,1,0)]
    for a,b in zip(route[1:],route[2:]):
        assert sum(abs(x-y)>1e-9 for x,y in zip(a,b))<=1


def test_deviation_jumps_and_backtracking_fail_closed():
    for point,error in [((0,.3,0),'off_commanded'),((1.5,0,0),'progress_jump')]:
        path=PathProgress([(0,0,0),(3,0,0)])
        with pytest.raises(ValueError,match=error):path.update(point)
    path=PathProgress([(0,0,0),(3,0,0)])
    path.update((.5,0,0));path.update((1.,0,0))
    with pytest.raises(ValueError):path.update((.6,0,0))


def test_duplicate_and_single_positions_keep_endpoint():
    path=PathProgress([(0,0,0),(0,0,0),(.1,0,0),(.1,0,0)])
    path.update((.1,0,0));assert path.remaining((.1,0,0))[-1]==(.1,0,0)
    single=PathProgress([(0,0,0),(0,0,0)])
    single.update((0,0,0));assert single.remaining((0,0,0))==[(0,0,0),(0,0,0)]
    with pytest.raises(ValueError):PathProgress([(math.nan,0,0)])
