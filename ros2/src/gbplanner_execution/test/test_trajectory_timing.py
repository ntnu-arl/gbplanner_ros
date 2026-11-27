"""Published ROS timing and follower duration share the generated time axis."""
import ast
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from trajectory_tools import TimedSamples, retime_polyline


def publisher_fixture():
    geometry = pytest.importorskip('geometry_msgs.msg')
    trajectory = pytest.importorskip('trajectory_msgs.msg')
    nav = pytest.importorskip('nav_msgs.msg')
    builtin = pytest.importorskip('builtin_interfaces.msg')
    source = Path(__file__).resolve().parents[1] / 'scripts/path_follower_node.py'
    tree = ast.parse(source.read_text())
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef))
    methods = [item for item in cls.body if isinstance(item, ast.FunctionDef)
               and item.name in {'sample_duration', 'publish_trajectory'}]
    pose = next(item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name == 'pose')
    extracted = ast.Module(body=[pose, ast.ClassDef(name='Follower', bases=[], keywords=[],
        body=methods, decorator_list=[])], type_ignores=[])
    namespace = dict(math=math, TimedSamples=TimedSamples, Path=nav.Path,
        Pose=geometry.Pose, PoseStamped=geometry.PoseStamped, Transform=geometry.Transform,
        Twist=geometry.Twist, MultiDOFJointTrajectory=trajectory.MultiDOFJointTrajectory,
        MultiDOFJointTrajectoryPoint=trajectory.MultiDOFJointTrajectoryPoint)
    exec(compile(ast.fix_missing_locations(extracted), str(source), 'exec'), namespace)
    node = namespace['Follower']()
    node.frame, node.joint, node.dt = 'map', 'base_link', .05
    stamp = builtin.Time(sec=17, nanosec=123000000)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: stamp))
    node.messages, node.paths = [], []
    node.commands = SimpleNamespace(publish=node.messages.append)
    node.path_pub = SimpleNamespace(publish=node.paths.append)
    node.stats = {'trajectory_commands': 0, 'hold_commands': 0}
    return node


def test_irregular_vertices_use_real_offsets_in_ros_and_eta():
    node = publisher_fixture()
    path = [(0., 0., 0., 0.), (.37, 0., 0., 0.), (1., 0., 0., 0.)]
    samples = retime_polyline(path, 1., .15, .05, merge_collinear=True)
    assert len(samples)*node.dt != pytest.approx(samples.time_offsets[-1])
    assert node.publish_trajectory(samples) == 17123000000
    message = node.messages[-1]
    times = [p.time_from_start.sec*1000000000+p.time_from_start.nanosec for p in message.points]
    assert times == [round(t*1e9) for t in samples.time_offsets]
    assert all(b > a for a, b in zip(times, times[1:]))
    assert node.sample_duration(samples) == pytest.approx(times[-1]*1e-9, abs=5e-10)
    middle = message.points[samples.vertex_indices[1]]
    assert middle.transforms[0].translation.x == .37
    assert middle.velocities[0].linear.x > 0.
    assert middle.accelerations[0].linear.x != 0.


def test_uniform_upstream_and_hold_keep_existing_clock():
    node = publisher_fixture()
    samples = [(0., 0., 0., 0.), (.2, 0., 0., .1)]
    node.publish_trajectory(samples)
    assert [p.time_from_start.nanosec for p in node.messages[-1].points] == [50000000, 100000000]
    assert node.sample_duration(samples) == .1
    node.publish_trajectory([samples[-1]], hold=True)
    hold = node.messages[-1].points[0]
    assert hold.time_from_start.nanosec == 50000000
    assert hold.velocities[0].linear.x == hold.accelerations[0].linear.x == 0.


def test_unrepresentable_nanosecond_order_is_rejected_before_publish():
    node = publisher_fixture()
    samples = retime_polyline([(0., 0., 0., 0.), (1., 0., 0., 0.)], 1., .15, .05)
    samples.time_offsets[1] = samples.time_offsets[0] + 1e-12
    with pytest.raises(ValueError, match='^trajectory_sample_time$'):
        node.publish_trajectory(samples)
    assert node.messages == []


def test_preserved_origin_keeps_old_header_and_every_sample_offset():
    node = publisher_fixture()
    samples = retime_polyline([(0., 0., 0., 0.), (.37, 0., 0., 0.),
                              (1., 0., 0., 0.)], 1., .15, .05, merge_collinear=True)
    origin = 10_987_654_321
    assert node.publish_trajectory(samples, origin_stamp_ns=origin) == origin
    message, path = node.messages[-1], node.paths[-1]
    assert message.header.stamp.sec == path.header.stamp.sec == 10
    assert message.header.stamp.nanosec == path.header.stamp.nanosec == 987654321
    assert [p.time_from_start.sec*10**9+p.time_from_start.nanosec for p in message.points] == [
        round(t*1.e9) for t in samples.time_offsets]
    assert node.stats == {'trajectory_commands': 1, 'hold_commands': 0}


def test_hold_cannot_reuse_a_trajectory_origin():
    node = publisher_fixture()
    samples = retime_polyline([(0., 0., 0., 0.), (1., 0., 0., 0.)], 1., .15, .05)
    with pytest.raises(ValueError, match='^invalid_preserved_trajectory_origin$'):
        node.publish_trajectory(samples, hold=True, origin_stamp_ns=10_987_654_321)
    assert node.messages == [] and node.paths == []
    assert node.stats == {'trajectory_commands': 0, 'hold_commands': 0}
