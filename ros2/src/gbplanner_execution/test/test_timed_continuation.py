"""Append-only timing preserves the commanded prefix and all exact vertices."""
import copy
from pathlib import Path
import struct
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from timed_continuation import prepare_timed_continuation
from trajectory_tools import TimedSamples, allocate_yaw, retime_polyline


PARAMETERS = dict(speed=.8, yaw_rate=.3, dt=.05, max_acceleration=.7,
                  max_yaw_acceleration=.2, max_points=20000,
                  smooth_yaw=True, merge_collinear=True)


def source():
    path = [(0., 0., 0., 0.), (.23, 0., 0., .046), (1.1, 0., 0., .22)]
    samples = retime_polyline(path, .8, .3, .05, .7, .2, merge_collinear=True)
    # Exercise independent copies of mutable rows, including signed zero.
    samples[:] = [list(row) for row in samples]
    samples.velocities = [list(row) for row in samples.velocities]
    samples.accelerations = [list(row) for row in samples.accelerations]
    samples.velocities[-1][0] = -0.
    return path, samples


def prefix_bytes(samples, n):
    values = [value for rows in (samples[:n], samples.velocities[:n], samples.accelerations[:n])
              for row in rows for value in row] + samples.time_offsets[:n]
    return struct.pack('!' + 'd' * len(values), *values)


def test_prefix_and_header_relative_times_are_unchanged_and_independently_owned():
    path, old = source()
    extension = [(1.1005, 0., 0., -2.), (1.4, 0., 0., 2.),
                 (1.4, 0., 0., 2.2), (1.4, .3, 0., 2.3)]
    before, count = copy.deepcopy(old), len(old)
    full, combined, tail_path = prepare_timed_continuation(path, old, extension, **PARAMETERS)
    assert combined is not old
    assert prefix_bytes(combined, count) == prefix_bytes(before, count)
    assert combined.vertex_indices[:len(path)] == before.vertex_indices
    assert all(combined[i] is not old[i] for i in range(count))
    assert combined.velocities[0] is not old.velocities[0]
    assert combined.accelerations[0] is not old.accelerations[0]
    assert combined.time_offsets is not old.time_offsets
    assert combined.vertex_indices is not old.vertex_indices
    assert old == before and old.velocities == before.velocities
    assert tail_path == allocate_yaw([tuple(before[-1])] + extension[1:], .8, .3)
    assert tail_path[0] == tuple(before[-1])
    assert full == path + tail_path[1:]
    assert [combined[i] for i in combined.vertex_indices] == full
    assert old.velocities[old.vertex_indices[1]][0] > 0.
    assert combined.velocities[count-1] == (-0., 0., 0., 0.)
    assert combined.accelerations[count-1] == (0.,) * 4
    reference = retime_polyline(tail_path, .8, .3, .05, .7, .2, merge_collinear=True)
    assert combined[count:] == reference[1:]
    assert combined.velocities[count:] == reference.velocities[1:]
    assert combined.accelerations[count:] == reference.accelerations[1:]
    assert combined.time_offsets[count:] == [old.time_offsets[-1] + (t-reference.time_offsets[0])
                                             for t in reference.time_offsets[1:]]
    stamps = [round(t*1e9) for t in combined.time_offsets]
    assert all(b > a for a, b in zip(stamps, stamps[1:]))
    old[0][0] = 99.
    old.velocities[0][0] = 99.
    old.accelerations[0][0] = 99.
    old.time_offsets[0] = 99.
    old.vertex_indices[0] = 99
    assert prefix_bytes(combined, count) == prefix_bytes(before, count)
    assert combined.vertex_indices[:len(path)] == before.vertex_indices


@pytest.mark.parametrize('field,axis', [('velocities', 0), ('accelerations', 3)])
def test_nonzero_old_endpoint_derivative_is_rejected_even_if_tiny(field, axis):
    path, old = source()
    getattr(old, field)[-1][axis] = 1e-20
    with pytest.raises(ValueError, match='^timed_continuation_endpoint_not_stationary$'):
        prepare_timed_continuation(path, old, [path[-1], (1.5, 0., 0., .2)], **PARAMETERS)


def test_old_endpoint_and_extension_root_are_checked_separately():
    path, old = source()
    mismatched = path[:-1] + [(*path[-1][:3], path[-1][3] + 1e-10)]
    with pytest.raises(ValueError, match='^timed_continuation_endpoint_mismatch$'):
        prepare_timed_continuation(mismatched, old, [path[-1], (1.5, 0., 0., .2)], **PARAMETERS)
    with pytest.raises(ValueError, match='^continuation_root_mismatch$'):
        prepare_timed_continuation(path, old, [(1.1011, 0., 0., .22), (1.5, 0., 0., .2)], **PARAMETERS)


def test_total_history_budget_is_retained_across_repeated_extensions():
    path, old = source()
    extension = [path[-1], (1.4, 0., 0., .3)]
    full, combined, _ = prepare_timed_continuation(path, old, extension, **PARAMETERS)
    cap = len(combined)
    _, exact, _ = prepare_timed_continuation(path, old, extension, **dict(PARAMETERS, max_points=cap))
    assert prefix_bytes(exact, cap) == prefix_bytes(combined, cap)
    with pytest.raises(ValueError, match='^trajectory_point_budget$'):
        prepare_timed_continuation(path, old, extension, **dict(PARAMETERS, max_points=cap-1))
    with pytest.raises(ValueError, match='^trajectory_point_budget$'):
        prepare_timed_continuation(full, combined, [full[-1], (1.7, 0., 0., .3)],
                                   **dict(PARAMETERS, max_points=cap))
    longer_path, longer, _ = prepare_timed_continuation(
        full, combined, [full[-1], (1.7, 0., 0., .3)], **PARAMETERS)
    assert prefix_bytes(longer, len(combined)) == prefix_bytes(combined, len(combined))
    assert longer.vertex_indices[:len(full)] == combined.vertex_indices
    assert [longer[i] for i in longer.vertex_indices] == longer_path


@pytest.mark.parametrize('kind,reason', [('shape', 'trajectory_sample_shape'),
                                       ('missing_field', 'trajectory_sample_shape'),
                                       ('vertex', 'trajectory_sample_vertices'),
                                       ('time', 'trajectory_sample_time')])
def test_malformed_old_sample_contract_is_rejected(kind, reason):
    path, old = source()
    if kind == 'shape':
        old.accelerations.pop()
    elif kind == 'missing_field':
        del old.accelerations
    elif kind == 'vertex':
        old.vertex_indices[1] += 1
    else:
        old.time_offsets[1] = old.time_offsets[0] + 1e-12
    with pytest.raises(ValueError, match='^' + reason + '$'):
        prepare_timed_continuation(path, old, [path[-1], (1.5, 0., 0., .2)], **PARAMETERS)


def test_nanosecond_collision_after_offset_is_rejected_without_retiming_prefix():
    path = [(0.,) * 4]
    old = TimedSamples()
    old.extend(path)
    old.velocities = [(0.,) * 4]
    old.accelerations = [(0.,) * 4]
    old.vertex_indices = [0]
    old.time_offsets = [1e8]
    # The local 1 ns duplicate segment is resolvable before adding 1e8 s;
    # IEEE double precision cannot represent that increment afterwards.
    with pytest.raises(ValueError, match='^trajectory_sample_time$'):
        prepare_timed_continuation(path, old, path * 2,
                                   **dict(PARAMETERS, dt=1e-9, smooth_yaw=False, merge_collinear=False))
    assert old.time_offsets == [1e8]
