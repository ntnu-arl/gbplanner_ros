"""Append a new timed tail while retaining an already commanded trajectory.

The caller owns the original trajectory header, action identity and collision
validation. This pure helper neither rebases time at the current robot pose nor
discards past or unexecuted samples. It is intended for explicit header-aware
execution; ordinary continuation keeps its existing implementation.
"""
import math

from trajectory_tools import TimedSamples, allocate_yaw, retime_polyline


_MAX_DURATION_NS = (2**31) * 1_000_000_000 - 1


def _state(state):
    try:
        return (len(state) == 4 and
                all(not isinstance(value, bool) and math.isfinite(value) for value in state))
    except (TypeError, ValueError, OverflowError):
        return False


def _path(path):
    return isinstance(path, (list, tuple)) and bool(path) and all(_state(p) for p in path)


def _copy_states(states):
    # tuple(existing_tuple) can return the same object. Construct each row so
    # even mutable input rows have no alias into the returned trajectory.
    return [tuple(value for value in state) for state in states]


def _check_times(offsets):
    previous = 0
    for value in offsets:
        try:
            scaled = value * 1.e9
            valid = (not isinstance(value, bool) and math.isfinite(value) and
                     value > 0. and math.isfinite(scaled))
            encoded = round(scaled) if valid else 0
        except (TypeError, ValueError, OverflowError):
            raise ValueError('trajectory_sample_time') from None
        if not valid or not previous < encoded <= _MAX_DURATION_NS:
            raise ValueError('trajectory_sample_time')
        previous = encoded


def _check_previous(path, samples):
    if not isinstance(samples, TimedSamples) or not samples:
        raise ValueError('trajectory_sample_shape')
    arrays = [getattr(samples, name, None)
              for name in ('velocities', 'accelerations', 'time_offsets')]
    if any(not isinstance(array, (list, tuple)) or len(array) != len(samples)
           for array in arrays):
        raise ValueError('trajectory_sample_shape')
    if any(not _state(row) for array in [samples, samples.velocities, samples.accelerations]
           for row in array):
        raise ValueError('trajectory_sample_shape')
    if tuple(path[-1]) != tuple(samples[-1]):
        raise ValueError('timed_continuation_endpoint_mismatch')
    indices = getattr(samples, 'vertex_indices', None)
    if (not isinstance(indices, (list, tuple)) or len(indices) != len(path) or
            any(not isinstance(i, int) or isinstance(i, bool) or not 0 <= i < len(samples)
                for i in indices) or indices[0] != 0 or indices[-1] != len(samples) - 1 or
            any(b <= a for a, b in zip(indices, indices[1:])) or
            any(tuple(samples[i]) != tuple(p) for p, i in zip(path, indices))):
        raise ValueError('trajectory_sample_vertices')
    _check_times(samples.time_offsets)
    # A tail can be independently retimed only at the exact rest boundary of
    # the old reference. Never silently zero a small nonzero derivative.
    if any(value != 0. for row in (samples.velocities[-1], samples.accelerations[-1])
           for value in row):
        raise ValueError('timed_continuation_endpoint_not_stationary')


def prepare_timed_continuation(previous_path, previous_samples, extension,
                               speed, yaw_rate, dt, max_acceleration,
                               max_yaw_acceleration, max_points, smooth_yaw,
                               merge_collinear):
    """Return ``(full_path, combined_TimedSamples, tail_path)``.

    All old poses, derivatives, time offsets and vertex indices are copied
    unchanged. Only ``[old_endpoint] + extension[1:]`` is yaw-allocated and
    retimed, from zero velocity and acceleration. The extension's first XYZ
    must be within 1 mm of that shared endpoint; its yaw is represented by the
    existing endpoint, as in ``prepare_continuation``.

    ``max_points`` bounds the entire history plus tail (at most 20,000), not
    just the unfinished part. Invalid input raises ValueError with a stable
    reason; errors from the existing retimer are propagated unchanged.
    """
    try:
        valid_parameters = all(not isinstance(value, bool) and math.isfinite(value) and value > 0.
                               for value in (speed, yaw_rate, dt, max_acceleration, max_yaw_acceleration))
    except (TypeError, ValueError, OverflowError):
        valid_parameters = False
    if (not valid_parameters or not isinstance(max_points, int) or isinstance(max_points, bool) or
            not 1 <= max_points <= 20000 or not isinstance(smooth_yaw, bool) or
            not isinstance(merge_collinear, bool)):
        raise ValueError('invalid_trajectory_parameters')
    if not _path(previous_path) or not _path(extension) or len(extension) < 2:
        raise ValueError('invalid_continuation_path')
    if isinstance(previous_samples, TimedSamples) and len(previous_samples) > max_points:
        raise ValueError('trajectory_point_budget')
    _check_previous(previous_path, previous_samples)
    if math.dist(previous_samples[-1][:3], extension[0][:3]) > .001:
        raise ValueError('continuation_root_mismatch')

    # One endpoint sample belongs to both trajectories. No other sample or
    # repeated/sub-millimetre input vertex may be removed to satisfy the cap.
    available = max_points - len(previous_samples) + 1
    if available < len(extension):
        raise ValueError('trajectory_point_budget')
    tail_path = _copy_states([previous_samples[-1], *extension[1:]])
    if smooth_yaw:
        tail_path = allocate_yaw(tail_path, speed, yaw_rate)
    tail = retime_polyline(tail_path, speed, yaw_rate, dt,
                          max_acceleration=max_acceleration,
                          max_yaw_acceleration=max_yaw_acceleration,
                          max_points=available, initial_velocity=(0.,) * 4,
                          merge_collinear=merge_collinear)
    combined = TimedSamples()
    combined.extend(_copy_states(previous_samples))
    combined.extend(_copy_states(tail[1:]))
    combined.velocities = _copy_states(previous_samples.velocities) + _copy_states(tail.velocities[1:])
    combined.accelerations = _copy_states(previous_samples.accelerations) + _copy_states(tail.accelerations[1:])
    combined.time_offsets = list(previous_samples.time_offsets)
    old_end = previous_samples.time_offsets[-1]
    tail_start = tail.time_offsets[0]
    combined.time_offsets.extend(old_end + (at - tail_start) for at in tail.time_offsets[1:])
    combined.vertex_indices = list(previous_samples.vertex_indices)
    combined.vertex_indices.extend(len(previous_samples) - 1 + i for i in tail.vertex_indices[1:])
    # Adding a large old offset can collapse otherwise distinct tail times at
    # ROS nanosecond resolution. Reject rather than shift or delete an event.
    _check_times(combined.time_offsets)
    full_path = _copy_states(previous_path) + _copy_states(tail_path[1:])
    return full_path, combined, tail_path
