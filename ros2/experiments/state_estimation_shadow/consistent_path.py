"""Construct a kinematically consistent forward path for a front control point.

The fixed planar model is C = B + ell * [cos(psi), sin(psi)] with zero
lateral velocity at C. For a straight B-path segment with tangent theta and
B arc length s, this requires dpsi/ds = sin(psi - theta) / ell. The terminal
heading supplies the boundary condition; integrating backwards is stable.

Only reference headings change. This is an experimental path construction,
not a change to the physical robot, the estimator or the production tracker.
The initial heading is an output, not an independently satisfiable boundary
condition, and its difference from the input heading is reported explicitly.
"""
from __future__ import annotations

import hashlib
import json
import math
from typing import Iterable


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def consistent_base_poses(samples: Iterable, offset_m: float):
    """Return ``(B poses, metadata)`` for poses ordered as ``(x, y, z, yaw)``.

    XYZ values, sampling order, repeated points and the final yaw are retained.
    Each nonzero planar interval is solved exactly, including across tangent
    changes. Repeated XY points inherit the same solved yaw; they introduce
    neither artificial travel nor an in-place rotation of B. Forward motion
    requires cos(psi - theta) > 0 everywhere. A route which violates that
    requirement is rejected rather than silently reversed or reshaped.
    """
    ell = float(offset_m)
    if not math.isfinite(ell) or ell <= 0.0:
        raise ValueError('offset_m must be finite and strictly positive')
    original = []
    for i, sample in enumerate(samples):
        if len(sample) != 4:
            raise ValueError(f'sample {i} must contain x, y, z, yaw')
        pose = tuple(float(value) for value in sample)
        if not all(math.isfinite(value) for value in pose):
            raise ValueError(f'sample {i} contains a nonfinite value')
        original.append(pose)
    if not original:
        raise ValueError('samples must not be empty')

    yaw = [0.0] * len(original)
    yaw[-1] = original[-1][3]
    intervals = []
    repeated = 0
    for i in range(len(original) - 2, -1, -1):
        dx = original[i + 1][0] - original[i][0]
        dy = original[i + 1][1] - original[i][1]
        ds = math.hypot(dx, dy)
        if ds == 0.0:
            yaw[i] = yaw[i + 1]
            repeated += 1
            continue
        theta = math.atan2(dy, dx)
        delta_end = _wrap(yaw[i + 1] - theta)
        # Backwards integration shrinks |delta| on the forward branch, so its
        # endpoint is the minimum forward speed ratio on this whole interval.
        forward_ratio = math.cos(delta_end)
        if forward_ratio <= 1.e-12:
            raise ValueError(
                f'interval {i} has no strictly forward solution: '
                f'endpoint heading differs from its B tangent by {delta_end:g} rad')
        tangent_half = math.tan(delta_end / 2.0)
        delta_start = 2.0 * math.atan(math.exp(-ds / ell) * tangent_half)
        # Keep the existing angle branch, including a terminal yaw outside
        # [-pi, pi], rather than introducing a wrap discontinuity at a vertex.
        yaw[i] = yaw[i + 1] + delta_start - delta_end
        delta_mid = 2.0 * math.atan(math.exp(-0.5 * ds / ell) * tangent_half)
        psi_mid = yaw[i + 1] + delta_mid - delta_end
        psi_prime = math.sin(delta_mid) / ell
        # dC/ds = tangent_B + ell * psi_prime * normal(psi).
        dc_x = math.cos(theta) - ell * psi_prime * math.sin(psi_mid)
        dc_y = math.sin(theta) + ell * psi_prime * math.cos(psi_mid)
        lateral_residual = -math.sin(psi_mid) * dc_x + math.cos(psi_mid) * dc_y
        intervals.append((i, ds, psi_mid, lateral_residual, forward_ratio,
                          abs(math.tan(delta_end)) / ell))

    corrected = [(pose[0], pose[1], pose[2], heading)
                 for pose, heading in zip(original, yaw)]
    center = [(x + ell * math.cos(heading), y + ell * math.sin(heading))
              for x, y, _, heading in corrected]
    chord_errors = []
    for i, ds, psi_mid, residual, forward_ratio, curvature in intervals:
        dx = center[i + 1][0] - center[i][0]
        dy = center[i + 1][1] - center[i][1]
        length = math.hypot(dx, dy)
        chord_errors.append((length, _wrap(math.atan2(dy, dx) - psi_mid)))
    chord_length = sum(length for length, _ in chord_errors)
    initial_delta = _wrap(yaw[0] - original[0][3])
    metadata = {
        'method': 'Exact backward integration of the fixed front-center nonholonomic constraint',
        'equation': 'dpsi/ds_B = sin(psi - theta_B) / offset_m; terminal yaw prescribed',
        'offset_m': ell,
        'input_base_sampled_path_sha256': hashlib.sha256(
            json.dumps(original).encode()).hexdigest(),
        'output_base_sampled_path_sha256': hashlib.sha256(
            json.dumps(corrected).encode()).hexdigest(),
        'sample_count': len(corrected),
        'nonzero_planar_intervals': len(intervals),
        'repeated_xy_intervals': repeated,
        'base_xyz_preserved': all(a[:3] == b[:3] for a, b in zip(original, corrected)),
        'terminal_yaw_preserved': corrected[-1][3] == original[-1][3],
        'input_initial_yaw_rad': original[0][3],
        'output_initial_yaw_rad': yaw[0],
        'initial_yaw_change_rad': initial_delta,
        'initial_center_position_change_m': 2.0 * ell * abs(math.sin(initial_delta / 2.0)),
        'minimum_forward_center_speed_per_base_speed': min(
            (row[4] for row in intervals), default=None),
        'maximum_center_curvature_per_m': max((row[5] for row in intervals), default=0.0),
        'exact_midpoint_lateral_constraint_residual_max_per_base_speed': max(
            (abs(row[3]) for row in intervals), default=0.0),
        'sampled_center_chord_heading_error_rms_rad': math.sqrt(
            sum(length * error ** 2 for length, error in chord_errors) / chord_length
        ) if chord_length else 0.0,
        'sampled_center_chord_heading_error_max_abs_rad': max(
            (abs(error) for _, error in chord_errors), default=0.0),
        'sampling_note': 'The continuous segment solution obeys the constraint exactly; '
                         'a finite center chord only approximates its midpoint tangent.',
        'boundary_note': 'The final yaw is prescribed. The solved initial yaw can differ '
                         'from the physical starting yaw; that mismatch is not suppressed.',
        'scope': 'Fixed planar turning-center hypothesis; no online truth input or model identification',
    }
    return corrected, metadata
