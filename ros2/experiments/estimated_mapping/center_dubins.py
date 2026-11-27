#!/usr/bin/env python3
"""Construct the six Dubins families at the fixed forward control point C.

The input/output robot pose is B=(x,y,z,yaw), with C=B+ell*e(yaw).
Planning in C's SE(2) state fixes both endpoint B poses and the current yaw;
there is no preparatory turn that changes B before a second global search.

This module performs planar geometry only. Every candidate is a NEW geometric
path, not a claim to retain the previous planner polyline. The caller must
project its B height using the original terrain model and validate its full B
sweep with the native validator before execution. If all six candidates fail
those checks, this connector has no solution; it is not a complete obstacle
planner. Curvature is bounded but changes discontinuously at segment joins.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

TAU = 2.0 * math.pi
FAMILIES = ('LSL', 'RSR', 'LSR', 'RSL', 'RLR', 'LRL')
DEFAULT_OFFSET_M = .2159
DEFAULT_RADIUS_M = .6
DEFAULT_SAMPLE_STEP_M = .02


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def mod2pi(angle):
    return angle % TAU


def _pose(values, label):
    if len(values) != 4:
        raise ValueError(label+' must contain x, y, z, yaw')
    result = tuple(float(v) for v in values)
    if not all(math.isfinite(v) for v in result):
        raise ValueError(label+' must be finite')
    return result


def _normalized_lengths(family, alpha, beta, distance):
    """Closed-form nonnegative segment lengths in units of turning radius.

    L/R entries are turning angles; S entries are straight lengths divided by
    radius. Domain failures exclude that family. Only roundoff-sized negative
    radicands or acos overshoots are clamped at degenerate tangencies.
    """
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    cab, d = math.cos(alpha-beta), distance
    tolerance = 1.e-12 * max(1., d*d)

    def root(value):
        if value < -tolerance:
            return None
        return math.sqrt(max(0., value))

    if family == 'LSL':
        p = root(2.+d*d-2.*cab+2.*d*(sa-sb))
        if p is None:
            return None
        tangent = math.atan2(cb-ca, d+sa-sb)
        return mod2pi(-alpha+tangent), p, mod2pi(beta-tangent)
    if family == 'RSR':
        p = root(2.+d*d-2.*cab+2.*d*(-sa+sb))
        if p is None:
            return None
        tangent = math.atan2(ca-cb, d-sa+sb)
        return mod2pi(alpha-tangent), p, mod2pi(-beta+tangent)
    if family == 'LSR':
        p = root(-2.+d*d+2.*cab+2.*d*(sa+sb))
        if p is None:
            return None
        tangent = math.atan2(-ca-cb, d+sa+sb)-math.atan2(-2., p)
        return mod2pi(-alpha+tangent), p, mod2pi(-beta+tangent)
    if family == 'RSL':
        p = root(d*d-2.+2.*cab-2.*d*(sa+sb))
        if p is None:
            return None
        tangent = math.atan2(ca+cb, d-sa-sb)-math.atan2(2., p)
        return mod2pi(alpha-tangent), p, mod2pi(beta-tangent)
    if family == 'RLR':
        cosine = (6.-d*d+2.*cab+2.*d*(sa-sb))/8.
        if abs(cosine) > 1.+tolerance:
            return None
        p = mod2pi(TAU-math.acos(max(-1., min(1., cosine))))
        t = mod2pi(alpha-math.atan2(ca-cb, d-sa+sb)+p/2.)
        return t, p, mod2pi(alpha-beta-t+p)
    if family == 'LRL':
        cosine = (6.-d*d+2.*cab+2.*d*(-sa+sb))/8.
        if abs(cosine) > 1.+tolerance:
            return None
        p = mod2pi(TAU-math.acos(max(-1., min(1., cosine))))
        t = mod2pi(-alpha-math.atan2(ca-cb, d+sa-sb)+p/2.)
        return t, p, mod2pi(beta-alpha-t+p)
    raise ValueError('Unknown Dubins family')


def _advance(center_xy_yaw, distance, curvature):
    """Exact constant-curvature integration from a segment's initial pose."""
    x, y, yaw = center_xy_yaw
    if curvature == 0.:
        return x+distance*math.cos(yaw), y+distance*math.sin(yaw), yaw
    end_yaw = yaw+curvature*distance
    return (x+(math.sin(end_yaw)-math.sin(yaw))/curvature,
            y+(math.cos(yaw)-math.cos(end_yaw))/curvature, end_yaw)


def _sample(family, lengths, start, goal, ell, radius, step):
    start_center = (start[0]+ell*math.cos(start[3]),
                    start[1]+ell*math.sin(start[3]), start[3])
    target_center = (goal[0]+ell*math.cos(goal[3]),
                     goal[1]+ell*math.sin(goal[3]), goal[3])
    total = radius*sum(lengths)
    if total == 0. and start[2] != goal[2]:
        raise ValueError('A zero-length planar path cannot connect different endpoint heights')
    current, travelled = start_center, 0.
    center_rows = [[*start_center[:2], start[2], start_center[2]]]
    sample_s = [0.]
    segments = []
    base_length = 0.
    for kind, normalized in zip(family, lengths):
        length = radius*normalized
        curvature = 0. if kind == 'S' else (1. if kind == 'L' else -1.)/radius
        first_index = len(center_rows)-1
        # Equal subdivisions bound the arc length by step and retain every
        # exact primitive boundary. No duplicated transition sample is needed.
        count = max(1, int(math.ceil(length/step))) if length > 0. else 0
        for index in range(1, count+1):
            local_s = length*index/count
            x, y, psi = _advance(current, local_s, curvature)
            global_s = travelled+local_s
            fraction = global_s/total if total > 0. else 0.
            z = start[2]+fraction*(goal[2]-start[2])
            center_rows.append([x, y, z, psi])
            sample_s.append(global_s)
        end = _advance(current, length, curvature)
        segments.append({'kind': kind, 'length_m': length, 'curvature_per_m': curvature,
                         'turn_angle_rad': curvature*length,
                         'start_index': first_index, 'end_index': len(center_rows)-1,
                         'start_center_xy_yaw': list(current), 'end_center_xy_yaw': list(end),
                         'start_arclength_m': travelled, 'end_arclength_m': travelled+length})
        base_length += length*math.sqrt(1.+(ell*curvature)**2)
        current, travelled = end, travelled+length
    residual_xy = math.dist(current[:2], target_center[:2])
    residual_yaw = abs(wrap(current[2]-target_center[2]))
    scale = max(1., total, *(abs(v) for v in (*start[:2], *goal[:2])))
    tolerance = 1.e-10*scale
    if residual_xy > tolerance or residual_yaw > 1.e-10:
        raise ArithmeticError(f'{family} endpoint failed closed-form verification: '
                              f'{residual_xy:g} m / {residual_yaw:g} rad')
    # Set exact requested XYZ only after verifying the analytic endpoint.
    # Preserve the continuous yaw branch; endpoint orientation is identical.
    lifted_target_yaw = goal[3]+TAU*round((current[2]-goal[3])/TAU)
    center_rows[-1] = [*target_center[:2], goal[2], lifted_target_yaw]
    base_rows = [[x-ell*math.cos(psi), y-ell*math.sin(psi), z, psi]
                 for x, y, z, psi in center_rows]
    base_rows[0] = list(start)
    base_rows[-1] = [*goal[:3], lifted_target_yaw]
    max_curvature = max((abs(s['curvature_per_m']) for s in segments if s['length_m'] > 0.), default=0.)
    return {'family': family, 'status': 'constructed', 'center_length_m': total,
            'base_planar_length_m': base_length,
            'normalized_segment_lengths': list(lengths), 'segments': segments,
            'center_samples_xyzyaw': center_rows, 'base_samples_xyzyaw': base_rows,
            'center_sample_arclengths_m': sample_s,
            'sampling': {'maximum_requested_center_arclength_step_m': step,
                         'maximum_actual_center_arclength_step_m': max(
                             (b-a for a, b in zip(sample_s, sample_s[1:])), default=0.),
                         'all_primitive_boundaries_retained': True,
                         'sample_count': len(center_rows),
                         'z_policy': 'Linear placeholder between declared endpoint heights; native terrain projection required before validation/execution.'},
            'endpoint_verification': {'analytic_center_position_error_m': residual_xy,
                                      'analytic_orientation_error_rad': residual_yaw,
                                      'roundoff_tolerance_m': tolerance,
                                      'base_start_xyz_exact': base_rows[0][:3] == list(start[:3]),
                                      'base_goal_xyz_exact': base_rows[-1][:3] == list(goal[:3]),
                                      'start_yaw_error_rad': wrap(base_rows[0][3]-start[3]),
                                      'goal_yaw_error_rad': wrap(base_rows[-1][3]-goal[3]),
                                      'yaw_policy': 'Continuous unwrapped yaw; terminal orientation equals requested yaw modulo 2*pi.'},
            'maximum_center_curvature_per_m': max_curvature,
            'maximum_absolute_base_tangent_heading_difference_rad': math.atan(ell*max_curvature),
            'minimum_center_forward_speed_per_base_speed': 1./math.sqrt(1.+(ell*max_curvature)**2),
            'native_collision_and_terrain_validated': False}


def generate_candidates(start_base_xyzyaw, target_base_xyzyaw, *,
                        offset_m=DEFAULT_OFFSET_M, radius_m=DEFAULT_RADIUS_M,
                        sample_step_m=DEFAULT_SAMPLE_STEP_M):
    """Return all constructible fixed-radius families sorted by C path length.

    A constant C speed of 0.3 m/s and radius 0.6 m imply |omega|<=0.5 rad/s.
    The parameterized math is reusable, but a caller selecting another radius
    must declare its limits before evaluating candidates; no radius sweep is
    performed here. This function never reads a map or simulator truth.
    """
    start = _pose(start_base_xyzyaw, 'start_base_xyzyaw')
    goal = _pose(target_base_xyzyaw, 'target_base_xyzyaw')
    ell, radius, step = map(float, (offset_m, radius_m, sample_step_m))
    if not all(math.isfinite(v) and v > 0. for v in (ell, radius, step)):
        raise ValueError('offset, radius and sample step must be finite and positive')
    cx0, cy0 = start[0]+ell*math.cos(start[3]), start[1]+ell*math.sin(start[3])
    cx1, cy1 = goal[0]+ell*math.cos(goal[3]), goal[1]+ell*math.sin(goal[3])
    dx, dy = cx1-cx0, cy1-cy0
    direction = math.atan2(dy, dx)
    distance = math.hypot(dx, dy)/radius
    alpha, beta = mod2pi(start[3]-direction), mod2pi(goal[3]-direction)
    candidates, rejected = [], []
    for family in FAMILIES:
        lengths = _normalized_lengths(family, alpha, beta, distance)
        if lengths is None:
            rejected.append({'family': family, 'reason': 'No real tangent/three-circle solution at the declared radius'})
            continue
        candidates.append(_sample(family, lengths, start, goal, ell, radius, step))
    candidates.sort(key=lambda c: (c['center_length_m'], FAMILIES.index(c['family'])))
    return {'scope': 'Pure planar fixed-center Dubins candidate generation; no collision/terrain or physical-execution claim',
            'start_base_xyzyaw': list(start), 'target_base_xyzyaw': list(goal),
            'offset_m': ell, 'radius_m': radius, 'sample_step_m': step,
            'maximum_center_curvature_per_m': 1./radius,
            'yaw_rate_at_nominal_center_speed_0p3_mps': .3/radius,
            'coordinate_relation': 'C=B+ell*[cos(yaw),sin(yaw)]; B=C-ell*[cos(yaw),sin(yaw)]',
            'velocity_relation': 'v_B_body=(v_C, -ell*omega), omega=kappa_C*v_C; no reverse C motion',
            'endpoint_yaw_policy': 'Both endpoint orientations prescribed; target yaw may be chosen by the upstream planner, but is fixed within this connector call.',
            'all_six_families_considered': True, 'candidates': candidates,
            'rejected_families': rejected,
            'limitations': ['All candidates require original native B sweep and terrain validation.',
                            'Six fixed-radius Dubins families do not exhaust obstacle-avoiding paths.',
                            'Curvature is bounded but curvature rate/acceleration continuity is not guaranteed at primitive boundaries.',
                            'The connector intentionally proposes new geometric paths rather than preserving a previous planner polyline.'],
            'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start-base', nargs=4, type=float, required=True, metavar=('X', 'Y', 'Z', 'YAW'))
    parser.add_argument('--target-base', nargs=4, type=float, required=True, metavar=('X', 'Y', 'Z', 'YAW'))
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = generate_candidates(args.start_base, args.target_base)
    text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
    if args.output is None:
        print(text, end='')
    else:
        args.output.write_text(text, encoding='utf-8')
        print(json.dumps({'output': str(args.output),
                          'candidates': [{'family': c['family'], 'center_length_m': c['center_length_m']}
                                         for c in result['candidates']],
                          'rejected_families': result['rejected_families']}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
