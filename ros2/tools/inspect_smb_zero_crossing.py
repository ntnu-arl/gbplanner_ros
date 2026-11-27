#!/usr/bin/env python3
"""Independent signed-TSDF bracket references on the saved map91; no ROS nodes.

Only complete trilinear sign brackets are accepted. Containing-voxel fallback
can establish known free samples before a crossing, but a fallback sign change
does not prove existence of a continuous zero. The reported bracket bounds the
zero of this saved interpolated field, not the physical ground error.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

from inspect_smb_tsdf_projection import RecordedMap


def compact_query(mapping, x, y, z):
    value = mapping.query([x, y, z])
    return {'z': z, 'distance': value['distance'], 'mode': value['mode']}


def continuous_root(mapping, x, y, lo, hi):
    """Solve the piecewise-linear vertical field independently of bisection."""
    r = mapping.resolution
    cuts = [lo, hi]
    for k in range(math.floor(lo/r-.5), math.ceil(hi/r-.5)+1):
        z = (k+.5)*r
        if lo < z < hi:
            cuts.append(z)
    cuts.sort()
    roots = []
    for low_z, high_z in zip(cuts, cuts[1:]):
        low, high = mapping.query([x, y, low_z]), mapping.query([x, y, high_z])
        if low['mode'] != 'trilinear' or high['mode'] != 'trilinear':
            continue
        ld, hd = low['distance'], high['distance']
        if hd > 0. and ld <= 0.:
            roots.append(low_z + (-ld)*(high_z-low_z)/(hd-ld))
    return max(roots) if roots else None


def inspect(mapping, point, length=1.5, tolerance=1e-4):
    x, y, start = point
    first = compact_query(mapping, x, y, start)
    samples = [first]
    result = {'start': point, 'step_m': mapping.resolution*.5, 'z_tolerance_m': tolerance,
              'samples': samples}
    if first['distance'] is None:
        return {**result, 'status': 'unknown_at_start'}
    if first['distance'] <= 0.:
        return {**result, 'status': 'no_positive_start'}
    previous = first
    # Include both cell faces and interpolation-stencil center planes. A
    # start-relative half-voxel grid can straddle a stencil extremum and miss
    # a short sign pocket, or reach a fallback sample below an actual root.
    half = mapping.resolution*.5
    end = start-length
    index = math.ceil(start/half)-1
    heights = []
    while index*half > end:
        heights.append(index*half)
        index -= 1
    heights.append(end)
    for z in heights:
        current = compact_query(mapping, x, y, z)
        samples.append(current)
        if current['distance'] is None:
            return {**result, 'status': 'unknown_before_crossing'}
        if current['distance'] <= 0.:
            result['initial_bracket'] = {'negative': current, 'positive': previous}
            if current['mode'] != 'trilinear' or previous['mode'] != 'trilinear':
                return {**result, 'status': 'unsupported_discontinuous_bracket'}
            lo, hi = current, previous
            exact = continuous_root(mapping, x, y, lo['z'], hi['z'])
            result['independent_piecewise_linear_root_z'] = exact
            iterations = 0
            while hi['z']-lo['z'] > tolerance:
                mid = compact_query(mapping, x, y, (lo['z']+hi['z'])/2.)
                iterations += 1
                if mid['distance'] is None:
                    return {**result, 'status': 'unknown_in_refinement'}
                if mid['mode'] != 'trilinear':
                    return {**result, 'status': 'unsupported_discontinuous_refinement'}
                if mid['distance'] > 0.:
                    hi = mid
                else:
                    lo = mid
            if exact is None or not lo['z']-1e-12 <= exact <= hi['z']+1e-12:
                raise AssertionError('Bisection bracket does not contain independent linear root')
            estimate = (lo['z']+hi['z'])/2.
            return {**result, 'status': 'continuous_crossing', 'refinement_iterations': iterations,
                    'final_bracket': {'negative': lo, 'positive': hi},
                    'bracket_width_m': hi['z']-lo['z'], 'estimated_ground_z': estimate,
                    'estimated_base_z': estimate+.24,
                    'root_location_error_bound_m': (hi['z']-lo['z'])/2.,
                    'physical_ground_error_bound_m': None}
        previous = current
    return {**result, 'status': 'no_crossing_within_projection_length'}


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=workspace/'results/smb_continuous/run5')
    parser.add_argument('--output', type=Path, default=workspace/'results/smb_continuous/run5_zero_crossing_reference.json')
    args = parser.parse_args()
    source = args.source.resolve()
    metadata = json.loads(source.with_name(source.name+'_map.json').read_text())
    raw = source.with_name(source.name+'_map.bin').read_bytes()
    mapping = RecordedMap(metadata, raw, .1)
    run = json.loads(source.with_suffix('.json').read_text())
    states = run['checks']['automatic_policy']['actions'][0]['commanded_path']['path']
    summary = {'scope': 'Independent read-only Python signed-bracket/analytic-linear references; no core or ROS execution',
               'map_sha256': hashlib.sha256(raw).hexdigest(), 'map_stamp': metadata['stamp'],
               'map_revision': 91, 'waypoints': [],
               'contract': 'Half-voxel vertical samples; stop at first unknown or unsupported sign change; '
                           'refine only complete trilinear brackets; error bound applies to this field root, not ground truth.',
               'sampling_grid': 'World-zero half-voxel planes, including voxel faces and trilinear stencil centers; '
                                'the first and last partial intervals are also no longer than half a voxel.',
               'sampling_limit': 'Complete trilinear vertical intervals are piecewise linear. Fallback intervals '
                                 'can be discontinuous; a detected sign change involving fallback is unsupported.'}
    for index, state in enumerate(states):
        traces = {name: inspect(mapping, [state[0], state[1], z]) for name, z in [
            ('commanded_planning_height', state[2]+.3), ('physical_flat_ground_planning_height', .54)]}
        summary['waypoints'].append({'index': index, 'original_commanded_pose': state, 'traces': traces})
        print(index, {name: (trace['status'], trace.get('independent_piecewise_linear_root_z'),
                            trace.get('estimated_base_z')) for name, trace in traces.items()})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')


if __name__ == '__main__':
    main()
