#!/usr/bin/env python3
"""Inspect recorded Pittsburgh support, without ROS, simulation, or map edits.

The simplex query and vertical trace follow the current C++ sources. This is
an independent offline translation, not a call to the running ValidatePath
service. Acquisition-stamp association does not prove immutable-map identity.
"""
import argparse
from collections import Counter
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from inspect_smb_tsdf_projection import RecordedMap


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def simplex(mapping, point):
    """Use observed vertices only; even tiny positive coefficients need data."""
    grid = [v / mapping.resolution - .5 for v in point]
    lower = [math.floor(v) for v in grid]
    fraction = [v - i for v, i in zip(grid, lower)]
    axes = sorted(range(3), key=lambda a: -fraction[a])  # stable x,y,z ties
    weights = [1. - fraction[axes[0]], fraction[axes[0]] - fraction[axes[1]],
               fraction[axes[1]] - fraction[axes[2]], fraction[axes[2]]]
    vertex, neighbors, missing = [0, 0, 0], [], []
    distance, weight = 0., 0.
    for i, coefficient in enumerate(weights):
        if i:
            vertex[axes[i - 1]] = 1
        index = tuple(a + b for a, b in zip(lower, vertex))
        cell = mapping.cells.get(index)
        observed = cell is not None and cell['weight'] >= 1e-6
        entry = {'index': list(index), 'coefficient': coefficient,
                 'required': coefficient != 0., 'observed': observed, 'cell': cell}
        neighbors.append(entry)
        if coefficient == 0.:
            continue
        if not observed:
            missing.append(list(index))
        else:
            distance += coefficient * cell['distance']
            weight += coefficient * cell['weight']
    return {'point': list(point), 'lower_index': lower, 'fraction': fraction,
            'axes': axes, 'neighbors': neighbors, 'missing_required': missing,
            'distance': None if missing else distance,
            'weight': None if missing else weight}


def cast_down(mapping, start, max_length, detailed=False):
    """First observed sign bracket, including every vertical simplex knot."""
    resolution = mapping.resolution
    end_z, step, z_tolerance = start[2] - max_length, .5 * resolution, .001 * resolution
    trace = []

    def query(z):
        result = simplex(mapping, [start[0], start[1], z])
        trace.append(result if detailed else {'z': z, 'distance': result['distance'],
                                             'missing_required': result['missing_required']})
        return result

    def finish(status, terminal, **values):
        return {'status': status, 'start': list(start), 'terminal_query': terminal,
                'queries': trace, **values}

    def next_lower_plane(z, spacing, offset):
        plane = (math.floor(z / spacing - offset) + offset) * spacing
        if plane >= z:
            plane -= spacing
        return plane

    xy_grid = [v / resolution - .5 for v in start[:2]]
    fraction = [v - math.floor(v) for v in xy_grid]
    upper_z = start[2]
    upper = query(upper_z)
    if upper['distance'] is None:
        return finish('unknown', upper)
    if upper['distance'] <= 0.:
        return finish('rejected_nonpositive_start', upper)
    while upper_z > end_z:
        lower_z = math.floor(upper_z / step) * step
        if lower_z >= upper_z:
            lower_z -= step
        for value in fraction:
            lower_z = max(lower_z, next_lower_plane(upper_z, resolution, .5 + value))
        lower_z = max(lower_z, end_z)
        if lower_z >= upper_z:
            return finish('no_progress', upper)
        lower = query(lower_z)
        if lower['distance'] is None:
            return finish('unknown', lower)
        if lower['distance'] <= 0.:
            for _ in range(64):
                if upper_z - lower_z <= z_tolerance:
                    break
                middle_z = lower_z + .5 * (upper_z - lower_z)
                middle = query(middle_z)
                if middle['distance'] is None:
                    return finish('unknown', middle)
                if middle['distance'] > 0.:
                    upper_z, upper = middle_z, middle
                else:
                    lower_z, lower = middle_z, middle
            if upper_z - lower_z > z_tolerance:
                return finish('rejected_bracket_width', lower)
            denominator = upper['distance'] - lower['distance']
            if not math.isfinite(denominator) or denominator <= 0.:
                return finish('rejected_denominator', lower)
            ratio = -lower['distance'] / denominator
            estimate = lower_z + ratio * (upper_z - lower_z)
            check = query(estimate)
            if check['distance'] is None:
                return finish('unknown', check)
            tolerance = resolution * 1e-6 + 32. * sys.float_info.epsilon * max(
                abs(lower['distance']), abs(upper['distance']))
            if abs(check['distance']) > tolerance:
                return finish('rejected_residual', check)
            return finish('supported', check, surface_z=estimate,
                          lower_z=lower_z, upper_z=upper_z,
                          residual=check['distance'], residual_tolerance=tolerance)
        upper_z, upper = lower_z, lower
    return finish('no_crossing_in_range', upper)


def body_box(mapping, center, size):
    """Mirror SparseVoxelMap::box's inclusive expanded-AABB arithmetic."""
    resolution = mapping.resolution
    half = [v * .5 for v in size]
    radius = [math.floor(v / resolution) + 1 for v in half]
    origin = [math.floor(v / resolution) for v in center]
    # Preserve its center/index round trip at decimal voxel resolutions.
    origin = [math.floor(((i + .5) * resolution) / resolution) for i in origin]
    intervals = []
    for axis in range(3):
        indices = []
        for i in range(origin[axis] - radius[axis], origin[axis] + radius[axis] + 1):
            low = float(i) * resolution - half[axis]
            high = low + resolution + size[axis]
            if not (center[axis] < low or center[axis] > high):
                indices.append(i)
        intervals.append(indices)
    counts, first_nonfree = Counter(), []
    for z, y, x in itertools.product(intervals[2], intervals[1], intervals[0]):
        cell = mapping.cells.get((x, y, z))
        status = 'unknown' if cell is None or cell['occupancy'] == -1 else (
            'occupied' if cell['occupancy'] == 100 else 'free')
        counts[status] += 1
        if status != 'free' and len(first_nonfree) < 8:
            first_nonfree.append({'index': [x, y, z], 'status': status, 'cell': cell})
    status = 'occupied' if counts['occupied'] else 'unknown' if counts['unknown'] else 'free'
    return {'status': status, 'center': list(center), 'size': list(size),
            'index_intervals': [[v[0], v[-1]] if v else [] for v in intervals],
            'counts': dict(counts), 'first_nonfree': first_nonfree}


def inspect_pose(mapping, pose, parameters, detailed=False):
    """Evaluate geometry of the saved root request (single point, Exact size)."""
    planning = list(pose[:3])
    planning[2] += parameters['ground.max_ground_height'] - parameters['ground.robot_height']
    offset, size = parameters['robot.center_offset'], parameters['robot.size']
    # Use the baseState round trip used by the validator, not an alternate Z.
    base = list(planning)
    base[2] += parameters['ground.robot_height'] - parameters['ground.max_ground_height']
    body = body_box(mapping, [v + o for v, o in zip(base, offset)], size)
    support = cast_down(mapping, planning, parameters['ground.projection_max_length'], detailed)
    result = {'base_pose': list(pose), 'planning_position': planning,
              'body': body, 'central_support': support}
    reason = 'ok'
    if body['status'] != 'free':
        reason = 'current_pose_not_free'
    elif support['status'] != 'supported':
        reason = 'terrain_support_missing'
    if support['status'] == 'supported':
        projected_base = [base[0], base[1], support['surface_z'] + parameters['ground.robot_height']]
        result['projected_body'] = body_box(mapping, [v + o for v, o in zip(projected_base, offset)], size)
        result['height_difference_m'] = support['surface_z'] + parameters['ground.max_ground_height'] - planning[2]
        if reason == 'ok' and abs(result['height_difference_m']) > mapping.resolution + 1e-8:
            reason = 'path_off_terrain'
    # Projected body is useful for planning but the fixed-path validator checks
    # the original body and support. Do not silently swap in a projected pose.
    result['single_point_geometry_reason'] = reason
    return result


def consecutive_groups(rows):
    groups = []
    for row in rows:
        reason = row['single_point_geometry_reason']
        if not groups or groups[-1]['reason'] != reason:
            groups.append({'first_sample_x': row['base_pose'][0], 'last_sample_x': row['base_pose'][0],
                           'reason': reason, 'samples': 1})
        else:
            groups[-1]['last_sample_x'] = row['base_pose'][0]
            groups[-1]['samples'] += 1
    return groups


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=root / 'results/pittsburgh_port/autonomy1.json')
    parser.add_argument('--output', type=Path, default=root / 'results/pittsburgh_port/autonomy1_support.json')
    args = parser.parse_args()
    run_path = args.run.resolve()
    run = json.loads(run_path.read_text())
    parameters = run['configuration']['effective_planner_parameters']
    if parameters['ground.tsdf_projection_mode'] != 'zero_crossing_simplex':
        raise ValueError('This tool only diagnoses the continuous simplex projection')
    pose = run['observation_captures']['final']['odometry']['position_and_yaw']
    stem = run_path.with_suffix('')
    failure = stem.with_name(stem.name + '_failure_map.bin')
    background = stem.with_name(stem.name + '_background')
    before = background / 'before_home_map.bin'
    final = background / 'final_map.bin'
    metadata_before = run['observation_captures']['before_home']['accepted_map']
    sources = {
        'simplex': root / 'src/gbplanner_local/include/gbplanner_local/observed_tsdf_simplex.hpp',
        'sparse_map': root / 'src/gbplanner_local/src/sparse_voxel_map.cpp',
        'ground': root / 'src/gbplanner_local/src/ground_terrain.cpp',
        'validation': root / 'src/gbplanner_local/include/gbplanner_local/path_validation.hpp',
        'inspector': Path(__file__).resolve(),
    }
    maps = {'before_home': RecordedMap(metadata_before, before.read_bytes(), parameters['map.resolution']),
            'failure': RecordedMap(run['failure_map'], failure.read_bytes(), parameters['map.resolution'])}
    report = {
        'scope': 'Offline recorded-map data and independent Python translation of current C++ geometry',
        'limitations': [
            'failure_map is acquisition-stamp-associated, not proven to be the exact immutable ValidatePath input.',
            'The saved odometry and map were independently received, not captured atomically.',
            'This diagnostic does not execute native simulation, ROS services, or C++ libraries.',
            'Consecutive grid samples are sampled bands, not proof of every point or a traversable path between them.',
            'A map-level support result does not establish visibility, true surface error, or dynamic safety.',
        ],
        'inputs': {str(p.relative_to(root)): sha256(p) for p in [run_path, failure, before, final]},
        'source_sha256': {name: {'path': str(path.relative_to(root)), 'sha256': sha256(path)}
                          for name, path in sources.items()},
        'failure_equals_final_bytes': failure.read_bytes() == final.read_bytes(),
        'failure_map_association': run['failure_map'],
        'saved_service_response': run['checks']['root_before_autonomy'],
        'parameters': {name: value for name, value in parameters.items()
                       if name.startswith(('map.', 'ground.', 'robot.'))},
        'request_geometry': {'bound_mode': 3, 'bound_name': 'Exact', 'clearance_margin': 0.,
                             'single_point': True, 'first_pose_is_observation': True,
                             'support_ray': 'central only via validateFixedEdge'},
        'actual_saved_pose': pose, 'maps': {},
    }
    for label, mapping in maps.items():
        actual = inspect_pose(mapping, pose, parameters, True)
        grid = [inspect_pose(mapping, [i * .05, 0., pose[2], 0.], parameters) for i in range(65)]
        columns = []
        lower = [math.floor(v / mapping.resolution - .5) for v in pose[:2]]
        for x, y in itertools.product(range(lower[0], lower[0] + 2), range(lower[1], lower[1] + 2)):
            columns.append({'xy_index': [x, y], 'cells': [
                {'index': [x, y, z], 'cell': mapping.cells.get((x, y, z))} for z in range(-3, 13)]})
        negative = sorted((math.hypot(c['center'][0] - pose[0], c['center'][1] - pose[1]), key, c)
                          for key, c in mapping.cells.items()
                          if -.5 <= c['center'][2] <= .3 and c['distance'] <= 0. and c['weight'] >= 1e-6)
        side_rays = []
        for offset in [(0., 0., 0.), (.5, 0., .5), (-.5, 0., .5), (0., .5, .5), (0., -.5, .5)]:
            start = [a + b for a, b in zip(actual['planning_position'], offset)]
            side_rays.append({'offset': offset, 'ray': cast_down(mapping, start, parameters['ground.projection_max_length'])})
        report['maps'][label] = {'cell_count': len(mapping.cells), 'actual': actual,
            'actual_columns': columns, 'five_candidate_rays_not_used_by_fixed_path_validation': side_rays,
            'nearest_low_nonpositive_cells': [{'xy_distance': d, 'index': key, 'cell': c} for d, key, c in negative[:12]],
            'scan': {'step_m': .05, 'y_m': 0., 'base_z_m': pose[2],
                     'consecutive_sample_groups': consecutive_groups(grid), 'samples': grid}}
    old, new = maps['before_home'].cells, maps['failure'].cells
    keys = set(old) | set(new)
    changed = [key for key in keys if old.get(key) != new.get(key)]
    lower = [math.floor(v / parameters['map.resolution'] - .5) for v in pose[:2]]
    local = [key for key in changed if lower[0] <= key[0] <= lower[0] + 1
             and lower[1] <= key[1] <= lower[1] + 1 and -3 <= key[2] <= 12]
    report['before_to_failure'] = {'added_cells': len(set(new) - set(old)),
        'removed_cells': len(set(old) - set(new)), 'changed_cells_including_additions': len(changed),
        'changed_actual_four_columns': [{'index': key, 'before': old.get(key), 'after': new.get(key)} for key in sorted(local)]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    print(json.dumps({'output': str(args.output), 'failure_equals_final_bytes': report['failure_equals_final_bytes'],
        'maps': {name: {'actual_reason': data['actual']['single_point_geometry_reason'],
                       'terminal_z': data['actual']['central_support']['terminal_query']['point'][2],
                       'missing_required': data['actual']['central_support']['terminal_query']['missing_required'],
                       'sample_groups': data['scan']['consecutive_sample_groups']}
                 for name, data in report['maps'].items()},
        'before_to_failure': report['before_to_failure']}, indent=2))


if __name__ == '__main__':
    main()
