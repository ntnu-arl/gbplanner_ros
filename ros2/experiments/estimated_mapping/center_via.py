"""Single-via fixed-radius C connectors through recorded planner B poses.

This is an independent experimental geometry generator. It does not replace
production planning or preserve the complete input polyline. Every possible
via is an intermediate pose (including its yaw) of the supplied real planner
path. No scene coordinates, seed selection or radius sweep are introduced.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import time

TAU = 2.0 * math.pi
ROOT = Path(__file__).resolve().parents[2]


def _wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def _pose(values):
    if len(values) != 4:
        raise ValueError('Expected B pose [x,y,z,yaw]')
    value = [float(v) for v in values]
    if not all(math.isfinite(v) for v in value):
        raise ValueError('Nonfinite B pose')
    return value


def _check_summary(check):
    return {k: v for k, v in check.items() if k != 'projected_base_path'}


def _compose(via_index, via, first, second, checks, start, target, offset):
    """Concatenate two native-valid legs without introducing a new segment."""
    a = copy.deepcopy(checks[first['id']]['projected_base_path'])
    b = copy.deepcopy(checks[second['id']]['projected_base_path'])
    xy_error = math.dist(a[-1][:2], b[0][:2])
    z_error = abs(a[-1][2]-b[0][2])
    yaw_error = abs(_wrap(a[-1][3]-b[0][3]))
    if xy_error > 1.e-10 or z_error > 1.e-10 or yaw_error > 1.e-10:
        raise ValueError('Native-valid legs disagree at their declared via pose')
    # A Dubins family can wind by 2*pi. Align the second leg's continuous
    # branch by a single integer shift, preserving every physical orientation.
    yaw_shift = TAU * round((a[-1][3]-b[0][3])/TAU)
    for row in b:
        row[3] += yaw_shift
    seam_index = len(a)-1
    base = a+b[1:]
    center = [[x+offset*math.cos(yaw), y+offset*math.sin(yaw), z, yaw]
              for x, y, z, yaw in base]
    segments = []
    length_offset = 0.
    for leg_index, leg in enumerate((first, second)):
        index_offset = 0 if leg_index == 0 else seam_index
        branch = 0. if leg_index == 0 else yaw_shift
        for primitive in leg['candidate']['segments']:
            segment = copy.deepcopy(primitive)
            segment.update(leg_index=leg_index, native_candidate_id=leg['id'])
            segment['start_index'] += index_offset
            segment['end_index'] += index_offset
            segment['start_arclength_m'] += length_offset
            segment['end_arclength_m'] += length_offset
            segment['start_center_xy_yaw'][2] += branch
            segment['end_center_xy_yaw'][2] += branch
            segments.append(segment)
        length_offset += leg['candidate']['center_length_m']
    if (math.dist(base[0][:2], start[:2]) > 1.e-10 or
            math.dist(base[-1][:2], target[:2]) > 1.e-10 or
            abs(_wrap(base[0][3]-start[3])) > 1.e-10 or
            abs(_wrap(base[-1][3]-target[3])) > 1.e-10):
        raise AssertionError('Combined connector changed a prescribed endpoint pose')
    return {
        'family': f'via_{via_index:03d}_{first["candidate"]["family"]}_{second["candidate"]["family"]}',
        'via_planner_index': via_index, 'via_base_xyzyaw': via,
        'leg_candidate_ids': [first['id'], second['id']],
        'base_reference_xyzyaw': base, 'center_reference_xyzyaw': center,
        'segments': segments, 'center_length_m': length_offset,
        'base_planar_length_m': sum(leg['candidate']['base_planar_length_m'] for leg in (first, second)),
        'maximum_center_curvature_per_m': max(
            leg['candidate']['maximum_center_curvature_per_m'] for leg in (first, second)),
        'radius_m': .6, 'offset_m': offset,
        'native_check': {'success': True, 'method': 'Both complete legs passed original native checks; '
                         'an identical via pose is shared, so concatenation introduces no new geometric segment.',
                         'legs': [_check_summary(checks[leg['id']]) for leg in (first, second)]},
        'seam': {'combined_sample_index': seam_index,
                 'removed_duplicate': {'leg_index': 1, 'sample_index': 0},
                 'first_leg_last_z_m': a[-1][2], 'second_leg_first_z_m': b[0][2],
                 'xyz_error_before_join_m': math.dist(a[-1][:3], b[0][:3]),
                 'yaw_orientation_error_before_join_rad': yaw_error,
                 'second_leg_constant_yaw_branch_shift_rad': yaw_shift,
                 'no_new_segment_created': True},
        'endpoint': {'start_xy_error_m': math.dist(base[0][:2], start[:2]),
                     'goal_xy_error_m': math.dist(base[-1][:2], target[:2]),
                     'start_yaw_error_rad': _wrap(base[0][3]-start[3]),
                     'goal_yaw_error_rad': _wrap(base[-1][3]-target[3]),
                     'start_z_projection_change_m': base[0][2]-start[2],
                     'goal_z_projection_change_m': base[-1][2]-target[2]},
        'sample_policy': 'All samples of both selected legs retained in order except the explicitly '
                         'indexed duplicate via. Native projection changes only Z; one whole-leg '
                         '2*pi yaw branch shift preserves orientations.',
        'terminal_yaw_preserved': True,
        'all_candidate_xyz_retained_in_order': True,
        'final_live_ros_validation_required': True,
    }


def construct_via(root, folder, start, target, planner_base_poses, offset_m,
                  follower, parameters, map_contract, spin_until=None):
    """Return sorted ``feasible`` paths and ``selected`` (or None).

    ``map_contract`` contains map/robot/ground/bounds from center_path's native
    request, with an existing packed observed map. ``planner_base_poses`` is
    the actual returned B path, not graph planning-height poses. Each interior
    sample is considered once as a fixed-position, fixed-yaw via. Only the
    shortest native-valid family of each leg is combined for that via.

    Existing search vertex/edge caps bound the count of via poses and native
    candidate legs. They do not claim computational equivalence to graph
    edges. There is one batch of native leg checks and no parameter sweep.
    """
    from center_dubins import generate_candidates
    from center_path import run_native, save
    root, folder = Path(root), Path(folder)
    folder.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    start, target = _pose(start), _pose(target)
    original = [_pose(row) for row in planner_base_poses]
    if len(original) < 3:
        raise ValueError('Single-via extension needs an actual planner path with an interior pose')
    radius = float(follower['speed'])/float(follower['yaw_rate'])
    step = min(.02, float(follower['speed'])*float(follower['sample_period']),
               radius*float(follower['yaw_rate'])*float(follower['sample_period']))
    if abs(radius-.6) > 1.e-12 or abs(step-.015) > 1.e-12:
        raise ValueError('This declared experiment fixes radius=.6 m and sampling step=.015 m')
    if not math.isfinite(offset_m) or offset_m <= 0:
        raise ValueError('Positive finite fixed-center offset required')
    vertex_budget = int(parameters['search.max_vertices'])
    edge_budget = int(parameters['search.max_edges'])
    if vertex_budget != 250 or edge_budget != 2000 or int(parameters['search.max_attempts']) != 6000:
        raise ValueError('Original 250/2000/6000 search limits changed')
    if len(original) > vertex_budget:
        raise ValueError('Recorded source/via/goal pose count exceeds existing vertex budget')
    if (math.dist(original[-1][:2], target[:2]) > 1.e-8 or
            abs(_wrap(original[-1][3]-target[3])) > 1.e-8):
        raise ValueError('Via source path ends at a different prescribed target pose')
    contract = {k: copy.deepcopy(v) for k, v in map_contract.items() if k != 'candidates'}
    map_path = Path(contract['map']['path'])
    if not map_path.is_absolute():
        raise ValueError('Pass an absolute observed map path to construct_via')
    observed_sha = hashlib.sha256(map_path.read_bytes()).hexdigest()
    if contract['map'].get('sha256', observed_sha) != observed_sha:
        raise ValueError('Observed packed map hash changed')
    contract['map']['sha256'] = observed_sha
    result = {
        'status': 'running', 'truth_used': False,
        'scope': 'Independent single-via C-space extension using real planner interior B poses; '
                 'production planner, entrypoints and defaults remain unchanged.',
        'start_base_xyzyaw': start, 'target_base_xyzyaw': target,
        'offset_m': offset_m, 'radius_m': radius, 'sample_step_m': step,
        'planner_input_base_path': original, 'via_selection': 'All actual interior planner poses, '
        'with their original XY and yaw; no synthetic detour points or yaw sweep.',
        'composition_policy': 'At most one combination per via: shortest native-valid first leg '
        'plus shortest native-valid second leg; no Cartesian-product search.',
        'budgets': {'max_vertices': vertex_budget, 'max_native_candidate_legs': edge_budget,
                    'original_max_attempts': int(parameters['search.max_attempts']),
                    'original_seed': parameters.get('search.sampling.seed', parameters.get('sampling.seed')),
                    'scope': 'Caps reused for candidate accounting, not a claim of equal graph-search compute.'},
        'map_record': contract['map'], 'vias': [], 'feasible': [], 'selected': None,
        'source_sha256': {name: hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()
                          for name in ('center_via.py','center_path.py','center_dubins.py')},
        'final_live_ros_validation_required': True,
        'limitations': ['A single planner-derived via is not a complete obstacle-aware search.',
                       'Curvature is bounded; curvature-rate and command acceleration are not guaranteed continuous.',
                       'Native checks admit the sampled B polyline, not every point of the underlying analytic curve.',
                       'Both-leg admission does not prove physical tracking or estimator accuracy.'],
    }
    legs = {}
    attempted_families = 0
    for index, via in enumerate(original[1:-1], start=1):
        entry = {'planner_index': index, 'base_xyzyaw': via, 'legs': {}, 'status': 'pending'}
        for leg_index, (leg_start, leg_end) in enumerate(((start, via), (via, target))):
            generated = generate_candidates(leg_start, leg_end, offset_m=offset_m,
                                            radius_m=radius, sample_step_m=step)
            attempted_families += 6
            leg_entry = {'mathematically_rejected': generated['rejected_families'], 'candidates': []}
            for candidate in generated['candidates']:
                identity = f'via_{index:03d}_leg_{leg_index}_{candidate["family"]}'
                legs[identity] = {'id': identity, 'via_index': index, 'leg_index': leg_index,
                                  'candidate': candidate}
                leg_entry['candidates'].append({'id': identity, 'family': candidate['family'],
                                               'center_length_m': candidate['center_length_m']})
            entry['legs'][str(leg_index)] = leg_entry
        result['vias'].append(entry)
    if len(legs) > edge_budget:
        raise ValueError('Constructed candidate legs exceed original edge budget; no silent truncation')
    result['search_counts'] = {'planner_poses': len(original), 'via_poses': len(result['vias']),
        'mathematical_family_attempts': attempted_families, 'native_candidate_legs': len(legs),
        'native_passed_legs': 0, 'evaluated_via_combinations': 0, 'feasible_via_combinations': 0}
    save(folder/'generated_legs.json', {'legs': list(legs.values()), 'search_counts': result['search_counts']})
    contract['candidates'] = [{'id': key, 'base_path': value['candidate']['base_samples_xyzyaw']}
                              for key, value in legs.items()]
    native_dir = folder/'native_legs'
    native_dir.mkdir()
    native = run_native(root, native_dir, contract, spin_until)
    native_checks = {check['id']: check for check in native['candidates']}
    if set(native_checks) != set(legs):
        raise AssertionError('Native batch omitted or added a candidate leg')
    for identity, check in native_checks.items():
        if not check['success']:
            continue
        original_leg = legs[identity]['candidate']['base_samples_xyzyaw']
        projected = check['projected_base_path']
        if len(original_leg) != len(projected) or any(
                [a[0],a[1],a[3]] != [b[0],b[1],b[3]] for a,b in zip(original_leg,projected)):
            raise AssertionError('Native projection changed candidate XY/yaw or omitted samples')
        result['search_counts']['native_passed_legs'] += 1
    for via in result['vias']:
        chosen = []
        for leg_index in (0,1):
            successful = []
            for candidate in via['legs'][str(leg_index)]['candidates']:
                check = native_checks[candidate['id']]
                candidate['native'] = _check_summary(check)
                if check['success']:
                    successful.append(legs[candidate['id']])
            successful.sort(key=lambda leg: (leg['candidate']['center_length_m'], leg['id']))
            via['legs'][str(leg_index)]['native_passed_families'] = len(successful)
            chosen.append(successful[0] if successful else None)
        if any(leg is None for leg in chosen):
            via['status'] = 'rejected'
            via['reason'] = 'no_native_valid_leg_'+'_'.join(str(i) for i,leg in enumerate(chosen) if leg is None)
            continue
        result['search_counts']['evaluated_via_combinations'] += 1
        try:
            combined = _compose(via['planner_index'], via['base_xyzyaw'], *chosen,
                                native_checks, start, target, offset_m)
        except ValueError as error:
            via.update(status='rejected', reason=str(error))
            continue
        via.update(status='passed', reason='shortest_native_valid_legs_joined',
                   selected_leg_ids=[leg['id'] for leg in chosen],
                   center_length_m=combined['center_length_m'], seam=combined['seam'])
        result['feasible'].append(combined)
    result['feasible'].sort(key=lambda candidate: (candidate['center_length_m'], candidate['via_planner_index']))
    result['search_counts']['feasible_via_combinations'] = len(result['feasible'])
    result['selected'] = result['feasible'][0] if result['feasible'] else None
    result['status'] = 'completed'
    result['outcome'] = 'native_valid_single_via_found' if result['selected'] else 'all_single_via_candidates_rejected'
    result['native'] = {'runtime': native['runtime'], 'map': native['map'],
                        'request': str(native_dir/'terrain_request.json'),
                        'response': str(native_dir/'terrain_response.json')}
    result['elapsed_wall_s'] = time.monotonic()-started
    save(folder/'construction.json', result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--connector-dir', type=Path, default=ROOT/
        'results/estimated_mapping/center_dubins_replay/refined_target/center_connector')
    parser.add_argument('--planner-probe', type=Path, default=ROOT/
        'results/estimated_mapping/run9_multiround_refined/round2/planned_return_search/planner_probe.json')
    parser.add_argument('--output', type=Path, default=ROOT/'results/estimated_mapping/center_via_design')
    args = parser.parse_args()
    import yaml
    from planned_execution import _state
    original = json.loads((args.connector_dir/'construction.json').read_text())
    contract = json.loads((args.connector_dir/'terrain_request.json').read_text())
    probe = json.loads(args.planner_probe.read_text())
    parameters = yaml.safe_load(args.planner_probe.with_suffix('.yaml').read_text())['gbplanner_local']['ros__parameters']
    follower = yaml.safe_load((ROOT/'src/gbplanner_tracking/config/smb.yaml').read_text())[
        'gbplanner_path_follower']['ros__parameters']
    result = construct_via(ROOT, args.output, original['start_base_xyzyaw'],
                           original['target_base_xyzyaw'],
                           [_state(row) for row in probe['search_response']['path']],
                           original['offset_m'], follower, parameters, contract)
    print(json.dumps({'status': result['status'], 'outcome': result['outcome'],
                      'counts': result['search_counts'], 'output': str(args.output),
                      'selected': None if not result['selected'] else {
                          k: result['selected'][k] for k in ('family','via_planner_index','center_length_m','endpoint','seam')}}))
    return 0


if __name__ == '__main__':
    sys.dont_write_bytecode = True
    raise SystemExit(main())
