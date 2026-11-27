#!/usr/bin/env python3
"""Check new, pre-departure local-endpoint routes on an earlier recorded map.

This proposes a new command before dispatch. It never edits the old Action,
resets its progress, publishes motion, or claims an online historical replay.
Ordinary local routing knots are retained as provenance, not silently treated
as user waypoints. Actual integration must admit this route kind explicitly.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'src/gbplanner_execution/scripts'),
               str(ROOT/'src/gbplanner_local/scripts')]
from ground_maneuver import Limits, construct_maneuvers
from inspect_smb_tsdf_projection import RecordedMap
from pittsburgh_mesh.check_pose_maneuver import SavedGeometry
from pittsburgh_mesh.inspect_support import cast_down
from path_progress import PathProgress


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a new output; old results are retained')
    base = ROOT/'results/pittsburgh_port'
    paths = {name: base/name for name in ('autonomy3.json', 'autonomy3_history.json',
        'autonomy3_motion_analysis.json', 'autonomy3_background/before_home_snapshot.json',
        'autonomy3_background/before_home_map.bin')}
    run = json.loads(paths['autonomy3.json'].read_text())
    motion = json.loads(paths['autonomy3_motion_analysis.json'].read_text())
    history = json.loads(paths['autonomy3_history.json'].read_text())
    snapshot = json.loads(paths['autonomy3_background/before_home_snapshot.json'].read_text())
    command = motion['local']['commanded_path']
    old_route = command['path']
    earlier = [row for row in history['odometry'] if row['sim_s'] <= command['sim_s']]
    entry = earlier[-1]
    stationary_window = [row for row in earlier
                         if row['stamp_ns'] >= entry['stamp_ns']-200_000_000]
    if len(stationary_window) < 3 or any(row['speed_mps'] > 1e-7
            or abs(row['yaw_rate']) > 1e-7 for row in stationary_window):
        raise ValueError('This stop-to-stop construction requires an observed stationary start')
    parameters = run['configuration']['effective_planner_parameters']
    mapping = RecordedMap(snapshot['accepted_map'],
        paths['autonomy3_background/before_home_map.bin'].read_bytes(), parameters['map.resolution'])
    z = entry['pose'][2]
    geometry = SavedGeometry(mapping, parameters, old_route, z)
    counts = Counter()

    def valid_pose(pose):
        counts['queries'] += 1
        # A newly proposed route is checked against the observed world. The
        # old command's tube is not a constraint on a different, unissued route.
        if not geometry.body_free(pose[0], pose[1]):
            counts['body_not_free'] += 1
            return False
        if not geometry.support_free(pose[0], pose[1]):
            counts['support_missing'] += 1
            return False
        counts['accepted'] += 1
        return True

    limits = Limits(max_speed=.3, max_yaw_rate=.4, max_acceleration=.3,
        max_yaw_acceleration=.6, integration_dt=.05, max_duration=30.,
        max_path_length=5., max_points=20000, max_wall_time=5., max_candidates=2)
    start = tuple(entry['pose'][i] for i in (0, 1, 3))
    goal = tuple(old_route[-1][i] for i in (0, 1, 3))
    result = construct_maneuvers(start, [goal], offset=.2159, limits=limits, valid_pose=valid_pose)
    candidate_checks, fixture_routes = [], []
    for candidate_index, candidate in enumerate(result.candidates):
        route = [(s.x, s.y, z, s.yaw) for s in candidate.samples]
        progress = PathProgress([point[:3] for point in route])
        failures, maximum_height_error = [], 0.
        for i, point in enumerate(route):
            try:
                progress.update(point[:3])
            except ValueError as error:
                failures.append({'sample': i, 'reason': str(error)})
                break
            ray = cast_down(mapping, [point[0], point[1],
                z+parameters['ground.max_ground_height']-parameters['ground.robot_height']],
                parameters['ground.projection_max_length'])
            if ray['status'] != 'supported':
                failures.append({'sample': i, 'reason': 'exact_ground_support'})
                break
            error = abs(ray['surface_z']+parameters['ground.robot_height']-z)
            maximum_height_error = max(maximum_height_error, error)
            if error > mapping.resolution+1e-8:
                failures.append({'sample': i, 'reason': 'exact_ground_height'})
                break
        last = route[-1]
        candidate_checks.append({'candidate': candidate_index,
            'status': 'passed' if not failures else 'failed', 'failures': failures,
            'reference_samples': len(route), 'duration_s': candidate.duration,
            'base_arc_length_m': candidate.path_length,
            'endpoint_xy_error_m': math.dist(last[:2], old_route[-1][:2]),
            'endpoint_xyz_error_m': math.dist(last[:3], old_route[-1][:3]),
            'endpoint_yaw_error_rad': abs(math.remainder(last[3]-old_route[-1][3], 2*math.pi)),
            'maximum_support_height_error_m': maximum_height_error,
            'reference_progress_m': progress.progress, 'reference_polyline_length_m': progress.arc[-1],
            'progress_scope': 'Proposed unissued route only; the executed Action cursor is untouched',
            'production_cpp_segments': 'pending' if not failures else 'not_run'})
        if not failures:
            fixture_routes.append((f'candidate_{candidate_index}', route))
    fixture_path = args.output.with_suffix('.fixture')
    if fixture_routes:
        if fixture_path.exists():
            parser.error('Fixture already exists')
        # Same policy and scene bounds as the existing production C++ helper;
        # replace only the source-map row count and explicitly proposed paths.
        header = (base/'autonomy3_diagnosis/failure.fixture').read_text().splitlines()[:7]
        header[1] = str(snapshot['accepted_map']['width'])
        lines = header+[str(len(fixture_routes))]
        for name, route in fixture_routes:
            lines.append(f'{name} {len(route)} 1')
            lines.extend(' '.join(format(v, '.17g') for v in point) for point in route)
        fixture_path.write_text('\n'.join(lines)+'\n')
    sources = list(paths.values())+[Path(__file__), ROOT/'src/gbplanner_execution/scripts/ground_maneuver.py',
        ROOT/'src/gbplanner_local/scripts/path_progress.py',
        ROOT/'tools/pittsburgh_mesh/check_pose_maneuver.py']
    report = {'scope': 'New pre-dispatch local-endpoint route construction; not historical online replay',
        'native_execution': False, 'production_defaults_changed': False, 'active_goal_rewritten': False,
        'task_kind': 'ordinary_local_endpoint', 'source_route': old_route,
        'task_obligation': 'Retain original local endpoint XYZ/yaw; no explicit waypoint or gate task is claimed',
        'start_odometry': entry, 'stationary_observations': len(stationary_window),
        'start_nominal_velocity': [0., 0.], 'candidate_base_z': z,
        'map': snapshot['accepted_map'],
        'map_scope': 'Recorded27.6s map predates the36.26s stopped start; it is not the exact live dispatch snapshot',
        'limits': asdict(limits), 'geometry': dict(counts), 'construction': asdict(result),
        'candidate_checks': candidate_checks,
        'cpp_fixture': str(fixture_path) if fixture_routes else None,
        'source_sha256': {str(path.relative_to(ROOT)): sha(path) for path in sources},
        'remaining_obligations': ['Current-map C++ complete segment validation',
            'Task admission, mission budget and live initial-state verification',
            'Explicit primitive reference transport and feedback execution',
            'Actual odometry, stopped boundary, continuous motion and retained-route validation']}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'output': str(args.output), 'status': result.status,
        'candidates': len(result.candidates), 'geometry': dict(counts), 'checks': candidate_checks},
        ensure_ascii=False))


if __name__ == '__main__':
    main()
