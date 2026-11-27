#!/usr/bin/env python3
"""One old/hold cold-start pairing of motion5's real first automatic local route.

Run in the container after building smb_rotation_hold_replay. Each case uses
the original mesh, its own fresh vehicle, one second settling and unchanged
physical/controller settings. PathProgress is replayed offline without changing
the route tube or cancelling the core before its terminal result is observed.
"""
import argparse
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import subprocess


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def distance(point, path, dimensions):
    best = math.inf
    for a, b in zip(path, path[1:]):
        delta = [b[k]-a[k] for k in range(dimensions)]
        norm = sum(v*v for v in delta)
        fraction = max(0., min(1., sum((point[k]-a[k])*delta[k]
                                     for k in range(dimensions))/norm)) if norm else 0.
        best = min(best, math.sqrt(sum((point[k]-a[k]-fraction*delta[k])**2
                                      for k in range(dimensions))))
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    args.output.mkdir(parents=True, exist_ok=True)
    source = root/'results/edgar_mine/motion5_history.json'
    history = json.loads(source.read_text())
    record, = [r for r in history['paths'] if r['phase'] == 'automatic_exploration' and r['path']]
    path = record['path']
    if len(path) != 7 or abs(path[-1][0]-1.1303776349174686) > 1e-12:
        raise RuntimeError('expected exact motion5 first local route')
    # command_path is published after prepare(): it already contains the
    # measured-to-planner prefix. Interpolate it directly, without reconnecting
    # a second measured prefix that was never sent to the real tracker.
    accepted = next(r for r in history['events'] if r['layer'] == 'execution'
                    and r['phase'] == 'automatic_exploration' and r['state'] == 'VALIDATING')
    current = path[0]
    trajectory = module('rotation_trajectory', root/'src/gbplanner_execution/scripts/trajectory_tools.py')
    progress_module = module('rotation_progress', root/'src/gbplanner_local/scripts/path_progress.py')
    speed = .20000000298023224
    connected = path
    samples = trajectory.interpolate(connected, speed, .5, .05)
    if len(samples) != 192:
        raise RuntimeError(f'expected recorded 192 tracking points, got {len(samples)}')
    route_file = args.output/'motion5_tracking_route.csv'
    with route_file.open('w') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(['x', 'y', 'z', 'yaw'])
        writer.writerows(samples)
    (args.output/'route.json').write_text(json.dumps({
        'source_time_s': record['time_s'], 'execution_validation_time_s': accepted['time_s'],
        'measured_start': current, 'commanded_path': path, 'connected_path': connected,
        'tracking_points': len(samples), 'speed_mps': speed,
    }, indent=2)+'\n')
    mesh = root/'src/planner_gazebo_sim/collision/edgar_mine.triangles'
    cases = {}
    for treatment in ('old', 'hold'):
        out = args.output/treatment
        command = [str(args.executable.resolve()), str(mesh), str(route_file.resolve()),
                   treatment, str(out.resolve())]
        result = subprocess.run(command, capture_output=True, text=True, timeout=60.)
        out.mkdir(exist_ok=True)
        (out/'stdout.log').write_text(result.stdout)
        (out/'stderr.log').write_text(result.stderr)
        if result.returncode:
            raise RuntimeError(f'{treatment} replay failed: {result.stderr}')
        summary = json.loads((out/'summary.json').read_text())
        with (out/'trajectory.csv').open() as f:
            rows = list(csv.DictReader(f))
        progress = progress_module.PathProgress([p[:3] for p in connected])
        first_violation = None
        first_rotation = []
        initial_rotation_done = False
        maximum_xy = maximum_xyz = 0.
        # Match the 50 Hz odometry sampling independently of the 20 Hz control.
        for index, row in enumerate(rows):
            point = [float(row[k]) for k in ('x', 'y', 'z')]
            xy, xyz = distance(point, connected, 2), distance(point, connected, 3)
            maximum_xy, maximum_xyz = max(maximum_xy, xy), max(maximum_xyz, xyz)
            if row['control_tick'] == '1':
                if row['rotation_only'] == '1' and not initial_rotation_done:
                    first_rotation.append((float(row['time_s']), point, xy, xyz))
                elif first_rotation:
                    initial_rotation_done = True
            if index % 10 == 0 and first_violation is None:
                try:
                    progress.update(point)
                except ValueError as exc:
                    first_violation = {'time_s': float(row['time_s']), 'reason': str(exc),
                                       'xyz': point, 'xy_error_m': xy, 'xyz_error_m': xyz,
                                       'progress_m': progress.progress}
        summary.update({
            'offline_progress_first_violation': first_violation,
            'maximum_route_xy_error_m': maximum_xy, 'maximum_route_xyz_error_m': maximum_xyz,
            'initial_rotation_duration_s': first_rotation[-1][0] if first_rotation else 0.,
            'initial_rotation_max_xy_error_m': max((r[2] for r in first_rotation), default=0.),
            'initial_rotation_max_xyz_error_m': max((r[3] for r in first_rotation), default=0.),
            'first_rotation_end_xyz': first_rotation[-1][1] if first_rotation else None,
            'initial_rotation_translation_m': math.dist(first_rotation[0][1][:2], first_rotation[-1][1][:2])
                                               if first_rotation else 0.,
            'all_four_wheel_samples': sum(r['wheel_contacts'] == '4' for r in rows),
            'samples': len(rows), 'chassis_contact_samples': sum(r['chassis_contacts'] != '0' for r in rows),
        })
        (out/'analysis.json').write_text(json.dumps(summary, indent=2)+'\n')
        cases[treatment] = summary
    tracked_files = [source, mesh, route_file, Path(__file__).resolve(),
                     root/'tools/smb_rotation_hold_replay.cpp',
                     root/'src/gbplanner_tracking/src/pure_pursuit.cpp',
                     root/'src/gbplanner_tracking/src/pure_pursuit_node.cpp',
                     root/'src/gbplanner_tracking/include/gbplanner_tracking/pure_pursuit.hpp',
                     root/'install/gbplanner_smb/lib/libsmb_vehicle.so',
                     root/'install/gbplanner_tracking/lib/libpure_pursuit.so', args.executable]
    summary = {
        'scope': 'original motion5 first local route; original edgar mesh; independent cold starts; one old and one hold',
        'limitations': 'No historical contact/wheel state, ROS Actions, online mapping or execution cancellation. '
                       'PathProgress checked offline at 50 Hz; core continues after a violation to compare endpoint.',
        'settling_s': 1., 'physics_step_s': .002, 'control_step_s': .05, 'budget_s': 60.,
        'parameter_sweep': False, 'position_tolerance_m': .04, 'yaw_tolerance_rad': .1,
        'path_tube_m': .15, 'cases': cases,
        'sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked_files},
    }
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(cases, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
