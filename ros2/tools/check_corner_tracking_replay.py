#!/usr/bin/env python3
"""One candidate on motion7's exact third local route and original Edgar mesh.

The recorded live run is the baseline. This probe does not rerun it or sweep
parameters. A fresh chassis cannot reproduce historical contact/wheel state;
online mapping, ROS Actions and cancellation are exercised in the next mission.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess

from check_rotation_hold_replay import distance, module


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = root/'results/edgar_mine/motion7_history.json'
    history = json.loads(source.read_text())
    paths = [r for r in history['paths']
             if r['phase'] == 'automatic_exploration' and r['path']]
    record = paths[2]
    route = record['path']
    accepted = next(r for r in history['events']
                    if r['layer'] == 'tracking' and r['state'] == 'TRACKING'
                    and r['reason'] == 'goal_accepted' and r['time_s'] > record['time_s'])
    trajectory = module('corner_trajectory', root/'src/gbplanner_execution/scripts/trajectory_tools.py')
    progress_module = module('corner_progress', root/'src/gbplanner_local/scripts/path_progress.py')
    speed = .20000000298023224
    samples = trajectory.interpolate(route, speed, .5, .05)
    if len(route) != 5 or len(samples) != accepted['remaining_waypoints'] or len(samples) != 168:
        raise RuntimeError('expected exact motion7 third local route and 168 tracking points')
    route_file = out/'tracking_route.csv'
    with route_file.open('w') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(['x', 'y', 'z', 'yaw'])
        writer.writerows(samples)
    (out/'route.json').write_text(json.dumps({
        'source_time_s': record['time_s'], 'tracking_accepted_time_s': accepted['time_s'],
        'measured_start': route[0], 'commanded_path': route,
        'tracking_points': len(samples), 'speed_mps': speed,
    }, indent=2)+'\n')
    exe = out/'smb_corner_replay'
    compiler = ['g++', '-std=c++17', '-O2', '-I/usr/include/eigen3',
                '-Isrc/gbplanner_tracking/include', '-Isrc/gbplanner_smb/include',
                'tools/smb_rotation_hold_replay.cpp', 'src/gbplanner_tracking/src/pure_pursuit.cpp',
                '-Linstall/gbplanner_smb/lib', '-Wl,-rpath,'+str(root/'install/gbplanner_smb/lib'),
                '-lsmb_vehicle', '-o', str(exe)]
    build = subprocess.run(compiler, cwd=root, capture_output=True, text=True, timeout=90.)
    (out/'build.log').write_text(build.stdout+build.stderr)
    if build.returncode:
        raise RuntimeError(f'corner replay build failed: see {out/"build.log"}')
    mesh = root/'src/planner_gazebo_sim/collision/edgar_mine.triangles'
    candidate = out/'hold_corners'
    result = subprocess.run([str(exe), str(mesh), str(route_file), 'hold_corners', str(candidate)],
                            cwd=root, capture_output=True, text=True, timeout=60.)
    (out/'stdout.log').write_text(result.stdout)
    (out/'stderr.log').write_text(result.stderr)
    if result.returncode:
        raise RuntimeError(f'corner replay failed: {result.stderr}')
    summary = json.loads((candidate/'summary.json').read_text())
    with (candidate/'trajectory.csv').open() as f:
        rows = list(csv.DictReader(f))
    progress = progress_module.PathProgress([p[:3] for p in route])
    first_violation = None
    maximum_xy = maximum_xyz = 0.
    for index, row in enumerate(rows):
        point = [float(row[k]) for k in ('x', 'y', 'z')]
        xy, xyz = distance(point, route, 2), distance(point, route, 3)
        maximum_xy, maximum_xyz = max(maximum_xy, xy), max(maximum_xyz, xyz)
        if index % 10 == 0 and first_violation is None:
            try:
                progress.update(point)
            except ValueError as exc:
                first_violation = {'time_s': float(row['time_s']), 'reason': str(exc),
                                   'xyz': point, 'xy_error_m': xy, 'xyz_error_m': xyz,
                                   'progress_m': progress.progress}
    ticks = [r for r in rows if r['control_tick'] == '1']
    summary.update({
        'scope': 'one optional preserve_corners candidate; exact motion7 third local route; original mesh',
        'limitations': 'Independent cold start without historical wheel/contact state, ROS Actions or online mapping. '
                      'PathProgress checked offline at 50 Hz; core continues after violations.',
        'baseline': 'recorded motion7 live mission; not rerun', 'parameter_sweep': False,
        'offline_progress_first_violation': first_violation,
        'maximum_route_xy_error_m': maximum_xy, 'maximum_route_xyz_error_m': maximum_xyz,
        'corner_limited_control_ticks': sum(r['corner_limited'] == '1' for r in ticks),
        'initial_pending_corners': int(ticks[0]['pending_corners']),
        'final_pending_corners': int(ticks[-1]['pending_corners']),
        'all_four_wheel_samples': sum(r['wheel_contacts'] == '4' for r in rows),
        'samples': len(rows), 'chassis_contact_samples': sum(r['chassis_contacts'] != '0' for r in rows),
        'position_tolerance_m': .04, 'yaw_tolerance_rad': .1, 'path_tube_m': .15,
    })
    tracked = [source, mesh, route_file, Path(__file__).resolve(), exe,
               root/'tools/smb_rotation_hold_replay.cpp',
               root/'src/gbplanner_tracking/src/pure_pursuit.cpp',
               root/'src/gbplanner_tracking/include/gbplanner_tracking/pure_pursuit.hpp',
               root/'install/gbplanner_smb/lib/libsmb_vehicle.so']
    summary['sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    (out/'analysis.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps({k: v for k, v in summary.items() if k != 'sha256'}, indent=2))


if __name__ == '__main__':
    main()
