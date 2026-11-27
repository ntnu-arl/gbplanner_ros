#!/usr/bin/env python3
"""One yaw-governor candidate on motion9's exact first local route and original Edgar mesh.

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
    parser.add_argument('--treatment', choices=('hold_governed', 'hold_governed_terminal'),
                        default='hold_governed')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = root/'results/edgar_mine/motion9_history.json'
    history = json.loads(source.read_text())
    paths = [r for r in history['paths']
             if r['phase'] == 'automatic_exploration' and r['path']]
    record = paths[0]
    route = record['path']
    accepted = next(r for r in history['events']
                    if r['layer'] == 'tracking' and r['state'] == 'TRACKING'
                    and r['reason'] == 'goal_accepted' and r['time_s'] > record['time_s'])
    trajectory = module('corner_trajectory', root/'src/gbplanner_execution/scripts/trajectory_tools.py')
    progress_source = out/'path_progress_checked.py'
    if not progress_source.exists():
        raise RuntimeError('expected frozen pre-change PathProgress in output directory')
    progress_module = module('governor_progress', progress_source)
    speed = .20000000298023224
    samples = trajectory.interpolate(route, speed, .5, .05)
    if len(route) != 10 or len(samples) != accepted['remaining_waypoints'] or len(samples) != 301:
        raise RuntimeError('expected exact motion9 first local route and 301 tracking points')
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
    exe = out/'smb_governor_replay'
    compiler = ['g++', '-std=c++17', '-O2', '-I/usr/include/eigen3',
                '-Isrc/gbplanner_tracking/include', '-Isrc/gbplanner_smb/include',
                'tools/smb_rotation_hold_replay.cpp', 'src/gbplanner_tracking/src/pure_pursuit.cpp',
                '-Linstall/gbplanner_smb/lib', '-Wl,-rpath,'+str(root/'install/gbplanner_smb/lib'),
                '-lsmb_vehicle', '-o', str(exe)]
    build = subprocess.run(compiler, cwd=root, capture_output=True, text=True, timeout=90.)
    (out/'build.log').write_text(build.stdout+build.stderr)
    if build.returncode:
        raise RuntimeError(f'governor replay build failed: see {out/"build.log"}')
    mesh = root/'src/planner_gazebo_sim/collision/edgar_mine.triangles'
    candidate = out/args.treatment
    result = subprocess.run([str(exe), str(mesh), str(route_file), args.treatment, str(candidate)],
                            cwd=root, capture_output=True, text=True, timeout=60.)
    (out/'stdout.log').write_text(result.stdout)
    (out/'stderr.log').write_text(result.stderr)
    if result.returncode:
        raise RuntimeError(f'governor replay failed: {result.stderr}')
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
        'scope': 'one optional rotation_hold_yaw_governor candidate; exact motion9 first local route; original mesh',
        'limitations': 'Independent cold start without historical wheel/contact state, ROS Actions or online mapping. '
                      'PathProgress checked offline at 50 Hz; core continues after violations.',
        'baseline': ('saved motion9_candidate1 ordinary governor; not rerun' if
                     args.treatment == 'hold_governed_terminal' else 'recorded motion9 live mission; not rerun'),
        'parameter_sweep': False,
        'offline_progress_first_violation': first_violation,
        'maximum_route_xy_error_m': maximum_xy, 'maximum_route_xyz_error_m': maximum_xyz,
        'corner_limited_control_ticks': sum(r['corner_limited'] == '1' for r in ticks),
        'initial_pending_corners': int(ticks[0]['pending_corners']),
        'final_pending_corners': int(ticks[-1]['pending_corners']),
        'all_four_wheel_samples': sum(r['wheel_contacts'] == '4' for r in rows),
        'samples': len(rows), 'chassis_contact_samples': sum(r['chassis_contacts'] != '0' for r in rows),
        'position_tolerance_m': .04, 'yaw_tolerance_rad': .1, 'path_tube_m': .15,
    })
    hold_ticks = [r for r in ticks if r['hold_active'] == '1']
    summary['yaw_governor'] = {
        'position_tolerance_m': .04,
        'hold_control_ticks': len(hold_ticks),
        'attenuated_control_ticks': sum(float(r['governor_yaw_scale']) < 1. for r in hold_ticks),
        'zero_requested_yaw_control_ticks': sum(float(r['governor_yaw_scale']) == 0. for r in hold_ticks),
        'maximum_abs_anchor_body_x_m': max((abs(float(r['anchor_body_x'])) for r in rows if r['hold_active'] == '1'), default=0.),
        'maximum_anchor_xy_displacement_m': max((float(r['anchor_body_x'])**2 + float(r['anchor_body_y'])**2 for r in rows if r['hold_active'] == '1'), default=0.)**.5,
        'maximum_abs_applied_w_radps': max(abs(float(r['applied_w'])) for r in rows),
        'maximum_abs_actual_w_radps': max(abs(float(r['body_wz'])) for r in rows),
        'original_path_progress_sha256': hashlib.sha256(progress_source.read_bytes()).hexdigest(),
    }
    aligning_ticks = [r for r in ticks if r['mode'] == '1']
    summary['terminal'] = {
        'governor_enabled': args.treatment == 'hold_governed_terminal',
        'alignment_control_ticks': len(aligning_ticks),
        'alignment_entries': sum(r['mode'] == '1' and (i == 0 or ticks[i-1]['mode'] != '1')
                                 for i, r in enumerate(ticks)),
        'alignment_recovery_exits': sum(r['mode'] == '0' and i > 0 and ticks[i-1]['mode'] == '1'
                                        for i, r in enumerate(ticks)),
        'first_alignment_s': float(aligning_ticks[0]['time_s']) if aligning_ticks else None,
        'attenuated_alignment_ticks': sum(float(r['governor_yaw_scale']) < 1. for r in aligning_ticks),
        'final_mode': int(ticks[-1]['mode']),
        'final_requested_v_w': [float(ticks[-1][k]) for k in ('requested_v', 'requested_w')],
        'final_limited_v_w': [float(ticks[-1][k]) for k in ('limited_v', 'limited_w')],
        'final_applied_w': float(ticks[-1]['applied_w']),
        'final_actual_body_vx_wz': [float(ticks[-1][k]) for k in ('body_vx', 'body_wz')],
    }
    if args.treatment == 'hold_governed_terminal':
        baseline_csv = root/'results/edgar_mine/yaw_governor/motion9_candidate1/hold_governed/trajectory.csv'
        with baseline_csv.open() as f:
            baseline_rows = list(csv.DictReader(f))
        shared = set(rows[0]) & set(baseline_rows[0])
        first_difference = None
        for old, new in zip(baseline_rows, rows):
            different = [k for k in sorted(shared) if abs(float(old[k])-float(new[k])) > 1.e-12]
            if different:
                first_difference = {'time_s': float(new['time_s']), 'columns': different}
                break
        summary['saved_baseline_prefix'] = {
            'file': str(baseline_csv), 'sha256': hashlib.sha256(baseline_csv.read_bytes()).hexdigest(),
            'absolute_tolerance': 1.e-12, 'shared_columns': len(shared),
            'first_difference': first_difference,
        }
    tracked = [source, mesh, route_file, progress_source, Path(__file__).resolve(), exe,
               root/'tools/smb_rotation_hold_replay.cpp',
               root/'src/gbplanner_tracking/src/pure_pursuit.cpp',
               root/'src/gbplanner_tracking/include/gbplanner_tracking/pure_pursuit.hpp',
               root/'install/gbplanner_smb/lib/libsmb_vehicle.so']
    summary['sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    (out/'analysis.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps({k: v for k, v in summary.items() if k != 'sha256'}, indent=2))


if __name__ == '__main__':
    main()
