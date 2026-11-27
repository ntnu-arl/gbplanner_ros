#!/usr/bin/env python3
"""Compare the recorded third route with one fixed base-yaw feedback setting.

Run inside the sourced ROS workspace after building the vehicle library and
smb_terminal_replay. This is a cold-start physical replay on an infinite floor;
map350 is a frozen failure snapshot, not the map used to plan the route.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess


def cross_track(point, path):
    best = math.inf
    for a, b in zip(path, path[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        norm = dx*dx+dy*dy
        t = max(0., min(1., ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/norm)) if norm else 0.
        best = min(best, math.hypot(point[0]-a[0]-t*dx, point[1]-a[1]-t*dy))
    return best


def box_status(point, cells):
    # Closed box/cell intersection, matching SparseVoxelMap::path(p,p,size).
    center = [point[0], point[1], point[2]+.015]
    half = [.585, .585, .125]
    low = [math.ceil((c-h)/.1)-1 for c, h in zip(center, half)]
    high = [math.floor((c+h)/.1) for c, h in zip(center, half)]
    occupied, unknown = 0, 0
    for x in range(low[0], high[0]+1):
        for y in range(low[1], high[1]+1):
            for z in range(low[2], high[2]+1):
                status = cells.get((x, y, z), -1)
                occupied += status == 100
                unknown += status == -1
    return occupied, unknown


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=root/'results/yaw_tracking')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    source = root/'results/smb_continuous'
    history_path = source/'run10_history.json'
    history = json.loads(history_path.read_text())
    record = history['commanded_paths'][3]
    path = record['path']
    if len(path) != 4 or abs(path[-1][0]-(-4.251237495230816)) > 1.e-12:
        raise RuntimeError('Expected original third route; refusing substitute path')
    route_file = args.output/'run10_third_route.csv'
    with route_file.open('w') as f:
        f.write('x,y,z,yaw\n')
        for row in path:
            f.write(','.join(repr(v) for v in row)+'\n')
    metadata = json.loads((source/'run10_map.json').read_text())
    if metadata['requested_map_revision'] != 350 or not metadata['exact_requested_revision']:
        raise RuntimeError('Expected exact failure snapshot map350')
    data = (source/'run10_map.bin').read_bytes()
    if metadata['point_step'] != 24 or metadata['is_bigendian'] or len(data) != metadata['width']*24:
        raise RuntimeError('Unexpected saved voxel layout')
    cells = {}
    for x, y, z, occupancy, distance, weight in struct.iter_unpack('<fffb3xff', data):
        key = tuple(math.floor(v/.1) for v in (x, y, z))
        if key in cells:
            raise RuntimeError('Duplicate map voxel')
        cells[key] = occupancy
    results = {}
    for name, gain in [('open_loop', 0.), ('yaw_feedback', 2.)]:
        out = args.output/name
        command = [str(args.executable.resolve()), str(out.resolve()), str(route_file.resolve()),
                   '0', 'position', str(gain)]
        process = subprocess.run(command, text=True, capture_output=True, timeout=90)
        out.mkdir(exist_ok=True)
        (out/'stdout.log').write_text(process.stdout)
        (out/'stderr.log').write_text(process.stderr)
        result = json.loads((out/'summary.json').read_text())
        result['status'] = 'passed' if result['passed'] else 'failed'
        samples = []
        with (out/'trajectory.csv').open() as f:
            for row in csv.DictReader(f):
                if row['phase'] == 'route' and row['control_tick'] == '1':
                    samples.append(row)
        offsets = [cross_track([float(row['x_m']), float(row['y_m'])], path) for row in samples]
        statuses = [box_status([float(row[k]) for k in ('x_m', 'y_m', 'z_m')], cells)
                    for row in samples]
        occupied = [i for i, value in enumerate(statuses) if value[0]]
        result.update({
            'process_returncode': process.returncode,
            'control_sample_count': len(samples),
            'max_cross_track_m': max(offsets),
            'max_cross_track_before_terminal_m': max(
                (v for v, r in zip(offsets, samples) if r['mode'] == 'TRACKING'), default=0.),
            'frozen_map350_box_occupied_samples': len(occupied),
            'frozen_map350_box_unknown_samples': sum(v[1] > 0 for v in statuses),
            'first_frozen_map350_occupied_s': float(samples[occupied[0]]['route_time_s']) if occupied else None,
            'frozen_map_query': 'closed physical 1.17 x 1.17 x .25 m box, base offset z=.015; control samples only',
        })
        results[name] = result
    summary = {
        'status': 'passed' if results['yaw_feedback']['status'] == 'passed'
                  and not results['yaw_feedback']['frozen_map350_box_occupied_samples'] else 'failed',
        'scope': 'same recorded command polyline; independent cold starts and 1 s settling; unchanged PP and terminal laws',
        'limitations': 'No historical wheel/contact/integrator state, online mapping or ROS Actions. Static floor physics; frozen map350 postdates planning.',
        'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in (history_path, source/'run10_map.json', source/'run10_map.bin')},
        'implementation_sha256': {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in (root/'src/gbplanner_smb/src/smb_vehicle.cpp',
                                            root/'src/gbplanner_smb/include/gbplanner_smb/smb_vehicle.hpp',
                                            root/'tools/smb_terminal_replay.cpp', Path(__file__).resolve(),
                                            root/'install/gbplanner_smb/lib/libsmb_vehicle.so',
                                            root/'install/gbplanner_tracking/lib/libpure_pursuit.so')},
        'executable_sha256': hashlib.sha256(args.executable.read_bytes()).hexdigest(),
        'route_points': len(path), 'route_start_time_s': record['time_s'],
        'feedback_gain_per_s': 2., 'parameter_sweep': False,
        'cases': results,
    }
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps({k: {q: v[q] for q in ('status', 'elapsed_s', 'final_distance_m',
                     'max_cross_track_m', 'frozen_map350_box_occupied_samples') if q in v}
                      for k, v in results.items()}))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
