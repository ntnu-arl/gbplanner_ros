#!/usr/bin/env python3
"""Replay the saved run2/map77 candidate without requesting new local planning."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess

EXPECTED = {
    'run2_map.bin': '342aed4dfbd5329f7f59b8ad35b045e50a23a8ba617c74e30842cddf38ce0942',
    'run2_map.json': '1da7c091715b017bbe609ce64798058938669e1a4ea2d70af1b14237b13af099',
    'run2_history.json': 'e5d1e875ed28697f57489f70dd1647effebd4cdc84e8c3f8f20c4b01ba15e445',
}


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=root / 'results/smb_continuous')
    parser.add_argument('--output', type=Path, default=root / 'results/reference_geometry')
    parser.add_argument('--executable', type=Path, help='Native executable; otherwise use ros2 run')
    parser.add_argument('--prepare-only', action='store_true', help='Verify and extract input without executing')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    hashes = {name: hashlib.sha256((args.source / name).read_bytes()).hexdigest() for name in EXPECTED}
    require(hashes == EXPECTED, 'Source differs from the saved run2/map77 evidence; refusing a replacement input')
    metadata = json.loads((args.source / 'run2_map.json').read_text())
    require(metadata['requested_map_revision'] == 77 and metadata['exact_requested_revision'],
            'Snapshot is not the exact requested map revision 77')
    require(metadata['width'] == 84059 and metadata['height'] == 1 and metadata['point_step'] == 24
            and not metadata['is_bigendian'] and metadata['data_bytes'] == 84059 * 24,
            'Unexpected original raw voxel layout')
    metrics = json.loads((args.source / 'run2_history.json').read_text())['planning_metrics']
    require(len(metrics) == 1, 'Expected one recorded failed candidate')
    metric = metrics[0]
    require(metric['map_revision'] == 77 and metric['tsdf_cells'] == 84059
            and metric['reason'] == 'reference_path_unavailable', 'Recorded failure does not match the snapshot')
    diagnostic = metric['reference_path_diagnostics']
    require(diagnostic['segment'] == 4 and diagnostic['reason'] == 'ground_segment_occupied'
            and diagnostic['coordinates'] == 'planning_height', 'Unexpected recorded failure segment')
    path = metric['candidate_path']
    require(len(path) == 9, 'Expected the original nine recorded candidate waypoints')
    rows = [metric['planning_root'], diagnostic['from'], diagnostic['to'], *path]
    require(all(len(row) == 4 and all(math.isfinite(x) for x in row) for row in rows),
            'Invalid recorded XYZ/yaw coordinates')
    candidate = args.output / 'run2_candidate.txt'
    # repr preserves every double read from the saved metric; there is no
    # projection, interpolation, new random seed, or local-planner call here.
    candidate.write_text('77 84059 9\n' + ''.join(' '.join(repr(x) for x in row) + '\n' for row in rows))
    provenance = {
        'source': str(args.source.resolve()), 'source_sha256': hashes,
        'map_revision': 77, 'exact_requested_revision': True, 'cells': 84059,
        'candidate_source': 'run2_history.json planning_metrics[0].candidate_path',
        'coordinate_precision': 'exact numbers saved in the original planning metric',
        'candidate_sha256': hashlib.sha256(candidate.read_bytes()).hexdigest(),
        'candidate_points': len(path), 'candidate_resampled': False,
        'global_scope': 'Fresh graph from the saved root; historical pre-request global graph was not saved.',
    }
    (args.output / 'input.json').write_text(json.dumps(provenance, ensure_ascii=False, indent=2) + '\n')
    if args.prepare_only:
        print(json.dumps({'status': 'prepared', **provenance}, ensure_ascii=False))
        return
    command = [str(args.executable)] if args.executable else ['ros2', 'run', 'gbplanner_local', 'reference_geometry_replay']
    command += [str((args.source / 'run2_map.bin').resolve()), str(candidate.resolve())]
    process = subprocess.run(command, capture_output=True, text=True, timeout=90)
    (args.output / 'native_stdout.log').write_text(process.stdout)
    (args.output / 'native_stderr.log').write_text(process.stderr)
    try:
        result = json.loads(process.stdout)
    except json.JSONDecodeError:
        result = {'status': 'invalid_native_output', 'stdout_file': 'native_stdout.log'}
    summary = {**provenance, 'command': command, 'returncode': process.returncode, 'native': result}
    (args.output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    require(process.returncode == 0 and result.get('status') == 'passed',
            f'Native replay failed: {result}; evidence saved in {args.output}')
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == '__main__':
    main()
