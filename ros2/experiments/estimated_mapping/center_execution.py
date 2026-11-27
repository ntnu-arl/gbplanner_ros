"""Optional fixed-center planning for the isolated estimated-map experiment.

Only the experiment runner imports this module. It constructs new geometry
from the current estimated pose; no production launch or planner is replaced.
"""
from pathlib import Path
import json
import math
import signal
import subprocess
import sys
import time

from center_path import FIELDS, construct, packed_map, save


def plan_snapshot(root, out, start, raw_path, offset_m, follower, parameters, arrays):
    """CPU/file work runs outside the node that owns ongoing sensor callbacks."""
    root, out = Path(root), Path(out)
    folder = out/'center_connector'
    direct = construct(root, folder, start, raw_path[-1], offset_m, follower,
                       parameters, arrays)
    evidence = {'geometry': 'center_dubins', 'upstream_base_path': raw_path,
                'initial_base_xyzyaw': start, 'upstream_terminal_pose_fixed': raw_path[-1],
                'direct_native_results': [{k: c[k] for k in ('id','success','reason','failure_index')}
                                          for c in direct['native']['candidates']],
                'source_polyline_retention_claimed': False,
                'meaning': 'GBPlanner supplies a guide and endpoint; the independent connector plans new B geometry.'}
    candidates = direct['feasible']
    if not candidates:
        from center_via import construct_via
        contract = json.loads((folder/'terrain_request.json').read_text())
        contract.pop('candidates')
        via = construct_via(root, out/'center_via', start, raw_path[-1], raw_path,
                            offset_m, follower, parameters, contract)
        candidates = via['feasible']
        evidence['via_result'] = 'center_via/construction.json'
    evidence['native_feasible_count'] = len(candidates)
    save(out/'center_planning.json', evidence)
    return {'feasible': candidates, 'evidence': evidence}


def prepare(node, root, out, initial_probe, start, raw_path, offset_m, follower):
    """Run construction in a child while the recorder keeps spinning."""
    import yaml
    root, out = Path(root), Path(out)
    if initial_probe.get('exact_target_reported') is not True:
        raise RuntimeError('The upstream planner did not reach the requested exact target')
    cloud = node.mapping_stage.cloud
    if cloud is None or cloud.header.frame_id != 'map':
        raise RuntimeError('No observed map-frame voxel cloud')
    parameters = yaml.safe_load((out/'planner_probe.yaml').read_text())[
        'gbplanner_local']['ros__parameters']
    inputs = out/'center_input'
    inputs.mkdir()
    map_record = packed_map(node.mapping_stage.cloud_arrays(cloud), inputs/'map.bin')
    request = {'root': str(root), 'out': str(out), 'start': start, 'raw_path': raw_path,
               'offset_m': offset_m, 'follower': follower, 'parameters': parameters,
               'map_record': map_record,
               'map_stamp_ns': cloud.header.stamp.sec*1000000000+cloud.header.stamp.nanosec}
    request_path, response_path = inputs/'request.json', inputs/'result.json'
    save(request_path, request)
    started = time.monotonic()
    process = None
    with (inputs/'worker.log').open('w') as log:
        try:
            process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()),
                '--request', str(request_path), '--output', str(response_path)],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            node.spin_until(lambda: process.poll() is not None, 'independent center planning worker')
            if process.returncode:
                raise RuntimeError('Center planning worker failed; see center_input/worker.log')
            response = json.loads(response_path.read_text())
            evidence = response['evidence']
            evidence.update(map_stamp_ns=request['map_stamp_ns'],
                            worker_wall_seconds=time.monotonic()-started,
                            sensor_callbacks_spun_during_construction=True)
            save(out/'center_planning.json', evidence)
            return response['feasible'], evidence
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait(timeout=3.)


def stage(candidate, current, raw_path, revision):
    """Carry every projected planner output sample to the existing Action."""
    base, center = candidate['base_reference_xyzyaw'], candidate['center_reference_xyzyaw']
    return {'kind': 'return', 'raw_planner_base_xyzyaw': raw_path,
            'base_reference_xyzyaw': base, 'center_reference_xyzyaw': center,
            'target_base_xyzyaw': base[-1], 'search_map_revision': revision,
            'initial_center_gap_m': math.dist(current['center_xyzyaw'][:2], center[0][:2]),
            'initial_yaw_gap_rad': math.atan2(math.sin(center[0][3]-current['base_xyzyaw'][3]),
                                             math.cos(center[0][3]-current['base_xyzyaw'][3])),
            'sampling': {'all_new_planner_xyz_retained_in_order': True,
                         'terminal_yaw_preserved': True,
                         'new_planning_geometry': True},
            'connector_metadata': {k: v for k,v in candidate.items()
                if k not in ('base_reference_xyzyaw','center_reference_xyzyaw','native_check')},
            'meaning': 'New fixed-radius C geometry, terrain-projected B samples retained in full; no preturn or post-turn resampling.'}


if __name__ == '__main__':
    import argparse
    import hashlib
    import numpy as np
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    request = json.loads(args.request.read_text())
    data = Path(request['map_record']['path']).read_bytes()
    if hashlib.sha256(data).hexdigest() != request['map_record']['sha256']:
        raise ValueError('Observed map changed before worker read it')
    dtype = np.dtype({'names': FIELDS, 'formats': ['<f4','<f4','<f4','i1','<f4','<f4'],
                      'offsets': [0,4,8,12,16,20], 'itemsize': 24})
    values = np.frombuffer(data, dtype=dtype)
    if len(values) != request['map_record']['count']:
        raise ValueError('Map count changed before worker read it')
    result = plan_snapshot(request['root'], request['out'], request['start'], request['raw_path'],
        request['offset_m'], request['follower'], request['parameters'], {k: values[k] for k in FIELDS})
    save(args.output, result)
