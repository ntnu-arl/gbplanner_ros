"""Independent curvature-bounded C connector, validated as physical B geometry.

GBPlanner supplies the target pose and estimated map. This optional connector
creates new planning geometry; it never claims to retain an infeasible source
polyline. All six fixed-radius Dubins families are considered in length order.
Terrain support, body clearance and final ROS path admission use the original
native implementation. No truth input or scene geometry is used.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import signal
import subprocess
import time

import numpy as np


FIELDS = ('x', 'y', 'z', 'occupancy', 'tsdf_distance', 'tsdf_weight')


def save(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def packed_map(arrays, path):
    """Copy all observed fields; do not inflate, clear or infer unknown cells."""
    if set(arrays) != set(FIELDS):
        raise ValueError('Native terrain requires the complete observed six-field map')
    count = len(arrays['x'])
    if not count or any(len(arrays[k]) != count for k in FIELDS):
        raise ValueError('Inconsistent observed map fields')
    dtype = np.dtype({'names': FIELDS,
        'formats': ['<f4','<f4','<f4','i1','<f4','<f4'],
        'offsets': [0,4,8,12,16,20], 'itemsize': 24})
    values = np.zeros(count, dtype=dtype)
    for name in FIELDS:
        values[name] = arrays[name]
        if not np.array_equal(values[name], arrays[name]):
            raise ValueError('Map conversion changes field '+name)
    data = values.tobytes()
    Path(path).write_bytes(data)
    return {'path': str(Path(path).resolve()), 'count': count,
            'sha256': hashlib.sha256(data).hexdigest(),
            'field_values_preserved': True, 'point_step': 24}


def terrain_contract(parameters, map_record, candidates):
    """Use the same ground/robot configuration that the real planner loaded."""
    ground = {name: parameters['ground.'+name] for name in (
        'robot_height','max_ground_height','max_inclination','max_negative_inclination',
        'projection_max_length','interpolate_projection_distance','tsdf_projection_mode')}
    if 'ground.ray_projection_mode' in parameters:
        ground['ray_projection_mode'] = parameters['ground.ray_projection_mode']
    if 'ground.hanging_vertex_penalty' in parameters:
        ground['hanging_vertex_penalty'] = parameters['ground.hanging_vertex_penalty']
    return {'map': {**map_record, 'resolution': parameters['map.resolution']},
            'robot': {'size': parameters['robot.size'], 'center_offset': parameters['robot.center_offset']},
            'ground': ground,
            'bounds': {'min': parameters['planning.global_min'], 'max': parameters['planning.global_max']},
            'candidates': [{'id': c['family'], 'base_path': c['base_samples_xyzyaw']} for c in candidates]}


def run_native(root, folder, contract, spin_until=None):
    """Keep the caller's sensor callbacks running while native checks execute."""
    root, folder = Path(root), Path(folder)
    executable = root/'experiments/estimated_mapping/native_terrain/build/native_terrain_validate'
    request, response = folder/'terrain_request.json', folder/'terrain_response.json'
    if response.exists():
        raise FileExistsError('Use a new connector output directory')
    save(request, contract)
    started = time.monotonic()
    process = None
    with (folder/'terrain.log').open('w') as log:
        try:
            process = subprocess.Popen([str(executable), str(request), str(response)],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            if spin_until is None:
                process.wait(timeout=30.)
            else:
                spin_until(lambda: process.poll() is not None, 'native fixed-center terrain validation')
            if process.returncode != 0:
                raise RuntimeError('Native terrain validator exited '+str(process.returncode))
            result = json.loads(response.read_text())
            result['runtime'] = {'wall_seconds': time.monotonic()-started,
                'returncode': process.returncode,
                'executable_sha256': hashlib.sha256(executable.read_bytes()).hexdigest()}
            return result
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait(timeout=3.)


def make_candidates(start, target, offset_m, speed, yaw_rate, sample_period):
    from center_dubins import generate_candidates
    if not (speed > 0 and yaw_rate > 0 and sample_period > 0):
        raise ValueError('Positive speed/rate/period required')
    radius = speed/yaw_rate
    # Arc sampling satisfies the existing linear and angular sample steps.
    step = min(.02, speed*sample_period, radius*yaw_rate*sample_period)
    return generate_candidates(start, target, offset_m=offset_m,
                               radius_m=radius, sample_step_m=step)


def construct(root, folder, start, target, offset_m, follower, parameters,
              arrays, spin_until=None):
    """Construct new geometry and check it on this observed map snapshot.

    Live ROS validation is still required before sending any Action. The
    original upstream polyline is not treated as this connector's output.
    """
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=False)
    generated = make_candidates(start, target, offset_m, float(follower['speed']),
                                float(follower['yaw_rate']), float(follower['sample_period']))
    save(folder/'candidates.json', generated)
    map_record = packed_map(arrays, folder/'map.bin')
    native = run_native(root, folder, terrain_contract(parameters, map_record,
                        generated['candidates']), spin_until)
    by_family = {candidate['family']: candidate for candidate in generated['candidates']}
    feasible = []
    for check in native['candidates']:
        if not check['success']:
            continue
        candidate = by_family[check['id']]
        base = check['projected_base_path']
        original = candidate['base_samples_xyzyaw']
        if len(base) != len(original) or any(
                [p[0], p[1], p[3]] != [q[0], q[1], q[3]] for p, q in zip(base, original)):
            raise AssertionError('Native projection changed candidate XY/yaw or omitted samples')
        center = [[x+offset_m*math.cos(yaw), y+offset_m*math.sin(yaw), z, yaw]
                  for x, y, z, yaw in base]
        feasible.append({'family': candidate['family'], 'base_reference_xyzyaw': base,
            'center_reference_xyzyaw': center, 'center_length_m': candidate['center_length_m'],
            'base_planar_length_m': candidate['base_planar_length_m'],
            'maximum_center_curvature_per_m': candidate['maximum_center_curvature_per_m'],
            'segments': candidate['segments'], 'all_candidate_xyz_retained_in_order': True,
            'terminal_yaw_preserved': True, 'native_check': check})
    feasible.sort(key=lambda c: c['center_length_m'])
    result = {'status': 'completed', 'truth_used': False,
              'scope': 'Independent experiment; original production planning and default entrypoints unchanged',
              'start_base_xyzyaw': start, 'target_base_xyzyaw': target,
              'radius_m': generated['radius_m'], 'offset_m': offset_m,
              'map_record': map_record, 'native': native, 'feasible': feasible,
              'mathematically_rejected': generated['rejected_families'],
              'final_live_ros_validation_required': True,
              'source_sha256': {name: hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()
                                for name in ('center_path.py', 'center_dubins.py')}}
    save(folder/'construction.json', result)
    return result
