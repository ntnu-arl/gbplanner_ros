#!/usr/bin/env python3
"""Bounded, offline terminal-pose feasibility on recorded observations.

No ROS command, map update, or simulator launch occurs here. The measured
front-axis control offset is a local model hypothesis, not vehicle calibration.
Candidates must subsequently pass the production C++ segment validator and the
original monotonic PathProgress cursor before considering native execution.
"""
import argparse
from collections import Counter
import copy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'src/gbplanner_local/scripts'))
from inspect_smb_tsdf_projection import RecordedMap
from pittsburgh_mesh.inspect_support import body_box, cast_down
from path_progress import PathProgress


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def route_distance(point, route):
    best = math.inf
    for a, b in zip(route, route[1:]):
        delta = [y-x for x, y in zip(a[:3], b[:3])]
        norm = sum(v*v for v in delta)
        fraction = max(0., min(1., sum((p-x)*v for p, x, v in zip(point, a, delta))/norm)) if norm else 0.
        best = min(best, math.dist(point, [x+fraction*v for x, v in zip(a, delta)]))
    return best


class SavedGeometry:
    def __init__(self, mapping, parameters, route, base_z):
        self.mapping, self.parameters, self.route, self.base_z = mapping, parameters, route, base_z
        self.counts = Counter()
        self.body_cache, self.support_region_cache = {}, {}

    def body_free(self, x, y):
        # A fixed-Z, axis-aligned box touches the same cells throughout one
        # interval pair. Obtain the exact inclusive arithmetic once per pair.
        size = self.parameters['robot.size']
        offset = self.parameters['robot.center_offset']
        center = [x+offset[0], y+offset[1], self.base_z+offset[2]]
        resolution = self.mapping.resolution
        key = []
        for c, s in zip(center, size):
            origin = math.floor(c/resolution)
            origin = math.floor(((origin+.5)*resolution)/resolution)
            radius = math.floor(.5*s/resolution)+1
            touched = [i for i in range(origin-radius, origin+radius+1)
                       if not (c < i*resolution-.5*s or c > i*resolution-.5*s+resolution+s)]
            key.extend((touched[0], touched[-1]))
        key = tuple(key)
        if key not in self.body_cache:
            self.body_cache[key] = body_box(self.mapping, center, size)['status'] == 'free'
        return self.body_cache[key]

    def support_free(self, x, y):
        p, mapping = self.parameters, self.mapping
        r = mapping.resolution
        ix, iy = (math.floor(v/r-.5) for v in (x, y))
        region = ix, iy
        if region not in self.support_region_cache:
            # Sufficient whole-square proof: all simplex vertices are observed,
            # positive above one horizontal layer and negative at the next.
            # Convex interpolation then gives an observed zero inside the
            # original +/-one-voxel height interval for every point in the
            # square. An incomplete proof falls back to the exact ray below.
            expected = self.base_z-p['ground.robot_height']
            start_z = self.base_z+p['ground.max_ground_height']-p['ground.robot_height']
            lo, hi = math.floor(expected/r-.5), math.floor(start_z/r-.5)+1
            columns = [[mapping.cells.get((i, j, k)) for k in range(lo, hi+1)]
                       for i in (ix, ix+1) for j in (iy, iy+1)]
            covered = all(cell is not None and cell['weight'] >= 1e-6
                          for column in columns for cell in column)
            scale = max((abs(cell['distance']) for column in columns for cell in column if cell), default=0.)
            # A numerical zero on a voxel-centre plane can be a tiny positive
            # value after the ray's grid arithmetic. Do not infer its bracket:
            # retain a strict signed margin or use the actual vertical query.
            margin = r*1e-6 + 64.*sys.float_info.epsilon*scale
            proven = (covered and all(column[0]['distance'] < -margin for column in columns)
                      and all(cell['distance'] > margin for column in columns for cell in column[1:])
                      and (lo+.5)*r >= start_z-p['ground.projection_max_length']
                      and (lo+1.5)*r < start_z
                      and abs((lo+.5)*r-expected) <= r
                      and abs((lo+1.5)*r-expected) <= r)
            self.support_region_cache[region] = proven
        if self.support_region_cache[region]:
            return True
        ray = cast_down(mapping, [x, y, self.base_z+p['ground.max_ground_height']-p['ground.robot_height']],
                        p['ground.projection_max_length'])
        return ray['status'] == 'supported' and abs(ray['surface_z']+p['ground.robot_height']-self.base_z) <= r+1e-8

    def __call__(self, pose):
        self.counts['queries'] += 1
        x, y, _ = pose
        if route_distance([x, y, self.base_z], self.route) > .15:
            self.counts['outside_original_tube'] += 1
            return False
        if not self.body_free(x, y):
            self.counts['body_not_free'] += 1
            return False
        if not self.support_free(x, y):
            self.counts['support_missing'] += 1
            return False
        self.counts['accepted'] += 1
        return True


def main():
    from pittsburgh_mesh.pose_maneuver import Limits, Grid, search_maneuver
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--wall-budget', type=float, default=45.)
    parser.add_argument('--max-nodes', type=int, default=15000)
    parser.add_argument('--heuristic-weight', type=float, default=1.,
                        help='Weighted best-first priority; values above one seek feasibility without optimality')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a new output path; previous results are retained')
    base = ROOT/'results/pittsburgh_port'
    run_path, history_path = base/'autonomy3.json', base/'autonomy3_history.json'
    run, history = json.loads(run_path.read_text()), json.loads(history_path.read_text())
    motion = json.loads((base/'autonomy3_motion_analysis.json').read_text())
    entry = motion['local']['terminal_disk']['first_inside']['observation']
    route = motion['local']['commanded_path']['path']
    map_path = base/'autonomy3_failure_map.bin'
    parameters = run['configuration']['effective_planner_parameters']
    mapping = RecordedMap(run['failure_map'], map_path.read_bytes(), parameters['map.resolution'])
    # Keep the measured body height. Production validation must still verify
    # support and every commanded segment; no projected-height map is invented.
    base_z = entry['pose'][2]
    geometry = SavedGeometry(mapping, parameters, route, base_z)
    start = tuple(entry['pose'][i] for i in (0, 1, 3))
    goal = tuple(route[-1][i] for i in (0, 1, 3))
    # The original Action has already consumed this measured prefix. Each
    # search branch inherits its cursor; a validity callback must not advance
    # one shared mutable cursor while exploring unrelated alternatives.
    inherited = PathProgress([p[:3] for p in route])
    phase_start = motion['local']['progress_reconstruction']['start_observer_sim_s']
    prefix = [row for row in history['odometry']
              if row['sim_s'] >= phase_start and row['stamp_ns'] <= entry['stamp_ns']]
    for row in prefix:
        inherited.update(row['pose'][:3])
    constraint_counts = Counter()

    def advance_progress(state, old, new):
        probe = copy.copy(inherited)
        probe.progress = state
        probe.previous = (old.x, old.y, base_z)
        try:
            probe.update((new.x, new.y, base_z))
        except ValueError as error:
            constraint_counts[str(error)] += 1
            return None
        return probe.progress

    limits = Limits(max_wall_time=args.wall_budget, max_nodes=args.max_nodes,
                    heuristic_weight=args.heuristic_weight)
    result = search_maneuver(start, goal, geometry, control_offset=.2159,
                            limits=limits, grid=Grid(),
                            initial_velocity=(entry['body_linear_velocity'][0], entry['yaw_rate']),
                            initial_constraint_state=inherited.progress,
                            advance_constraint=advance_progress,
                            constraint_key=lambda state: state)
    candidate_checks = None
    if result.success:
        progress = copy.copy(inherited)
        initial_cursor = progress.progress
        failures, maximum_tube = [], 0.
        for index, sample in enumerate(result.samples):
            base_point = [sample.x, sample.y, base_z]
            maximum_tube = max(maximum_tube, route_distance(base_point, route))
            ray = cast_down(mapping, [sample.x, sample.y,
                base_z+parameters['ground.max_ground_height']-parameters['ground.robot_height']],
                parameters['ground.projection_max_length'])
            if ray['status'] != 'supported' or abs(ray['surface_z']+parameters['ground.robot_height']-base_z) > mapping.resolution+1e-8:
                failures.append({'sample': index, 'reason': 'exact_ground_support'})
                break
            try:
                progress.update(base_point)
            except ValueError as error:
                failures.append({'sample': index, 'reason': str(error)})
                break
        candidate_checks = {'status': 'passed' if not failures else 'failed',
            'recorded_prefix_samples': len(prefix), 'initial_progress_m': initial_cursor,
            'final_progress_m': progress.progress, 'maximum_route_xyz_error_m': maximum_tube,
            'failures': failures, 'exact_ground_rays_used': True,
            'production_cpp_segment_check': 'pending', 'native_execution': False}
    report = {'scope': 'Offline bounded SE(2) feasibility under an explicit local ICR model',
              'native_run': False, 'production_behavior_changed': False, 'map_mutated': False,
              'source_run_status': run['status'], 'map_association': run['failure_map'],
              'map_timing': 'The failure snapshot is later than this terminal entry; this is a design feasibility study, not an online replay.',
              'start_odometry': entry, 'goal_base_pose': route[-1], 'candidate_base_z': base_z,
              'control_offset_m': .2159, 'limits': asdict(limits),
              'geometry_counters': dict(geometry.counts), 'search': asdict(result),
              'sequential_constraint': {'enabled': True,
                  'recorded_prefix_samples': len(prefix),
                  'initial_progress_m': inherited.progress,
                  'label_key': 'Exact retained progress, not a progress bin',
                  'rejections': dict(constraint_counts)},
              'candidate_checks': candidate_checks,
              'source_hashes': {str(p.relative_to(ROOT)): digest(p) for p in
                  (run_path, history_path, map_path, Path(__file__),
                   Path(__file__).with_name('pose_maneuver.py'),
                   ROOT/'src/gbplanner_local/scripts/path_progress.py')},
              'limitations': ['A search budget exhausted result is not proof of infeasibility.',
                  'Fixed control offset is inferred locally, not valid for all contact states.',
                  'Sampled kinematics cannot establish collision-free continuous physics.',
                  'Candidates need current C++ swept-segment validation and original progress replay before native integration.']}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'output': str(args.output), 'status': result.status, 'reason': result.reason,
                      'geometry': dict(geometry.counts)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
