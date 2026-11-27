#!/usr/bin/env python3
"""Finite native-SMB frozen-map terrain/body probes; no ROS node or simulator.

Uses recorded effective parameters, complete observed visit history and a
finite new LocalSampler stream around explicitly selected graph vertices.
Graph messages omit private visitation types, so selected vertices are probe
origins, not asserted background centers. This is not timer/cluster/RNG replay.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shlex
import subprocess


def require(ok, message):
    if not ok:
        raise ValueError(message)


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot', type=Path)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--random-draws', type=int, default=4000)
    parser.add_argument('--max-expansions', type=int, default=16)
    parser.add_argument('--seed', type=int, default=20261101)
    parser.add_argument('--center-vertex', type=int, action='append', required=True,
                        help='Saved graph vertex used as a probe origin; does not assert its private type')
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    require(0 <= args.random_draws <= 100000 and 1 <= args.max_expansions <= 64, 'Invalid finite probe limits')
    require(0 <= args.seed <= 4294967295, 'Seed must fit uint32')
    snapshot = json.loads(args.snapshot.read_text())
    run = json.loads(args.run.read_text())
    metadata = snapshot['accepted_map']
    map_file = args.snapshot.parent/Path(metadata['data_file']).name
    graph_file = args.snapshot.parent/Path(snapshot['global_graph']['data_file']).name
    map_bytes, graph_bytes = map_file.read_bytes(), graph_file.read_bytes()
    require(hashlib.sha256(map_bytes).hexdigest() == metadata['sha256'], 'Saved map hash mismatch')
    require(hashlib.sha256(graph_bytes).hexdigest() == snapshot['global_graph']['sha256'], 'Saved graph hash mismatch')
    layout = [(f['name'], f['offset'], f['datatype'], f['count']) for f in metadata['fields']]
    require(layout == [('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 1), ('occupancy', 12, 1, 1),
                       ('tsdf_distance', 16, 7, 1), ('tsdf_weight', 20, 7, 1)]
            and not metadata['is_bigendian'] and metadata['height'] == 1 and metadata['point_step'] == 24
            and len(map_bytes) == metadata['row_step'] == metadata['width']*24, 'Unsupported map layout')
    graph = json.loads(graph_bytes)
    params = dict(run['configuration']['effective_planner_parameters'])
    readback = run['background_observation']['profile_readback']
    require(not readback['mismatches'] and all(readback['actual'].get(k) == v for k, v in params.items()),
            'Recorded effective parameters disagree with live node readback')
    history_metadata = snapshot['state_history']
    history_file = args.snapshot.parent/Path(history_metadata['data_file']).name
    history_bytes = history_file.read_bytes()
    require(hashlib.sha256(history_bytes).hexdigest() == history_metadata['sha256'], 'Saved history hash mismatch')
    history = json.loads(history_bytes)
    require(history_metadata['complete'] and not history['truncated'] and history['first_index'] == 0
            and history['total_states'] == len(history['states']) == history_metadata['saved_states'],
            'Complete observed visited history is required; home is not a substitute')
    require(history['coordinate_convention'] == 'planning_height'
            and history['background_exclusion_radius_m'] == 5, 'Unexpected history coordinate/radius semantics')
    visits = history['states']
    require(all(len(state) == 4 and all(math.isfinite(v) for v in state) for state in visits), 'Invalid history state')
    require(params['robot.type'] == 'kGround' and params['map.storage'] == 'sparse', 'Expected sparse ground configuration')
    require(params.get('sampling.space_type', 'kCuboid') == 'kCuboid'
            and params.get('planning.global_type', 'kCuboid') == 'kCuboid', 'Expected cuboid bounds')
    require(params['ground.tsdf_projection_mode'] == 'zero_crossing_simplex', 'Probe expects the recorded simplex terrain model')
    mode = run['background_observation']['request']['bound_mode']
    positions = []
    for vertex in graph['vertices']:
        p, q = vertex['pose']['position'], vertex['pose']['orientation']
        positions.append([p['x'], p['y'], p['z'], math.atan2(2*(q['w']*q['z']+q['x']*q['y']),
                                                            1-2*(q['y']**2+q['z']**2))])
    require([v['id'] for v in graph['vertices']] == list(range(len(positions))), 'Expected contiguous saved vertex IDs')
    require(all(0 <= i < len(positions) for i in args.center_vertex), 'Probe origin vertex is missing')
    center_ids = list(dict.fromkeys(args.center_vertex))
    # Native background centroids carry zero heading. Keep the explicit
    # probe's XYZ anchor while preserving that sampler heading convention.
    centers = [positions[i][:3]+[0.0] for i in center_ids]
    literal = lambda value: ('{'+','.join(literal(v) for v in value)+'}' if isinstance(value, list)
                             else json.dumps(value, allow_nan=False))
    lines = ['inline gbplanner2::Parameters diagnosticParameters(){gbplanner2::Parameters p;p.ground_robot=true;']
    fields = {'local_min': 'sampling.min', 'local_max': 'sampling.max', 'local_rotations': 'sampling.rotations',
              'distribution': 'sampling.distribution', 'seed': 'sampling.seed', 'center_offset': 'robot.center_offset',
              'edge_min': 'planning.edge_min', 'edge_max': 'planning.edge_max', 'overshoot': 'planning.overshoot',
              'neighbor_radius': 'planning.neighbor_radius', 'neighbor_min': 'planning.neighbor_min',
              'neighbor_max': 'planning.neighbor_max', 'graph_mode': 'planning.graph_mode',
              'ground.max_ground_height': 'ground.max_ground_height', 'ground.robot_height': 'ground.robot_height',
              'ground.max_inclination': 'ground.max_inclination', 'ground.max_negative_inclination': 'ground.max_negative_inclination',
              'ground.projection_max_length': 'ground.projection_max_length',
              'ground.interpolate_projection_distance': 'ground.interpolate_projection_distance'}
    for field, key in fields.items():
        lines.append(f'p.{field}={literal(params[key])};')
    lines += ['p.ground.tsdf_projection_mode=gbplanner2::GroundTsdfProjectionMode::ZeroCrossingSimplex;',
              'gbplanner2::RobotBounds body;', 'body.physical_size='+literal(params['robot.size'])+';',
              'body.extension='+literal(params['robot.size_extension'])+';',
              'body.min_extension='+literal(params['robot.size_extension_min'])+';',
              'body.relax_ratio='+literal(params['robot.relax_ratio'])+';',
              f'p.robot_size=body.size({mode});p.point_robot=body.pointMode({mode});']
    for index, name in enumerate(('x', 'y', 'z', 'heading')):
        for key, member in (('distribution', 'distribution'), ('min', 'minimum'), ('max', 'maximum'),
                            ('mean', 'mean'), ('scale', 'scale'), ('constant', 'constant')):
            source = f'sampling.{name}.{key}'
            if source in params:
                lines.append(f'p.axis_sampling[{index}].{member}={literal(params[source])};')
    lines += [f'p.seed={args.seed};p.validate();return p;}}',
              'inline std::vector<StateVec> diagnosticVisits(){return '+literal(visits)+';}',
              'inline std::vector<StateVec> diagnosticCenters(){return '+literal(centers)+';}',
              'inline std::vector<int> diagnosticCenterIds(){return '+literal(center_ids)+';}',
              'inline std::vector<StateVec> diagnosticVertices(){return '+literal(positions)+';}',
              f'inline int diagnosticRandomDraws(){{return {args.random_draws};}}',
              f'inline int diagnosticMaxExpansions(){{return {args.max_expansions};}}',
              'inline double diagnosticResolution(){return '+literal(params['map.resolution'])+';}',
              'inline gbplanner2::GlobalBound diagnosticBound(){return gbplanner2::GlobalBound::axisAligned('
              +literal(params['planning.global_min'])+','+literal(params['planning.global_max'])+');}',
              'inline planner_msgs::msg::Graph diagnosticGraph(){planner_msgs::msg::Graph g;']
    for vertex in graph['vertices']:
        lines.append('{planner_msgs::msg::Vertex v;v.id='+str(vertex['id'])+';')
        for group, axes in (('position', 'xyz'), ('orientation', 'xyzw')):
            for axis in axes:
                lines.append(f'v.pose.{group}.{axis}={literal(vertex["pose"][group][axis])};')
        lines += ['v.is_frontier='+literal(vertex['is_frontier'])+';', 'g.vertices.push_back(v);}']
    for edge in graph['edges']:
        lines += ['{planner_msgs::msg::Edge e;', f'e.source_id={edge["source_id"]};e.target_id={edge["target_id"]};',
                  'e.weight='+literal(edge['weight'])+';g.edges.push_back(e);}']
    lines.append('return g;}')
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    build = output.parent/(output.stem+'_build')
    build.mkdir(exist_ok=True)
    header = build/'smb_background_diagnostic_parameters.hpp'
    header.write_text('\n'.join(lines)+'\n')
    inputs = {'snapshot': str(args.snapshot), 'run': str(args.run), 'map_sha256': metadata['sha256'],
              'graph_sha256': snapshot['global_graph']['sha256'], 'history_sha256': history_metadata['sha256'],
              'effective_parameters': params, 'bound_mode': mode,
              'snapshot_atomic': snapshot['atomic_snapshot'], 'snapshot_association': snapshot['association'],
              'scope': __doc__, 'private_graph_types_available': False,
              'probe_origin_vertex_ids': center_ids, 'probe_origins': centers,
              'cluster_scope': 'Explicit graph vertex origins; no claim these are native unvisited centroids',
              'visited_scope': 'Complete received algorithm state_history; non-atomic association retained',
              'visited_states': visits, 'state_history_metadata': history_metadata,
              'random_stream': 'Fresh diagnostic seed, not the historical persistent background RNG state',
              'probe_seed': args.seed, 'grid_scan_performed': False,
              'adaptive_scope': 'Global background uses fixed configured geometry; per-call local PCA does not replace it',
              'expansion_scope': 'At most max_expansions independent graph copies, shared expandGround, no gain evaluation',
              'random_draws': args.random_draws, 'max_expansions': args.max_expansions}
    if args.prepare_only:
        output.write_text(json.dumps({'status': 'prepared_not_executed', 'input': inputs, 'generated_header': str(header)}, indent=2)+'\n')
        print(json.dumps({'status': 'prepared_not_executed', 'output': str(output)}))
        return
    configured = root/'build_fortress/gbplanner_local'
    target = configured/'CMakeFiles/gain_raycast_demo.dir'
    flags = dict(line.split(' = ', 1) for line in (target/'flags.make').read_text().splitlines() if ' = ' in line)
    command = shlex.split((target/'link.txt').read_text())
    command[next(i for i, value in enumerate(command) if value.endswith('.cpp.o'))] = str(root/'tools/diagnose_smb_background.cpp')
    executable = build/'diagnose_smb_background'
    command[command.index('-o')+1] = str(executable)
    command[1:1] = [word for key in ('CXX_DEFINES', 'CXX_INCLUDES', 'CXX_FLAGS') for word in shlex.split(flags[key])]+['-I'+str(build)]
    subprocess.run(command, cwd=configured, check=True)
    result = subprocess.run([str(executable), str(map_file.resolve())], capture_output=True, text=True, timeout=120)
    require(result.returncode == 0, result.stderr.strip() or 'Frozen-map diagnostic failed')
    report = json.loads(result.stdout)
    report.update(input=inputs, stderr=result.stderr)
    output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key: value for key, value in report.items() if key not in ('input', 'stderr')}, indent=2))


if __name__ == '__main__':
    main()
