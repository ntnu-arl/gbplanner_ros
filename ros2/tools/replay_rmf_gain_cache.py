#!/usr/bin/env python3
"""Profile a frozen RMF map, or compare exact gain reuse on the same input.

--ahead uses the recorded Action endpoint for one cached core probe. Explicit
seed/direction values are required because old runs did not save those inputs.
Sampling-budget overrides apply equally to both sides of a cache comparison;
two invocations with the same map, endpoint, seed and direction can compare
raw-draw and upstream sampling-round budgets as a new frozen-map experiment.
--prepare-only writes inputs and generated parameters without compiling or
starting ROS. Run the recorded invocation in the existing ROS build environment.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shlex
import subprocess
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--build-dir', type=Path)
    parser.add_argument('--ahead', action='store_true')
    parser.add_argument('--compare-ray-invariants', action='store_true',
                        help='Pair the original and invariant-reuse loops; both use exact pose-gain reuse')
    parser.add_argument('--pair-order', choices=('baseline-first', 'optimized-first'), default='baseline-first')
    parser.add_argument('--ahead-index', type=int, default=0)
    parser.add_argument('--seed', type=int)
    parser.add_argument('--direction', type=float)
    parser.add_argument('--sampling-budget-mode', choices=('raw_draws', 'sampling_rounds'))
    parser.add_argument('--max-attempts', type=int, help='Raw-draw limit, active only in raw_draws mode')
    parser.add_argument('--max-sampling-rounds', type=int,
                        help='sampleVertex call limit, active only in sampling_rounds mode')
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    if args.ahead and (args.seed is None or args.direction is None):
        parser.error('--ahead requires explicit --seed and --direction; these historical values are not inferred')
    if args.compare_ray_invariants and not args.ahead:
        parser.error('--compare-ray-invariants requires --ahead')
    if args.pair_order != 'baseline-first' and not args.compare_ray_invariants:
        parser.error('--pair-order requires --compare-ray-invariants')
    if args.seed is not None and not 0 <= args.seed < 2**32:
        parser.error('--seed must fit uint32')
    if args.direction is not None and not math.isfinite(args.direction):
        parser.error('--direction must be finite')
    if args.ahead_index < 0:
        parser.error('--ahead-index must be nonnegative')
    for flag, value in (('--max-attempts', args.max_attempts),
                        ('--max-sampling-rounds', args.max_sampling_rounds)):
        if value is not None and not 1 <= value <= 1000000:
            parser.error(f'{flag} must be in [1, 1000000]')
    root = Path(__file__).resolve().parents[1]
    config = args.config or root/'src/gbplanner_gazebo/config/rmf_exploration.yaml'
    params = yaml.safe_load(config.read_text())['gbplanner_local']['ros__parameters']
    # Old configurations deliberately retain their raw-draw behavior. Record
    # defaults and overrides so the generated header cannot silently disagree
    # with the budget described by the experiment's input metadata.
    params.setdefault('planning.sampling_budget_mode', 'raw_draws')
    params.setdefault('planning.max_attempts', 4000)
    params.setdefault('planning.max_sampling_rounds', 10000)
    budget_overrides = {}
    for key, value in (('planning.sampling_budget_mode', args.sampling_budget_mode),
                       ('planning.max_attempts', args.max_attempts),
                       ('planning.max_sampling_rounds', args.max_sampling_rounds)):
        if value is not None:
            budget_overrides[key] = value
            params[key] = value
    mode = params['planning.sampling_budget_mode']
    if mode not in ('raw_draws', 'sampling_rounds'):
        parser.error('Configured planning.sampling_budget_mode must be raw_draws or sampling_rounds')
    for key in ('planning.max_attempts', 'planning.max_sampling_rounds'):
        if type(params[key]) is not int or not 1 <= params[key] <= 1000000:
            parser.error(f'Configured {key} must be an integer in [1, 1000000]')
    source = json.loads(args.result.read_text())
    history = json.loads(args.result.with_name(args.result.stem+'_history.json').read_text())
    bound_mode = 0
    evidence = None
    if args.ahead:
        requests = [row for row in history['control'] if row.get('reason') == 'plan_ahead_requested']
        if args.ahead_index >= len(requests):
            parser.error('Requested ahead attempt is absent from the saved history')
        request = requests[args.ahead_index]
        routes = [row for row in history['execution'] if row['wall_s'] <= request['wall_s']
                  and row.get('state') == 'EXECUTING' and row.get('endpoint')]
        if not routes:
            parser.error('No recorded active Action endpoint before the selected ahead request')
        route = routes[-1]
        viewpoint = route['endpoint']
        bound_mode = request['bound_mode']
        evidence = {'request_wall_s': request['wall_s'], 'request_ros_s': request['sim_s'],
                    'observation_revision': request['observation_revision'],
                    'route_wall_s': route['wall_s'], 'route_generation': route['generation'],
                    'endpoint_source': 'execution.endpoint', 'bound_mode': bound_mode}
    else:
        home = source['final']['task']['home_position']
        event = next(e for e in source['final']['task']['events'] if e['state'] == 'START_EXPLORATION')
        odom = min(history['odometry'], key=lambda r: abs(r['stamp_ns']*1.e-9-event['ros_s']))
        x, y, z, w = odom['quaternion']
        viewpoint = [*home, math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]
    if len(viewpoint) != 4 or not all(math.isfinite(v) for v in viewpoint):
        raise ValueError('Invalid recorded viewpoint')
    seed = params['sampling.seed'] if args.seed is None else args.seed
    direction = 0. if args.direction is None else args.direction
    metadata = source['accepted_map']
    expected_fields = [('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('occupancy', 12, 1),
                       ('tsdf_distance', 16, 7), ('tsdf_weight', 20, 7)]
    layout = [(field['name'], field['offset'], field['datatype']) for field in metadata['fields']]
    if metadata['point_step'] != 24 or metadata['is_bigendian'] or layout != expected_fields:
        raise ValueError('Unsupported recorded map layout')
    map_file = args.result.with_name(Path(metadata['data_file']).name).resolve()
    if map_file.stat().st_size != metadata['width'] * metadata['height'] * 24:
        raise ValueError('Recorded map dimensions and byte length differ')
    if params['robot.type'] != 'kAerial' or params.get('planning.adaptive_exploration', False):
        raise ValueError('This RMF probe supports aerial fixed sampling geometry')
    if params['sampling.space_type'] != 'kCuboid' or params['planning.global_type'] != 'kCuboid':
        raise ValueError('This RMF probe supports cuboid bounds')
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    build = output.parent/(output.stem+'_build')
    build.mkdir(exist_ok=True)
    def literal(value):
        if isinstance(value, list):
            return '{'+','.join(literal(v) for v in value)+'}'
        return json.dumps(value, allow_nan=False)
    fields = {
        'local_min':'sampling.min', 'local_max':'sampling.max', 'local_rotations':'sampling.rotations',
        'gain_min_extension':'sampling.min_extension', 'gain_max_extension':'sampling.max_extension',
        'edge_min':'planning.edge_min', 'edge_max':'planning.edge_max', 'overshoot':'planning.overshoot',
        'max_velocity':'planning.v_max', 'traverse_length_max':'planning.traverse_length_max',
        'traverse_time_max':'planning.traverse_time_max', 'neighbor_radius':'planning.neighbor_radius',
        'neighbor_min':'planning.neighbor_min','neighbor_max':'planning.neighbor_max',
        'max_vertices':'planning.max_vertices','max_edges':'planning.max_edges','max_attempts':'planning.max_attempts',
        'sampling_budget_mode':'planning.sampling_budget_mode','max_sampling_rounds':'planning.max_sampling_rounds',
        'empty_graph_cutoff':'planning.empty_graph_cutoff','graph_mode':'planning.graph_mode',
        'tangent_yaw':'planning.tangent_yaw','leaf_gain_only':'planning.leaf_gain_only',
        'cluster_vertices_for_gain':'planning.cluster_vertices_for_gain','clustering_radius':'planning.clustering_radius',
        'seed':'sampling.seed','distribution':'sampling.distribution','center_offset':'robot.center_offset',
        'unknown_gain':'gain.unknown','free_gain':'gain.free','occupied_gain':'gain.occupied',
        'path_length_penalty':'planning.path_length_penalty','path_direction_penalty':'planning.path_direction_penalty',
        'gain.use_ray_model':'planning.use_ray_model_for_volumetric_gain',
        'gain.ray_method':'planning.ray_cast_method','gain.nonuniform_ray_cast':'planning.nonuniform_ray_cast',
        'gain.ray_cast_step_size_multiplier':'planning.ray_cast_step_size_multiplier',
        'gain.exp_gain_voxel_size':'planning.exp_gain_voxel_size',
        'refinement.enabled':'planning.path_safety_enhance_enable',
        'refinement.safety_extension':'robot.safety_extension',
        'refinement.interpolation_distance':'planning.path_interpolation_distance'}
    lines = ['inline gbplanner2::Parameters replayParameters(){gbplanner2::Parameters p;']
    for field, key in fields.items():
        lines.append(f'p.{field}={literal(params[key])};')
    lines.append('p.gain.reuse_ray_invariants='+literal(params.get('planning.reuse_ray_invariants', False))+';')
    # Use the same policy as the service, including budget scaling for relaxed
    # and minimum bounds; an endpoint probe must not silently become Extended.
    lines.extend(['gbplanner2::RobotBounds body;',
                  'body.physical_size='+literal(params['robot.size'])+';',
                  'body.extension='+literal(params['robot.size_extension'])+';',
                  'body.min_extension='+literal(params['robot.size_extension_min'])+';',
                  'body.relax_ratio='+literal(params['robot.relax_ratio'])+';',
                  f'p.robot_size=body.size({bound_mode});',
                  f'p.point_robot=body.pointMode({bound_mode});',
                  f'p.max_vertices=body.scaledBudget(p.max_vertices,{bound_mode});',
                  f'p.max_edges=body.scaledBudget(p.max_edges,{bound_mode});',
                  f'p.seed={seed};'])
    for field in ('distribution', 'min', 'max'):
        member = {'min': 'minimum', 'max': 'maximum'}.get(field, field)
        lines.append(f'p.axis_sampling[3].{member}='+literal(params[f'sampling.heading.{field}'])+';')
    for name in params['planning.exploration_sensors']:
        lines.append('{gbplanner2::SensorParameters s;s.name='+literal(name)+';')
        for field in ('type','range','min_range','center_offset','rotations','horizontal_fov','vertical_fov',
                      'horizontal_resolution','vertical_resolution','frontier_threshold'):
            lines.append(f's.{field}={literal(params[f"sensor.{name}.{field}"])};')
        lines.append('p.sensors.push_back(s);}')
    lines.extend(['p.validate();return p;}',
                  'inline StateVec replayRoot(){return '+literal(viewpoint)+';}',
                  'inline double replayDirection(){return '+literal(direction)+';}',
                  'inline bool replayAheadMode(){return '+literal(args.ahead)+';}',
                  'inline bool replayCompareRayInvariants(){return '+literal(args.compare_ray_invariants)+';}',
                  'inline bool replayOptimizedFirst(){return '+literal(args.pair_order == 'optimized-first')+';}',
                  'inline double replayResolution(){return '+literal(params['map.resolution'])+';}',
                  'inline gbplanner2::GlobalBound replayBound(){return gbplanner2::GlobalBound::axisAligned('
                  +literal(params['planning.global_min'])+','+literal(params['planning.global_max'])+');}'])
    (build/'rmf_gain_replay_parameters.hpp').write_text('\n'.join(lines)+'\n')
    invocation = ['python3', 'tools/replay_rmf_gain_cache.py', str(args.result), '--output', str(args.output)]
    if args.ahead:
        invocation += ['--ahead', '--ahead-index', str(args.ahead_index)]
    if args.compare_ray_invariants:
        invocation += ['--compare-ray-invariants', '--pair-order', args.pair_order]
    if args.seed is not None:
        invocation += ['--seed', str(seed)]
    if args.direction is not None:
        invocation += ['--direction', str(direction)]
    for flag, value in (('--sampling-budget-mode', args.sampling_budget_mode),
                        ('--max-attempts', args.max_attempts),
                        ('--max-sampling-rounds', args.max_sampling_rounds)):
        if value is not None:
            invocation += [flag, str(value)]
    if args.config:
        invocation += ['--config', str(args.config)]
    if args.build_dir:
        invocation += ['--build-dir', str(args.build_dir)]
    input_record = {
        'native_result': str(args.result), 'map_sha256': hashlib.sha256(map_file.read_bytes()).hexdigest(),
        'config_sha256': hashlib.sha256(config.read_bytes()).hexdigest(),
        'root': viewpoint, 'seed': seed, 'exploring_direction': direction, 'bound_mode': bound_mode,
        'parameters': params, 'ahead_evidence': evidence, 'budget_overrides': budget_overrides,
        'sampling_budget': {
            'mode': mode, 'max_attempts': params['planning.max_attempts'],
            'max_sampling_rounds': params['planning.max_sampling_rounds'],
            'active_limit': params['planning.max_attempts' if mode == 'raw_draws'
                                   else 'planning.max_sampling_rounds'],
            'raw_draws_per_sampling_round_limit': 1000},
        'seed_and_direction_provenance': {
            'seed': 'explicit probe value' if args.seed is not None else 'configured base seed',
            'direction': 'explicit probe value' if args.direction is not None else 'zero direction default',
            'historical_recovery': False},
        'global_state': 'fresh graph with synthetic home at probe root; historical global topology is unavailable',
        'map_scope': 'final accepted native map; a new frozen-map experiment, not the map at the recorded request time',
        'order': (args.pair_order+'; exact pose-gain cache on in both; one pair is not a statistical speedup estimate'
                  if args.compare_ray_invariants else 'one cached core request' if args.ahead else 'baseline once, then cached once'),
        'scope': 'LocalPlanner.plan + absorbLocal; no ROS service queue, mission budget, executor or controller timing'}
    if args.prepare_only:
        output.write_text(json.dumps({'status': 'prepared_not_executed', 'input': input_record,
                                     'invocation_from_ros2_root': invocation,
                                     'generated_header': str(build/'rmf_gain_replay_parameters.hpp'),
                                     'build_requirement': 'Rebuild gbplanner_local core after timing fields change; run inside its configured environment'},
                                    indent=2, allow_nan=False)+'\n')
        print(json.dumps({'status': 'prepared_not_executed', 'output': str(output), 'root': viewpoint,
                          'invocation_from_ros2_root': invocation}, indent=2))
        return
    # Reuse the configured compiler and dependency paths, without modifying CMake.
    configured = (args.build_dir or root/'build_fortress/gbplanner_local').resolve()
    target = configured/'CMakeFiles/gain_raycast_demo.dir'
    flags = {}
    for line in (target/'flags.make').read_text().splitlines():
        if ' = ' in line:
            key, value = line.split(' = ',1)
            flags[key] = shlex.split(value)
    command = shlex.split((target/'link.txt').read_text())
    object_index = next(i for i,s in enumerate(command) if s.endswith('.cpp.o'))
    command[object_index] = str(root/'tools/replay_rmf_gain_cache.cpp')
    executable = build/'replay_rmf_gain_cache'
    command[command.index('-o')+1] = str(executable)
    command[1:1] = flags['CXX_DEFINES']+flags['CXX_INCLUDES']+flags['CXX_FLAGS']+['-I'+str(build)]
    subprocess.run(command,cwd=configured,check=True)
    completed = subprocess.run([str(executable),str(map_file)],capture_output=True,text=True)
    if completed.returncode:
        raise RuntimeError('Offline comparison failed: '+completed.stderr.strip())
    result = json.loads(completed.stdout)
    result['input'] = input_record
    result['stderr'] = completed.stderr
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({key:result[key] for key in ('status','cells','baseline','cached','optimized') if key in result},indent=2))


if __name__ == '__main__':
    main()
