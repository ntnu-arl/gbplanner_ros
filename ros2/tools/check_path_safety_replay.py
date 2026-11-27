#!/usr/bin/env python3
"""Replay run10 routes with the real PathRefiner on saved failure map350.

The third route was planned on map246. Only the later execution-validation
map350 was archived; this experiment cannot reconstruct planning-time results.
No ROS nodes, vehicle simulation, new random path, or parameter sweep are used.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shlex
import subprocess

import yaml


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=workspace / 'results/smb_continuous/run10')
    parser.add_argument('--config', type=Path,
                        default=workspace / 'src/gbplanner_smb/config/exploration.yaml')
    parser.add_argument('--output', type=Path, default=workspace / 'results/path_safety')
    parser.add_argument('--executable', type=Path)
    parser.add_argument('--build', action='store_true',
                        help='Build the standalone executable using the existing gbplanner_local CMake includes')
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    source, output, config = args.source.resolve(), args.output.resolve(), args.config.resolve()
    output.mkdir(parents=True, exist_ok=True)
    files = {name: source.with_name(source.name + suffix) for name, suffix in {
        'history': '_history.json', 'run': '.json', 'map_metadata': '_map.json', 'map_binary': '_map.bin',
        'run_configuration': '_auto.yaml',
    }.items()}
    history, run, metadata = (json.loads(files[name].read_text())
                              for name in ('history', 'run', 'map_metadata'))
    require(metadata['requested_map_revision'] == 350 and metadata['exact_requested_revision']
            and metadata['source'] == 'execution_validation', 'Expected exact failure map350')
    require(metadata['width'] == 93330 and metadata['height'] == 1
            and metadata['point_step'] == 24 and metadata['row_step'] == 93330 * 24
            and not metadata['is_bigendian']
            and files['map_binary'].stat().st_size == metadata['data_bytes'] == 93330 * 24,
            'Unexpected saved run10 voxel layout')
    expected_fields = {'x': (0, 7), 'y': (4, 7), 'z': (8, 7), 'occupancy': (12, 1),
                       'tsdf_distance': (16, 7), 'tsdf_weight': (20, 7)}
    require({f['name']: (f['offset'], f['datatype']) for f in metadata['fields']} == expected_fields,
            'Saved map fields differ from the replay decoder')
    commands = [event for event in history['commanded_paths'] if event['path']]
    require(len(commands) == 3 and len(history['local_paths']) == 3
            and len(history['planning_metrics']) == 3, 'Expected three recorded local routes')
    commanded, planner, metric = commands[2], history['local_paths'][2], history['planning_metrics'][2]
    failure = run['execution_validation_failure']
    require(metric['map_revision'] == 246 and metric['success'] and metric['bound_mode'] == 3
            and metric['refinement']['reason'] == 'disabled'
            and metric['tsdf_projection_mode'] == 'zero_crossing', 'Unexpected third-plan metadata')
    require(commanded['path'][1:] == planner['path'], 'Command does not contain the recorded planner route')
    require(failure['map_revision'] == 350 and failure['first_pose_is_observation']
            and failure['reason'] == 'path_not_free' and not failure['path_truncated'],
            'Unexpected native execution-validation failure')
    for route in (commanded['path'], planner['path'], failure['path']):
        require(len(route) >= 2 and all(len(pose) == 4 and all(math.isfinite(x) for x in pose)
                                      for pose in route), 'Nonfinite or malformed recorded route')
    configured = yaml.safe_load(config.read_text())['gbplanner_local']['ros__parameters']
    require(configured['planning.path_safety_enhance_enable']
            and configured['robot.safety_extension'] == [2., 3., .2]
            and configured['planning.path_interpolation_distance'] == .5
            and configured['planning.relaxed_corridor_multiplier'] == 1.5,
            'Expected the explicit upstream SMB corridor settings')
    # Keep the archived run's ground-height/projection settings, including its
    # explicit zero-crossing override, and change only the four corridor items.
    replay_configuration = yaml.safe_load(files['run_configuration'].read_text())
    parameters = replay_configuration['gbplanner_local']['ros__parameters']
    refinement_keys = ('planning.path_safety_enhance_enable', 'robot.safety_extension',
                       'planning.path_interpolation_distance', 'planning.relaxed_corridor_multiplier')
    for name in refinement_keys:
        parameters[name] = configured[name]
    require(parameters['robot.type'] == 'kGround' and parameters['map.resolution'] == .1
            and parameters['ground.interpolate_projection_distance'], 'Unexpected recorded ground configuration')
    require(parameters['robot.size'] == [1.17, 1.17, .25]
            and parameters['robot.center_offset'] == [0., 0., .015],
            'Replay must retain run10 real robot bounding box')
    projection = parameters.get('ground.tsdf_projection_mode', 'upstream_step')
    require(projection == 'zero_crossing', 'Replay config must retain run10 zero-crossing support')
    fixture = output / 'run10_input.txt'
    lines = ['map350 93330']

    def values(name, data):
        lines.append(name + ' ' + ' '.join(repr(x) for x in data))

    values('robot_size', parameters['robot.size'])
    values('center_offset', parameters['robot.center_offset'])
    values('ground', [parameters[name] for name in (
        'ground.robot_height', 'ground.max_ground_height', 'ground.max_inclination',
        'ground.max_negative_inclination', 'ground.projection_max_length')])
    values('global_min', parameters['planning.global_min'])
    values('global_max', parameters['planning.global_max'])
    values('refinement', [*parameters['robot.safety_extension'],
                          parameters['planning.path_interpolation_distance'],
                          parameters['planning.relaxed_corridor_multiplier']])
    for name, route in [('commanded', commanded['path']), ('planner', planner['path']),
                        ('failure_request', failure['path'])]:
        lines.append(f'{name} {len(route)}')
        lines.extend(' '.join(repr(x) for x in pose) for pose in route)
    fixture.write_text('\n'.join(lines) + '\n')
    config_copy = output / 'exploration.yaml'
    config_copy.write_bytes(config.read_bytes())
    replay_config_copy = output / 'replay.yaml'
    replay_config_copy.write_text(yaml.safe_dump(replay_configuration, sort_keys=False))
    upstream = workspace.parent / 'gbplanner/config/smb/gbplanner_config.yaml'
    provenance = {
        'scope': 'Offline production PathRefiner on recorded run10 routes and exact failure map350',
        'source_files': {name: {'path': str(path), 'sha256': digest(path)} for name, path in files.items()},
        'implementation_files': {str(path.relative_to(workspace)): digest(path) for path in [
            workspace / 'tools/path_safety_replay.cpp', workspace / 'tools/check_path_safety_replay.py',
            workspace / 'src/gbplanner_local/src/path_refinement.cpp',
            workspace / 'src/gbplanner_local/src/ground_terrain.cpp',
            workspace / 'src/gbplanner_local/src/sparse_voxel_map.cpp']},
        'native_library': {'path': str(workspace / 'install/gbplanner_local/lib/libgbplanner_local.so'),
                           'sha256': digest(workspace / 'install/gbplanner_local/lib/libgbplanner_local.so')},
        'configuration': {'path': str(config), 'sha256': digest(config),
                          'snapshot': str(config_copy), 'parameters': configured},
        'replay_configuration': {'path': str(replay_config_copy), 'sha256': digest(replay_config_copy),
                                 'base': str(files['run_configuration']),
                                 'changed_keys': list(refinement_keys), 'parameters': parameters},
        'upstream_configuration': {'path': str(upstream), 'sha256': digest(upstream),
                                   'fields': ['RobotParams.safety_extension=[2,3,0.2]',
                                              'PlanningParams.path_safety_enhance_enable=true',
                                              'PlanningParams.path_interpolation_distance=0.5',
                                              'PlanningParams.relaxed_corridor_multiplier=1.5']},
        'fixture': {'path': str(fixture), 'sha256': digest(fixture)},
        'map_revision': 350, 'map_source': metadata['source'], 'map_stamp': metadata['stamp'],
        'original_planning_map_revision': 246, 'original_plan_time_s': metric['time_s'],
        'command_time_s': commanded['time_s'],
        'height_conversion': {'input': 'saved base_link height', 'native': 'planning height',
                              'added_z_m': parameters['ground.max_ground_height'] - parameters['ground.robot_height']},
        'settings': {'relaxed': False, 'preserve_endpoint': False,
                     'robot_size_m': parameters['robot.size'], 'bound_mode': 'Exact (3)'},
        'limits': [
            'Map350 is the failure-time execution-validation map, not original planning map246.',
            'No claim is made that these outputs would have been produced on map246 or driven successfully.',
            'The commanded route includes one prepended observed pose; planner_route excludes that prefix.',
            'Occupied-voxel XY clearance is the exact minimum over the linear returned route with Z overlap; '
            'it excludes unknown cells, planning-bound distance, and any tracking-error guarantee.',
            'The corridor extraction extension is not a guaranteed robot clearance margin.',
        ],
        'processes_started': 'One offline native executable; no ROS or physics process',
    }
    (output / 'input.json').write_text(json.dumps(provenance, ensure_ascii=False, indent=2) + '\n')
    if args.prepare_only:
        print(json.dumps({'status': 'prepared', 'fixture': str(fixture)}, ensure_ascii=False))
        return
    require(args.executable is not None, 'Provide --executable or use --prepare-only')
    executable = args.executable.resolve()
    if args.build:
        flags_path = workspace / 'build/gbplanner_local/CMakeFiles/output_interpolation_demo.dir/flags.make'
        flags = dict(line.split(' = ', 1) for line in flags_path.read_text().splitlines() if ' = ' in line)
        build_command = ['c++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Wpedantic',
                         *shlex.split(flags['CXX_DEFINES']), *shlex.split(flags['CXX_INCLUDES']),
                         str(workspace / 'tools/path_safety_replay.cpp'),
                         '-L' + str(workspace / 'install/gbplanner_local/lib'),
                         '-lgbplanner_local', '-o', str(executable)]
        build = subprocess.run(build_command, capture_output=True, text=True, timeout=60)
        (output / 'build.log').write_text(build.stdout + build.stderr)
        (output / 'build.json').write_text(json.dumps({'command': build_command,
                                                       'returncode': build.returncode}, indent=2) + '\n')
        require(build.returncode == 0, f'Native build failed; see {output}/build.log')
    command = [str(executable), str(files['map_binary']), str(fixture)]
    process = subprocess.run(command, capture_output=True, text=True, timeout=60)
    (output / 'native_stdout.log').write_text(process.stdout)
    (output / 'native_stderr.log').write_text(process.stderr)
    result = json.loads(process.stdout) if process.returncode == 0 else {'status': 'native_error'}
    summary = {**provenance, 'command': command, 'executable_sha256': digest(executable),
               'returncode': process.returncode, 'native': result}
    (output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    require(process.returncode == 0 and result.get('status') == 'completed',
            f'Native replay failed; see {output}/native_stderr.log')
    print(json.dumps({'status': 'completed', 'map_revision': 350,
                      'original_planning_map_revision': 246, 'cases': [
                          {key: case[key] for key in ('name', 'success', 'modified', 'reason',
                                                      'removed_vertices', 'adjusted_vertices',
                                                      'endpoint_xy_displacement_m')}
                          for case in result['cases']]}, ensure_ascii=False))


if __name__ == '__main__':
    main()
