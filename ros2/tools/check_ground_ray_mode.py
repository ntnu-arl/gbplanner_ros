#!/usr/bin/env python3
"""Check a selected ground projection mode with one native plan on saved SMB map 91.

Only local_planner_node is launched. A rejected native plan is recorded as
planning_success=false independently of the ROS interface check.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
                        help='Result JSON; default keeps separate files for ray and selected TSDF modes')
    parser.add_argument('--wall-budget', type=float, default=35.)
    parser.add_argument('--tsdf-mode', choices=('upstream_step', 'zero_crossing', 'zero_crossing_simplex'),
                        help='Enable TSDF projection with this mode; omitted keeps the upstream_sample ray check')
    args = parser.parse_args()
    filename = 'run5.json' if args.tsdf_mode is None else 'run5_'+args.tsdf_mode+'.json'
    output = (args.output or workspace/'results/ground_ray_mode'/filename).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    source = workspace/'results/smb_continuous/run5'
    config = workspace/'src/gbplanner_smb/config/exploration.yaml'
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    started, initialized = time.monotonic(), False
    process = node = None
    summary = {'interface_status': 'failed', 'planning_success': None,
               'scope': __doc__, 'domain': os.environ['ROS_DOMAIN_ID'],
               'motion_nodes_started': False, 'planner_requests': 0, 'checks': {}}
    try:
        require(math.isfinite(args.wall_budget) and args.wall_budget>0., 'Invalid wall budget')
        import yaml
        metadata = json.loads(source.with_name(source.name+'_map.json').read_text())
        history = json.loads(source.with_name(source.name+'_history.json').read_text())
        map_bytes = source.with_name(source.name+'_map.bin').read_bytes()
        require(len(map_bytes)==metadata['data_bytes']==metadata['height']*metadata['row_step'],
                'Saved map metadata and bytes disagree')
        saved = next(event for event in history['map_updates']
                     if event['stamp_sec']==metadata['stamp']['sec']
                     and event['stamp_nanosec']==metadata['stamp']['nanosec'])
        require(saved['map_revision']==91, 'This fixture requires saved SMB run5 map revision 91')
        stamp = metadata['stamp']['sec']+metadata['stamp']['nanosec']*1.e-9
        with source.with_suffix('.csv').open(newline='') as handle:
            sample = min(csv.DictReader(handle), key=lambda row:abs(float(row['odom_stamp_s'])-stamp))
        current = [float(sample[key]) for key in ('x_m', 'y_m', 'z_m', 'yaw_rad')]
        require(all(math.isfinite(value) for value in current), 'Invalid recorded measured pose')
        config_bytes = config.read_bytes()
        configured = yaml.safe_load(config_bytes)['gbplanner_local']['ros__parameters']
        expected_root = current.copy()
        expected_root[2] += configured['ground.max_ground_height']-configured['ground.robot_height']
        summary['recorded_inputs'] = {
            'source': str(source), 'saved_map_revision': saved['map_revision'],
            'map_stamp': metadata['stamp'], 'map_cells': metadata['width']*metadata['height'],
            'map_sha256': hashlib.sha256(map_bytes).hexdigest(), 'observed_base_pose': current,
            'odometry_time_s': float(sample['time_s']), 'odometry_stamp_s': float(sample['odom_stamp_s']),
            'odometry_map_stamp_difference_s': float(sample['odom_stamp_s'])-stamp,
            'config': str(config), 'config_sha256': hashlib.sha256(config_bytes).hexdigest(),
            'note': 'Current SMB configuration and a fresh local graph; no historical graph or direction history restored.'}
        source_files = [workspace/'src/gbplanner_local/src/local_planner_node.cpp',
                        workspace/'src/gbplanner_local/src/ground_terrain.cpp',
                        workspace/'src/gbplanner_local/include/gbplanner_local/ground_terrain.hpp',
                        workspace.parent/'gbplanner/src/rrg.cpp',
                        workspace.parent/'planner_common/src/voxblox/voxblox_common_impl.cpp']
        summary['source_sha256'] = {str(path):hashlib.sha256(path.read_bytes()).hexdigest()
                                    for path in source_files}

        import rclpy
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import Odometry
        from planner_msgs.srv import PlannerSrv, ValidatePath
        from rcl_interfaces.srv import GetParameters
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        def pose(values):
            result = Pose()
            result.position.x, result.position.y, result.position.z = values[:3]
            result.orientation.z, result.orientation.w = math.sin(values[3]/2.), math.cos(values[3]/2.)
            return result

        def record(value):
            p, q = value.position, value.orientation
            return [p.x, p.y, p.z, q.x, q.y, q.z, q.w]

        rclpy.init(args=[])
        initialized = True
        node = Node('ground_ray_mode_probe')
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        maps = node.create_publisher(PointCloud2, 'voxel_map', durable)
        odometry = node.create_publisher(Odometry, 'odometry', 10)
        accepted, metrics = [], []
        node.create_subscription(PointCloud2, 'accepted_voxel_map', accepted.append, durable)
        node.create_subscription(String, 'planning_metrics',
                                 lambda message:metrics.append(json.loads(message.data)), durable)
        planner = node.create_client(PlannerSrv, 'planner')
        validator = node.create_client(ValidatePath, 'validate_path')
        map_status = node.create_client(Trigger, 'map_status')
        sensor_gain = node.create_client(Trigger, 'sensor_gain')
        parameters = node.create_client(GetParameters, 'gbplanner_local/get_parameters')

        def publish_odometry():
            message = Odometry()
            message.header.frame_id, message.child_frame_id = metadata['frame_id'], 'base_link'
            message.header.stamp = node.get_clock().now().to_msg()
            message.pose.pose = pose(current)
            odometry.publish(message)

        node.create_timer(.03, publish_odometry)
        cloud = PointCloud2()
        cloud.header.frame_id = metadata['frame_id']
        cloud.header.stamp.sec = metadata['stamp']['sec']
        cloud.header.stamp.nanosec = metadata['stamp']['nanosec']
        for key in ('width', 'height', 'point_step', 'row_step', 'is_bigendian', 'is_dense'):
            setattr(cloud, key, metadata[key])
        cloud.fields = [PointField(**field) for field in metadata['fields']]
        cloud.data = map_bytes
        if args.tsdf_mode is None:
            overrides = {'global.enable': False, 'ground.interpolate_projection_distance': False,
                         'ground.ray_projection_mode': 'upstream_sample'}
            selected_mode = {'ground_projection': 'occupancy', 'ray_projection_mode': 'upstream_sample'}
        else:
            overrides = {'global.enable': False, 'ground.interpolate_projection_distance': True,
                         'ground.tsdf_projection_mode': args.tsdf_mode}
            selected_mode = {'ground_projection': 'tsdf', 'tsdf_projection_mode': args.tsdf_mode}
        summary['overrides'], summary['selected_mode'] = overrides, selected_mode
        launch = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
                  '--params-file', str(config)]
        for name, value in overrides.items():
            launch += ['-p', name+':='+json.dumps(value)]
        summary['launch'] = launch
        log_path = output.with_suffix('.log')
        with log_path.open('w') as log:
            process = subprocess.Popen(launch, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=dict(os.environ))
        deadline = time.monotonic()+args.wall_budget

        def until(predicate, label):
            while not predicate():
                if process.poll() is not None:
                    raise RuntimeError(f'Planner exit {process.returncode}; see {log_path}')
                if time.monotonic()>=deadline:
                    raise TimeoutError(label)
                rclpy.spin_once(node, timeout_sec=.02)

        def call(client, request):
            until(client.service_is_ready, 'Service discovery: '+client.srv_name)
            future = client.call_async(request)
            until(future.done, 'Service response: '+client.srv_name)
            return future.result()

        until(lambda:maps.get_subscription_count()>0 and odometry.get_subscription_count()>0,
              'Map and odometry discovery')
        publish_odometry()
        maps.publish(cloud)
        until(lambda:bool(accepted), 'Saved map acceptance')
        require(bytes(accepted[-1].data)==map_bytes and accepted[-1].fields==cloud.fields,
                'Native accepted map differs from the complete saved snapshot')
        expected = {**overrides, **{key:configured[key] for key in
                    ('sampling.seed', 'map.storage', 'map.resolution', 'robot.type',
                     'ground.robot_height', 'ground.max_ground_height')}}
        request = GetParameters.Request()
        request.names = list(expected)
        response = call(parameters, request)
        fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value'}
        require(len(response.values)==len(request.names), 'Incomplete GetParameters response')
        actual = {name:getattr(value, fields[value.type])
                  for name, value in zip(request.names, response.values)}
        summary['parameters'] = actual
        require(actual==expected, 'Native parameters do not match SMB configuration and the three overrides')
        status = call(map_status, Trigger.Request())
        summary['map_status'] = json.loads(status.message)
        state = summary['map_status']
        require(status.success and state['rejected_maps']==0 and state['known_cells']==saved['known_cells']
                and state['tsdf_fields'] and state['tsdf_cells']>0 and state['storage']=='sparse'
                and all(state.get(key)==value for key, value in selected_mode.items()),
                'Native map status does not confirm the selected projection mode and retained TSDF input')

        def root_ready():
            result = call(sensor_gain, Trigger.Request())
            data = json.loads(result.message)
            summary['sensor_gain'] = {'success': result.success, 'data': data}
            return result.success and len(data.get('root', []))==4 and all(
                abs(a-b)<1.e-9 for a, b in zip(data['root'], expected_root))

        until(root_ready, 'Native planner accepted the recorded measured pose')
        request = PlannerSrv.Request()
        request.header.frame_id, request.bound_mode = metadata['frame_id'], 3
        request.root_pose.orientation.w = 0.  # All-zero sentinel uses the accepted actual odometry.
        summary['planner_requests'] += 1
        response = call(planner, request)
        until(lambda:bool(metrics), 'Metrics for the single native planning request')
        native = metrics[-1]
        summary['planning_metrics'] = native
        summary['planning_success'] = native['success']
        summary['planner_response'] = {'status': response.status, 'bound_mode': response.planning_bound_mode,
                                       'path': [record(value) for value in response.path]}
        request = ValidatePath.Request()
        request.header.frame_id, request.bound_mode = metadata['frame_id'], response.planning_bound_mode
        request.path = response.path  # Also submit an empty native result without manufacturing waypoints.
        validation = call(validator, request)
        summary['validation'] = {'valid': validation.valid, 'reason': validation.reason,
                                 'map_revision': validation.map_revision, 'path_points': len(request.path),
                                 'first_pose_is_observation': request.first_pose_is_observation}
        expected_failures = {'root_not_free', 'no_feasible_path', 'best_path_too_short',
                             'no_ground_attached_path', 'no_gain'}
        summary['checks'] = {
            'selected_parameters_confirmed': actual==expected,
            'one_native_planning_request': summary['planner_requests']==1 and len(metrics)==1,
            'native_mode_confirmed': all(native.get(key)==value for key, value in selected_mode.items()),
            'configured_seed_preserved': native['seed']==configured['sampling.seed'],
            'actual_odometry_root_used': all(abs(a-b)<1.e-9
                                             for a, b in zip(native['planning_root'], expected_root)),
            'native_response_consistent': (bool(response.path)==native['success']
                                           and response.status==(0 if native['success'] else -1)
                                           and response.planning_bound_mode==3),
            'native_reason_recognized': native['reason']=='ok' if native['success']
                                       else native['reason'] in expected_failures,
            'same_map_used': native['map_revision']==validation.map_revision==state['map_revision'],
            'validation_response_consistent': (validation.reason=='ok' if validation.valid
                                              else bool(validation.reason) and validation.reason!='ok'),
            'empty_result_not_replaced': bool(response.path) or
                                        (not validation.valid and validation.reason=='invalid_path_size'),
        }
        require(all(summary['checks'].values()), 'Native interface evidence is inconsistent; see checks and responses')
        summary['interface_status'] = 'passed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
    finally:
        if process is not None:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=1.)
            summary['planner_exit'] = process.returncode
            if process.returncode!=0:
                summary['interface_status'] = 'failed'
                summary['cleanup_error'] = 'Native planner did not exit cleanly'
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        summary['elapsed_s'] = round(time.monotonic()-started, 3)
        output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(summary, ensure_ascii=False))
    return 0 if summary['interface_status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())
