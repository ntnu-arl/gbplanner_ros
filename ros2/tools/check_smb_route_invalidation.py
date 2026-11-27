#!/usr/bin/env python3
"""Locate a recorded SMB automatic-route rejection with offline native validation.

The snapshot stamp is matched to the recorded map revision. The original
ValidatePath request was not logged: its route is reconstructed with execution's
prepare/PathProgress and the last recorded odometry before STOPPING. No mapper,
tracking node, simulator or physical motion is started by this diagnostic.
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
import sys
import time


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=workspace/'results/smb_continuous/run5')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    source = args.source.resolve()
    output = (args.output or source.with_name(source.name+'_invalidation.json')).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))
    started = time.monotonic()
    process = node = None
    initialized = False
    summary = {'status': 'failed', 'source': str(source), 'domain': os.environ['ROS_DOMAIN_ID'],
               'scope': 'offline native validator, saved map, reconstructed remaining route',
               'original_validation_request_recorded': False}

    def require(condition, message):
        if not condition:
            raise AssertionError(message)

    try:
        import rclpy
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import Odometry
        from planner_msgs.srv import ValidatePath
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField
        from std_srvs.srv import Trigger
        import yaml
        sys.path[:0] = [str(workspace/'src/gbplanner_execution/scripts'),
                        str(workspace/'src/gbplanner_local/scripts')]
        from path_progress import PathProgress
        from trajectory_tools import prepare

        run = json.loads(source.with_suffix('.json').read_text())
        history = json.loads(source.with_name(source.name+'_history.json').read_text())
        metadata = json.loads(source.with_name(source.name+'_map.json').read_text())
        map_bytes = source.with_name(source.name+'_map.bin').read_bytes()
        require(len(map_bytes) == metadata['data_bytes'] == metadata['row_step']*metadata['height'],
                'Saved map binary size does not match metadata')
        stopped = next(event for event in history['execution']
                       if event.get('phase') == 'automatic_policy' and event['state'] == 'STOPPING'
                       and event['reason'].startswith('path_invalidated:'))
        action = next(action for action in reversed(run['checks']['automatic_policy']['actions'])
                      if action['commanded_path']['time_s'] <= stopped['time_s'])
        command = action['commanded_path']
        dispatch = next(event for event in history['execution']
                        if event['time_s'] >= command['time_s'] and event['state'] == 'VALIDATING')
        saved = next(event for event in history['map_updates']
                     if event['stamp_sec'] == metadata['stamp']['sec']
                     and event['stamp_nanosec'] == metadata['stamp']['nanosec'])
        tracking_config = workspace/'src/gbplanner_tracking/config/smb.yaml'
        follower = yaml.safe_load(tracking_config.read_text())['gbplanner_path_follower']['ros__parameters']
        saved_config = source.with_name(source.name+'_auto.yaml')
        planner_config = saved_config if saved_config.is_file() else workspace/'src/gbplanner_smb/config/exploration.yaml'
        planner_parameters = yaml.safe_load(planner_config.read_text())['gbplanner_local']['ros__parameters']
        # New PCI command_path is its preflight connector, not the action's
        # original planner response. Use the matching native route exactly
        # once, then prepend the follower's actual dispatch observation.
        native_routes = [event for event in history.get('local_paths', [])
                         if event.get('phase') == command['phase'] and event.get('path')
                         and event['time_s'] <= command['time_s']
                         and event['path'][-1] == command['path'][-1]]
        if native_routes:
            native = native_routes[-1]
            planned_path, route_source = native['path'], 'history.local_paths'
        else:
            require('validates_remaining_path' not in dispatch,
                    'New PCI recording has no matching original native route; refusing a duplicate preflight prefix')
            native = None
            planned_path, route_source = command['path'], 'legacy command_path'
        # SMB's nested tracking launch supplies this configuration. Ground
        # followers additionally force smooth_yaw=False in their constructor.
        smooth_yaw = follower.get('smooth_yaw', True) and follower.get('robot_type', 'kAerial') != 'kGround'
        connected, _ = prepare(planned_path, dispatch['observed_pose'],
                               dispatch['active_speed_mps'], follower['yaw_rate'],
                               follower['sample_period'], smooth_yaw)
        progress = PathProgress([point[:3] for point in connected])
        with source.with_suffix('.csv').open(newline='') as handle:
            samples = [row for row in csv.DictReader(handle)
                       if dispatch['time_s'] <= float(row['time_s']) <= stopped['time_s']]
        require(bool(samples), 'No recorded odometry between dispatch and cancellation')
        for row in samples:
            current = tuple(float(row[key]) for key in ('x_m', 'y_m', 'z_m'))
            progress.update(current)
        yaw = float(samples[-1]['yaw_rad'])
        remaining = [(*point, yaw) for point in progress.remaining(current)]
        delta = [b-a for a, b in zip(planned_path[0][:2], planned_path[1][:2])]
        displacement = [b-a for a, b in zip(planned_path[0][:2], current[:2])]
        length = math.hypot(*delta)
        same_revision = saved['map_revision'] == stopped['map_revision']
        request_limit = ('The failing request and its response revision were not recorded. '
                         f"Saved map revision {saved['map_revision']}; failure status revision {stopped['map_revision']}. "
                         + ('Their recorded map revisions match. ' if same_revision else
                            'This replay uses a later/different map and does not reproduce the exact failure snapshot. ')
                         + 'Route and pose are reconstructed from recorded native route, dispatch observation and odometry.')
        summary['recorded_inputs'] = {
            'command_time_s': command['time_s'], 'dispatch_time_s': dispatch['time_s'],
            'stop_time_s': stopped['time_s'],
            'failure_reason': stopped['reason'], 'failure_status_map_revision': stopped['map_revision'],
            'matched_recorded_revision': saved['map_revision'], 'matched_map_update_time_s': saved['time_s'],
            'saved_snapshot_matches_failure_status_revision': same_revision,
            'map_stamp': metadata['stamp'], 'map_sha256': hashlib.sha256(map_bytes).hexdigest(),
            'map_cells': metadata['width']*metadata['height'],
            'request_limit': request_limit,
            'dispatch_base_pose': dispatch['observed_pose'], 'observed_base_pose': [*current, yaw],
            'route_source': route_source, 'native_route_time_s': native['time_s'] if native else None,
            'native_planned_path': planned_path,
            'planner_configuration': {'source': str(planner_config),
                'sha256': hashlib.sha256(planner_config.read_bytes()).hexdigest(),
                'saved_automatic_configuration': planner_config == saved_config,
                'interpolate_projection_distance': planner_parameters.get('ground.interpolate_projection_distance', False),
                'tsdf_projection_mode': planner_parameters.get('ground.tsdf_projection_mode', 'upstream_step'),
                'ray_projection_mode': planner_parameters.get('ground.ray_projection_mode', 'voxel_surface')},
            'reconstruction_parameters': {'source': str(tracking_config),
                                          'robot_type': follower['robot_type'], 'smooth_yaw': smooth_yaw,
                                          'yaw_rate': follower['yaw_rate'],
                                          'sample_period': follower['sample_period']},
            'odometry_time_s': float(samples[-1]['time_s']), 'original_commanded_path': command['path'],
            'connected_path': connected, 'remaining_path': remaining, 'progress_m': progress.progress,
            'connected_length_m': progress.arc[-1],
            'planar_displacement_m': math.dist(dispatch['observed_pose'][:2], current[:2]),
            'first_segment_planar_length_m': length,
            'first_segment_planar_progress_m': sum(x*y for x, y in zip(delta, displacement))/length if length else None,
            'first_segment_cross_track_m': abs(delta[0]*displacement[1]-delta[1]*displacement[0])/length if length else None,
        }

        def pose(values):
            value = Pose()
            value.position.x, value.position.y, value.position.z = map(float, values[:3])
            angle = float(values[3]) if len(values) > 3 else 0.
            value.orientation.z, value.orientation.w = math.sin(angle/2.), math.cos(angle/2.)
            return value

        rclpy.init(args=[])
        initialized = True
        node = Node('smb_route_invalidation_replay')
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        maps = node.create_publisher(PointCloud2, 'voxel_map', durable)
        odometry = node.create_publisher(Odometry, 'odometry', 10)
        accepted = []
        node.create_subscription(PointCloud2, 'accepted_voxel_map', accepted.append, durable)
        validator = node.create_client(ValidatePath, 'validate_path')
        map_status = node.create_client(Trigger, 'map_status')

        def publish_odometry():
            message = Odometry()
            message.header.frame_id, message.child_frame_id = metadata['frame_id'], 'base_link'
            message.header.stamp = node.get_clock().now().to_msg()
            message.pose.pose = pose([*current, yaw])
            odometry.publish(message)

        node.create_timer(.03, publish_odometry)
        cloud = PointCloud2()
        cloud.header.frame_id = metadata['frame_id']
        cloud.header.stamp.sec, cloud.header.stamp.nanosec = metadata['stamp']['sec'], metadata['stamp']['nanosec']
        for key in ('width', 'height', 'point_step', 'row_step', 'is_bigendian', 'is_dense'):
            setattr(cloud, key, metadata[key])
        cloud.fields = [PointField(**field) for field in metadata['fields']]
        cloud.data = map_bytes
        launch = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
                  '--params-file', str(planner_config),
                  '-p', 'global.enable:=false', '-p', 'global.background_enable:=false']
        summary['launch'] = launch
        log_path = output.with_suffix('.log')
        with log_path.open('w') as log:
            process = subprocess.Popen(launch, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=dict(os.environ))

        def until(predicate, label):
            deadline = time.monotonic()+15.
            while not predicate():
                if process.poll() is not None:
                    raise RuntimeError(f'Planner exited with {process.returncode}; see {log_path}')
                if time.monotonic() > deadline:
                    raise TimeoutError(label)
                rclpy.spin_once(node, timeout_sec=.03)

        def call(client, request):
            until(client.service_is_ready, 'ROS service discovery')
            pending = client.call_async(request)
            until(pending.done, 'ROS service response')
            return pending.result()

        until(lambda: validator.service_is_ready() and maps.get_subscription_count() > 0
              and odometry.get_subscription_count() > 0, 'Planner/map/odometry discovery')
        publish_odometry()
        maps.publish(cloud)
        until(lambda: bool(accepted), 'Saved map acceptance')
        require(bytes(accepted[-1].data) == map_bytes and accepted[-1].fields == cloud.fields,
                'Native planner accepted a different map')
        reply = call(map_status, Trigger.Request())
        summary['replayed_map_status'] = json.loads(reply.message)
        require(reply.success and summary['replayed_map_status']['rejected_maps'] == 0, 'Map rejected')
        if 'ground.tsdf_projection_mode' in planner_parameters:
            require(summary['replayed_map_status'].get('tsdf_projection_mode') ==
                    planner_parameters['ground.tsdf_projection_mode'], 'Saved TSDF projection mode was not applied')
        if 'ground.ray_projection_mode' in planner_parameters:
            require(summary['replayed_map_status'].get('ray_projection_mode') ==
                    planner_parameters['ground.ray_projection_mode'], 'Saved ray projection mode was not applied')

        def validate(points, observation=False):
            request = ValidatePath.Request()
            request.header.frame_id = metadata['frame_id']
            request.bound_mode = int(dispatch['bound_mode'])
            request.path = [pose(point) for point in points]
            request.first_pose_is_observation = observation
            reply = call(validator, request)
            return {'valid': reply.valid, 'reason': reply.reason, 'replay_map_revision': reply.map_revision,
                    'first_pose_is_observation': observation, 'path': [list(point) for point in points]}

        def segments(points, observation=False):
            return [{'index': index, 'from': list(start), 'to': list(end),
                     **validate([start, end], observation and index == 0)}
                    for index, (start, end) in enumerate(zip(points, points[1:]))]

        summary['responses'] = {
            'original_route': validate(planned_path),
            'reconstructed_remaining_route': validate(remaining, True),
            'original_segments': segments(planned_path),
            'remaining_segments': segments(remaining, True),
        }
        # The guard is an actual historical observation only while the cursor
        # retains connected[0]. No arbitrary future/reference vertex inherits
        # observation semantics. Split requests cover every original segment.
        retained_observation = (len(remaining) >= 3 and
            tuple(remaining[1][:3]) == tuple(connected[0][:3]) and
            tuple(connected[0][:3]) == tuple(dispatch['observed_pose'][:3]))
        summary['retained_observation_guard'] = retained_observation
        if retained_observation:
            live_to_guard, guard_to_future = remaining[:2], remaining[1:]
            require(live_to_guard[:-1]+guard_to_future == remaining,
                    'Split validation changed or omitted original remaining geometry')
            split = {'live_to_guard': validate(live_to_guard, True),
                     'guard_to_future': validate(guard_to_future, True),
                     'unmarked_guard_to_future': validate(guard_to_future, False),
                     'unmarked_guard_attachment': validate(guard_to_future[:2], False),
                     'geometry_preserved': True}
            summary['responses']['split_observation_guard'] = split
            summary['checks'] = {
                'unmodified_remaining_rejected': not summary['responses']['reconstructed_remaining_route']['valid'],
                'both_observed_requests_valid': split['live_to_guard']['valid'] and split['guard_to_future']['valid'],
                'unmarked_original_guard_rejected': not split['unmarked_guard_attachment']['valid'],
                'split_requests_same_map_revision': split['live_to_guard']['replay_map_revision'] ==
                    split['guard_to_future']['replay_map_revision'],
                'all_remaining_segments_preserved': split['geometry_preserved'],
            }
            if planner_parameters.get('ground.tsdf_projection_mode') == 'zero_crossing':
                require(all(summary['checks'].values()), 'Zero-crossing retained-guard comparison failed on saved map')
                require(split['unmarked_guard_attachment']['reason'] == 'terrain_slope_exceeded',
                        'Unmarked historical observation did not reproduce fixed-edge slope rejection')
        summary['status'] = 'completed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
    finally:
        if process is not None and process.poll() is None:
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
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        summary['elapsed_s'] = round(time.monotonic()-started, 3)
        summary['processes_stopped'] = process is None or process.poll() is not None
        output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(summary, ensure_ascii=False))
    return 0 if summary['status'] == 'completed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
