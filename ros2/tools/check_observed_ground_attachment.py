#!/usr/bin/env python3
"""Replay the run4 ground-attachment failure through the native ROS validator."""
import argparse
import copy
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
    parser.add_argument('--source', type=Path, default=workspace/'results/smb_autonomy/run4',
                        help='Recorded run prefix, without the .json suffix')
    parser.add_argument('--output', type=Path,
                        default=workspace/'results/observed_attachment/summary.json')
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))
    started = time.monotonic()
    process = node = None
    initialized = False
    summary = {
        'status': 'failed', 'domain': os.environ['ROS_DOMAIN_ID'],
        'scope': 'offline native local_planner_node validation of recorded map and odometry',
        'source': str(source), 'checks': {},
        'snapshot_limit': 'Saved revision 110 is replayed; the failure used revision 108. '
                          'The original validation request was not recorded, so its observed '
                          'pose is reconstructed from the last odometry sample before cancellation.',
    }

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

        require(hasattr(ValidatePath.Request(), 'first_pose_is_observation'),
                'Rebuild and source planner_msgs with first_pose_is_observation before running this check')
        sys.path.insert(0, str(workspace/'src/gbplanner_local/scripts'))
        from path_progress import PathProgress

        run = json.loads(source.with_suffix('.json').read_text())
        history = json.loads(source.with_name(source.name+'_history.json').read_text())
        metadata = json.loads(source.with_name(source.name+'_map.json').read_text())
        map_bytes = source.with_name(source.name+'_map.bin').read_bytes()
        require(len(map_bytes) == metadata['data_bytes'] == metadata['row_step']*metadata['height'],
                'Recorded map metadata and binary length disagree')
        case = run['checks']['local_gain_exploration']
        command = case['commanded_path']
        cancellation = next(event for event in history['control']
                            if event['phase'] == 'local_gain_exploration'
                            and event['state'] == 'STOPPING'
                            and event['reason'] == 'path_invalidated:terrain_slope_exceeded')
        with source.with_suffix('.csv').open(newline='') as handle:
            samples = [row for row in csv.DictReader(handle)
                       if row['phase'] == 'local_gain_exploration'
                       and command['time_s'] <= float(row['time_s']) <= cancellation['time_s']]
        require(bool(samples), 'No recorded odometry before the cancellation')
        progress = PathProgress([point[:3] for point in command['path']])
        for row in samples:
            current = tuple(float(row[key]) for key in ('x_m', 'y_m', 'z_m'))
            progress.update(current)
        remaining = progress.remaining(current)
        require(remaining[1] == tuple(command['path'][0][:3]), 'Replay no longer retains the original guard point')
        yaw = float(samples[-1]['yaw_rad'])
        summary['recorded_inputs'] = {
            'odometry_time_s': float(samples[-1]['time_s']),
            'cancellation_time_s': cancellation['time_s'],
            'cancellation_map_revision': cancellation['map_revision'],
            'observed_base_pose': [*current, yaw], 'original_commanded_path': command['path'],
            'remaining_path': remaining, 'progress_m': progress.progress,
            'map_stamp': metadata['stamp'], 'map_cells': metadata['width']*metadata['height'],
            'map_sha256': hashlib.sha256(map_bytes).hexdigest(),
            'raw_connector_slope_degrees': math.degrees(math.atan2(
                remaining[1][2]-remaining[0][2], math.dist(remaining[0][:2], remaining[1][:2]))),
        }

        def pose(values):
            value = Pose()
            value.position.x, value.position.y, value.position.z = map(float, values[:3])
            angle = float(values[3]) if len(values) > 3 else 0.
            value.orientation.z, value.orientation.w = math.sin(angle/2.), math.cos(angle/2.)
            return value

        rclpy.init(args=[])
        initialized = True
        node = Node('observed_ground_attachment_probe')
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
            # Replay the recorded measured pose with a fresh transport stamp.
            message.header.stamp = node.get_clock().now().to_msg()
            message.pose.pose = pose([*current, yaw])
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

        launch = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
                  '--params-file', str(workspace/'src/gbplanner_smb/config/exploration.yaml'),
                  '-p', 'global.enable:=false', '-p', 'global.background_enable:=false']
        summary['launch'] = launch
        log_path = output.with_suffix('.log')
        with log_path.open('w') as log:
            process = subprocess.Popen(launch, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=dict(os.environ))
        deadline = time.monotonic()+20.

        def until(predicate, label):
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
        until(lambda: bool(accepted), 'Recorded map acceptance')
        require(bytes(accepted[-1].data) == map_bytes and accepted[-1].fields == cloud.fields,
                'Accepted map differs from the recorded complete snapshot')
        status = call(map_status, Trigger.Request())
        summary['replayed_map_status'] = json.loads(status.message)
        require(status.success and summary['replayed_map_status']['rejected_maps'] == 0,
                'Planner rejected the recorded map')

        def validate(points, observation):
            request = ValidatePath.Request()
            request.header.frame_id = metadata['frame_id']
            request.bound_mode = 3
            request.path = [pose(point) for point in points]
            request.first_pose_is_observation = observation
            before = copy.deepcopy(request.path)
            reply = call(validator, request)
            return {'valid': reply.valid, 'reason': reply.reason, 'map_revision': reply.map_revision,
                    'first_pose_is_observation': observation,
                    'request_path_unchanged': request.path == before}

        ordinary = validate(remaining, False)
        observed = validate(remaining, True)
        raised = [list(point) for point in remaining]
        raised[0][2] += .2
        off_terrain = validate(raised, True)
        original = validate(command['path'], False)
        summary['responses'] = {'ordinary_connector': ordinary, 'observed_connector': observed,
                                'raised_observation_0_2m': off_terrain, 'original_commanded_path': original}
        summary['checks'] = {
            'ordinary_request_still_rejects_raw_false_slope':
                not ordinary['valid'] and ordinary['reason'] == 'terrain_slope_exceeded',
            'explicit_observed_attachment_passes': observed['valid'] and observed['reason'] == 'ok',
            'raised_observation_is_rejected': not off_terrain['valid'],
            'original_route_remains_valid_and_unchanged': original['valid'] and all(
                result['request_path_unchanged'] for result in summary['responses'].values()),
        }
        require(all(summary['checks'].values()), 'Recorded attachment checks failed')
        summary['status'] = 'passed'
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
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
