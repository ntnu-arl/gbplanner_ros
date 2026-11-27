#!/usr/bin/env python3
"""Diagnose one native local request on run7's saved TSDF map, without motion."""
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


def recorded_inputs(source, config):
    history = json.loads(source.with_name(source.name+'_history.json').read_text())
    metadata = json.loads(source.with_name(source.name+'_map.json').read_text())
    data = source.with_name(source.name+'_map.bin').read_bytes()
    require(len(data) == metadata['data_bytes'] == metadata['row_step']*metadata['height'],
            'Recorded map metadata and binary length disagree')
    failure = next(event for event in history['planning_metrics']
                   if event['phase'] == 'local_gain_exploration'
                   and event['reason'] == 'reference_path_unavailable')
    request = next(event for event in reversed(history['control'])
                   if event['phase'] == 'local_gain_exploration'
                   and event['reason'] == 'local_requested'
                   and event['time_s'] <= failure['time_s'])
    with source.with_suffix('.csv').open(newline='') as handle:
        samples = [row for row in csv.DictReader(handle)
                   if float(row['time_s']) <= request['time_s']]
    require(bool(samples), 'No measured odometry before the local request')
    sample = samples[-1]
    current = [float(sample[key]) for key in ('x_m', 'y_m', 'z_m', 'yaw_rad')]
    require(all(math.isfinite(value) for value in current), 'Invalid recorded odometry')
    saved_revision = next((event['map_revision'] for event in history['map_updates']
                           if event['stamp_sec'] == metadata['stamp']['sec']
                           and event['stamp_nanosec'] == metadata['stamp']['nanosec']), None)
    require(saved_revision is not None, 'Saved map stamp has no recorded revision')
    config_bytes = config.read_bytes()
    details = {
        'observed_base_pose': current,
        'odometry_time_s': float(sample['time_s']), 'odometry_phase': sample['phase'],
        'local_request_time_s': request['time_s'], 'failure_time_s': failure['time_s'],
        'original_failure': failure,
        'failure_map_revision': failure['map_revision'], 'saved_map_revision': saved_revision,
        'map_stamp': metadata['stamp'], 'map_cells': metadata['width']*metadata['height'],
        'map_sha256': hashlib.sha256(data).hexdigest(),
        'config_file': str(config), 'config_sha256': hashlib.sha256(config_bytes).hexdigest(),
        'snapshot_limits': [
            f"Saved map revision {saved_revision} differs from request revision {failure['map_revision']}.",
            'Original candidate waypoints and complete global graph were not recorded. '
            'A fresh native graph is initialized from the measured pose; no historical graph is fabricated.',
            'Only the latest measured pose before the local request is replayed. '
            'The original exploration-direction history is not restored; fresh native heading starts at zero.',
            'The run did not archive its parameter YAML. The selected configuration is copied and hashed '
            'for this replay; equality with the original run configuration is not established.',
        ],
    }
    return details, metadata, data, config_bytes


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=workspace/'results/smb_autonomy/run7',
                        help='Recorded run prefix, without .json')
    parser.add_argument('--config', type=Path,
                        default=workspace/'src/gbplanner_smb/config/exploration.yaml')
    parser.add_argument('--output', type=Path, default=workspace/'results/reference_ground/run7.json')
    parser.add_argument('--wall-budget', type=float, default=25.)
    parser.add_argument('--inputs-only', action='store_true',
                        help='Inspect and save recorded inputs without importing ROS or launching a node')
    args = parser.parse_args()
    source, output, config = args.source.resolve(), args.output.resolve(), args.config.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    started = time.monotonic()
    process = node = None
    initialized = False
    summary = {
        'status': 'failed', 'source': str(source), 'domain': os.environ['ROS_DOMAIN_ID'],
        'scope': 'one native local_planner_node request on a recorded map and stationary measured pose',
        'motion_nodes_started': False, 'local_requests': 0,
    }
    try:
        require(math.isfinite(args.wall_budget) and args.wall_budget > 0, 'Invalid wall budget')
        inputs, metadata, map_bytes, config_bytes = recorded_inputs(source, config)
        summary['recorded_inputs'] = inputs
        saved_config = output.with_suffix('.config.yaml')
        require(saved_config != config, 'Output configuration must not overwrite its input')
        saved_config.write_bytes(config_bytes)
        summary['config_snapshot'] = str(saved_config)
        if args.inputs_only:
            summary['status'] = 'prepared'
            return 0

        import rclpy
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import Odometry, Path as RosPath
        from planner_msgs.msg import Graph
        from planner_msgs.srv import PlannerSrv
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rosidl_runtime_py.convert import message_to_ordereddict
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        def pose(values):
            value = Pose()
            value.position.x, value.position.y, value.position.z = values[:3]
            value.orientation.z, value.orientation.w = math.sin(values[3]/2.), math.cos(values[3]/2.)
            return value

        def state(value):
            q = value.orientation
            return [value.position.x, value.position.y, value.position.z,
                    math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]

        rclpy.init(args=[])
        initialized = True
        node = Node('reference_ground_path_probe')
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        maps = node.create_publisher(PointCloud2, 'voxel_map', durable)
        odometry = node.create_publisher(Odometry, 'odometry', 10)
        accepted, metrics, paths, graphs = [], [], [], []
        node.create_subscription(PointCloud2, 'accepted_voxel_map', accepted.append, durable)
        node.create_subscription(String, 'planning_metrics',
                                 lambda message: metrics.append(json.loads(message.data)), durable)
        node.create_subscription(RosPath, 'local_path', paths.append, durable)
        node.create_subscription(Graph, 'global_graph', graphs.append, durable)
        planner = node.create_client(PlannerSrv, 'planner')
        map_status = node.create_client(Trigger, 'map_status')
        direction_status = node.create_client(Trigger, 'exploration_direction')

        def publish_odometry():
            message = Odometry()
            message.header.frame_id, message.child_frame_id = metadata['frame_id'], 'base_link'
            message.header.stamp = node.get_clock().now().to_msg()
            message.pose.pose = pose(inputs['observed_base_pose'])
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

        # This invokes only the real planner. No SMB, mapper, PCI, tracker,
        # action client, velocity publisher or historical graph loader is started.
        launch = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
                  '--params-file', str(saved_config), '-p', 'global.enable:=true',
                  '-p', 'global.auto_home:=true', '-p', 'global.background_enable:=false']
        summary['launch'] = launch
        log_path = output.with_suffix('.log')
        with log_path.open('w') as log:
            process = subprocess.Popen(launch, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=dict(os.environ))
        deadline = time.monotonic()+args.wall_budget

        def until(predicate, label):
            while not predicate():
                if process.poll() is not None:
                    raise RuntimeError(f'Planner exited with {process.returncode}; see {log_path}')
                if time.monotonic() >= deadline:
                    raise TimeoutError(label)
                rclpy.spin_once(node, timeout_sec=.03)

        def call(client, request):
            until(client.service_is_ready, 'ROS service discovery')
            future = client.call_async(request)
            until(future.done, 'ROS service response')
            return future.result()

        until(lambda: planner.service_is_ready() and maps.get_subscription_count() > 0
              and odometry.get_subscription_count() > 0, 'Planner/map/odometry discovery')
        publish_odometry()
        maps.publish(cloud)
        until(lambda: bool(accepted) and bool(graphs), 'Recorded map and stationary odometry acceptance')
        require(bytes(accepted[-1].data) == map_bytes and accepted[-1].fields == cloud.fields,
                'Accepted map differs from the recorded complete snapshot')
        status = call(map_status, Trigger.Request())
        summary['replayed_map_status'] = json.loads(status.message)
        require(status.success and summary['replayed_map_status']['rejected_maps'] == 0,
                'Planner rejected the recorded map')
        direction = call(direction_status, Trigger.Request())
        summary['replayed_direction'] = json.loads(direction.message)
        until(lambda: bool(graphs[-1].vertices), 'Native auto_home initialization from recorded pose')
        graph_before = message_to_ordereddict(graphs[-1])
        request = PlannerSrv.Request()
        request.header.frame_id = metadata['frame_id']
        request.bound_mode = int(inputs['original_failure']['bound_mode'])
        # Quaternion defaults to w=1 in ROS 2; explicitly zero it to use
        # the all-zero sentinel for actual supplied odometry and
        # exercise GlobalMemory::addReferencePath, not the hypothetical-root path.
        request.root_pose.orientation.w = 0.
        summary['local_requests'] += 1
        response = call(planner, request)
        until(lambda: bool(metrics) and bool(paths), 'Planning metrics and local path publication')
        summary['planning_metrics'] = metrics[-1]
        require(metrics[-1]['auto_global']['reason'] != 'hypothetical_root',
                'Replay used a hypothetical root and skipped reference-path insertion')
        summary['response'] = {'status': response.status, 'bound_mode': response.planning_bound_mode,
                               'base_path': [state(value) for value in response.path]}
        summary['published_base_path'] = [state(value.pose) for value in paths[-1].poses]
        # Topic callbacks are independent; wait for a fresh native graph after
        # the service response instead of labelling a queued old graph "after".
        after_response = node.get_clock().now().nanoseconds
        publish_odometry()
        until(lambda: graphs[-1].header.stamp.sec*1_000_000_000+
              graphs[-1].header.stamp.nanosec >= after_response, 'Post-request native global graph')
        graph_output = output.with_suffix('.graphs.json')
        graph_output.write_text(json.dumps({'before': graph_before,
                                            'after': message_to_ordereddict(graphs[-1])}, indent=2)+'\n')
        summary['replayed_global_graphs'] = str(graph_output)
        detail = metrics[-1].get('reference_path_diagnostics')
        require(detail is not None, 'Rebuild the planner with reference-path diagnostics before running this probe')
        reproduced = metrics[-1]['reason'] == 'reference_path_unavailable'
        if reproduced:
            require(detail['reason'] not in ('not_attempted', 'ok'), 'Missing reference failure detail')
            require(bool(metrics[-1].get('candidate_path')), 'Failed candidate points were not retained')
            require(not response.path and not paths[-1].poses, 'Failed candidate was exposed as a returned route')
        summary['outcome'] = ('reproduced_reference_path_failure' if reproduced else
                              'not_reproduced' if metrics[-1]['success'] else 'different_planning_failure')
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
