#!/usr/bin/env python3
"""Check transform-topic interpolation and real TSDF fusion without any TF publisher."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(150 + os.getpid() % 30))

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_srvs.srv import Trigger

    summary = {
        'status': 'failed',
        'scope': 'native ROS2 mapper; transform-topic queue, dual extrinsics and TSDF/freespace fusion',
        'checks': {}, 'observations': {},
    }
    process = probe = log = None
    started = time.monotonic()
    deadline = started + 25.
    latest_status, snapshots = {}, {}
    resolution, truncation = .2, .6
    log_path = args.output.with_suffix('.log')

    def stop_process():
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=2.)
        except subprocess.TimeoutExpired:
            summary['checks']['mapper_exited_cleanly'] = False
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=1.)

    def cell(point):
        return tuple(int(math.floor(float(value) / resolution)) for value in point)

    def center(index):
        return tuple((value + .5) * resolution for value in index)

    def expected_pose(translation, yaw_degrees):
        # Independent planar geometry: the configured, inverted matrices are
        # T_B_D=[R90,(.4,.2,0)] and T_B_C=[R90,(-.2,.4,0)]. Hence
        # inverse(T_B_D)*T_B_C has identity rotation and translation (.2,.6,0).
        angle = math.radians(yaw_degrees)
        cosine, sine = math.cos(angle), math.sin(angle)
        origin = (translation[0] + .2 * cosine - .6 * sine,
                  translation[1] + .2 * sine + .6 * cosine, translation[2])
        return origin, angle

    def world_point(pose, point):
        origin, angle = pose
        cosine, sine = math.cos(angle), math.sin(angle)
        return (origin[0] + cosine * point[0] - sine * point[1],
                origin[1] + sine * point[0] + cosine * point[1], origin[2] + point[2])

    def expected_distance(origin, endpoint, voxel_center):
        ray = tuple(endpoint[i] - origin[i] for i in range(3))
        length = math.sqrt(sum(value * value for value in ray))
        projected = sum((voxel_center[i] - origin[i]) * ray[i] / length for i in range(3))
        return max(-truncation, min(truncation, length - projected))

    try:
        with tempfile.TemporaryDirectory(prefix='gbplanner-transform-queue-') as temporary:
            params = Path(temporary) / 'params.yaml'
            params.write_text(json.dumps({'voxblox_mapper': {'ros__parameters': {
                'frame_id': 'map', 'sensor_frame': 'lidar',
                'input_sensors': ['main'], 'input.main.topic': 'points', 'input.main.frame': 'lidar',
                'pointcloud_queue_size': 32, 'min_time_between_msgs_sec': 0.,
                'use_tf_transforms': False, 'timestamp_tolerance_sec': .001,
                'T_B_D': [0., 1., 0., -.2, -1., 0., 0., .4, 0., 0., 1., 0., 0., 0., 0., 1.],
                'T_B_C': [0., 1., 0., -.4, -1., 0., 0., -.2, 0., 0., 1., 0., 0., 0., 0., 1.],
                'invert_T_B_D': True, 'invert_T_B_C': True,
                'use_freespace_pointcloud': True, 'freespace.sensor_frames': ['lidar'],
                'map.storage': 'sparse', 'map.resolution': resolution,
                'tsdf.voxels_per_side': 4, 'tsdf.method': 'simple',
                'tsdf.truncation_distance': truncation, 'tsdf.use_weight_dropoff': False,
                'color_mode': 'none', 'esdf.enabled': False,
                'update_mesh_every_n_sec': 0., 'publish_map_every_n_sec': 0.,
            }}}))
            log = log_path.open('w')
            rclpy.init(args=[])
            probe = Node('voxblox_transform_queue_probe')
            reliable = QoSProfile(depth=40, reliability=ReliabilityPolicy.RELIABLE)
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            ordinary = probe.create_publisher(PointCloud2, 'points', reliable)
            freespace = probe.create_publisher(PointCloud2, 'freespace_pointcloud', reliable)
            transforms = probe.create_publisher(TransformStamped, 'transform', reliable)
            status_client = probe.create_client(Trigger, 'mapper_status')

            def record_snapshot(message):
                fields = {field.name: field for field in message.fields}
                if (message.header.frame_id != 'map' or set(fields) != {'x', 'y', 'z', 'intensity'} or
                        len(fields) != len(message.fields) or message.height != 1 or
                        message.row_step != message.width * message.point_step or
                        len(message.data) != message.row_step):
                    raise AssertionError('malformed TSDF distance snapshot')
                if any(field.datatype != PointField.FLOAT32 or field.count != 1 or
                       field.offset + 4 > message.point_step for field in fields.values()):
                    raise AssertionError('TSDF snapshot fields must be scalar float32')
                endian, records = ('>' if message.is_bigendian else '<'), {}
                for i in range(message.width):
                    values = tuple(struct.unpack_from(endian + 'f', message.data,
                                                      i * message.point_step + fields[name].offset)[0]
                                   for name in ('x', 'y', 'z', 'intensity'))
                    if not all(math.isfinite(value) for value in values):
                        raise AssertionError('TSDF snapshot contains nonfinite values')
                    index = cell(values[:3])
                    if index in records:
                        raise AssertionError('TSDF snapshot repeats a voxel')
                    records[index] = (values[:3], values[3])
                stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
                snapshots[stamp] = records

            subscription = probe.create_subscription(PointCloud2, 'tsdf_pointcloud', record_snapshot, durable)
            process = subprocess.Popen(
                ['ros2', 'run', 'gbplanner_voxblox', 'voxblox_mapper_node',
                 '--ros-args', '--params-file', str(params)],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=dict(os.environ))

            def ensure_running(label):
                if process.poll() is not None:
                    raise RuntimeError(f'mapper exited ({process.returncode}); see {log_path}')
                if time.monotonic() >= deadline:
                    raise TimeoutError(label)

            def spin_until(predicate, label):
                while True:
                    ensure_running(label)
                    value = predicate()
                    if value:
                        return value
                    rclpy.spin_once(probe, timeout_sec=.01)

            def status():
                nonlocal latest_status
                future = status_client.call_async(Trigger.Request())
                spin_until(future.done, 'mapper_status response')
                response = future.result()
                if response is None:
                    raise RuntimeError('empty mapper_status response')
                latest_status = json.loads(response.message)
                return latest_status

            def wait_status(predicate, label):
                while True:
                    ensure_running(label)
                    current = status()
                    if predicate(current):
                        return current
                    rclpy.spin_once(probe, timeout_sec=.01)

            def pause(seconds, label):
                ready = time.monotonic() + seconds
                spin_until(lambda: time.monotonic() >= ready, label)

            def check(name, condition):
                summary['checks'][name] = bool(condition)
                if not condition:
                    raise AssertionError(name)

            def remember(name, current):
                summary['observations'][name] = current
                return current

            def source(current):
                return current['input_sources']['main']

            def cloud(stamp, point):
                message = PointCloud2()
                message.header.frame_id, message.header.stamp.sec = 'lidar', int(stamp)
                message.width, message.height = 1, 1
                message.point_step, message.row_step = 12, 12
                message.is_dense, message.is_bigendian = True, False
                message.fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                                  for i, name in enumerate(('x', 'y', 'z'))]
                message.data = struct.pack('<fff', *point)
                return message

            def transform(stamp, translation, yaw_degrees):
                message = TransformStamped()
                message.header.frame_id, message.child_frame_id = 'map', 'tracking'
                message.header.stamp.sec = int(stamp)
                (message.transform.translation.x, message.transform.translation.y,
                 message.transform.translation.z) = translation
                half_angle = math.radians(yaw_degrees) * .5
                message.transform.rotation.z = math.sin(half_angle)
                message.transform.rotation.w = math.cos(half_angle)
                return message

            def snapshot_at(stamp, condition=lambda records: True):
                stamp_ns = int(stamp) * 1_000_000_000
                return spin_until(lambda: snapshots.get(stamp_ns)
                                  if stamp_ns in snapshots and condition(snapshots[stamp_ns]) else None,
                                  f'TSDF snapshot at {stamp}s')

            def check_return(name, records, pose, local_point):
                origin, _ = pose
                endpoint = world_point(pose, local_point)
                endpoint_index = cell(endpoint)
                expected = expected_distance(origin, endpoint, center(endpoint_index))
                observed = records.get(endpoint_index)
                summary['observations'][name] = {
                    'sensor_origin': list(origin), 'world_endpoint': list(endpoint),
                    'endpoint_voxel': list(endpoint_index), 'expected_tsdf_distance_m': expected,
                    'observed_tsdf_distance_m': None if observed is None else observed[1],
                    'observed_voxels': len(records),
                }
                check(name, observed is not None and abs(observed[1] - expected) < 2e-5)

            spin_until(status_client.service_is_ready, 'mapper_status discovery')
            spin_until(lambda: all(publisher.get_subscription_count() > 0
                                  for publisher in (ordinary, freespace, transforms)), 'input discovery')
            spin_until(lambda: probe.count_publishers('tsdf_pointcloud') > 0, 'TSDF publisher discovery')
            initial = remember('initial', status())
            check('transform_topic_mode_enabled', initial['use_tf_transforms'] is False and
                  initial['received_transforms'] == 0 and initial['queued_transforms'] == 0)

            local_return = (1.2, 0., 0.)
            ordinary.publish(cloud(11, local_return))
            waiting = remember('without_transform', wait_status(
                lambda current: source(current)['pending_scans'] == 1, 'cloud waiting without a transform'))
            check('missing_transform_retains_cloud', waiting['integrated_scans'] == 0 and
                  source(waiting)['rejected_scans'] == 0)

            transforms.publish(transform(10, (1.2, -.2, .1), 45.))
            wait_status(lambda current: current['received_transforms'] == 1, 'first transform delivery')
            pause(.08, 'single-sided sample remains pending across queue retries')
            one_sided = remember('only_older_transform', status())
            check('older_sample_is_not_extrapolated', source(one_sided)['pending_scans'] == 1 and
                  one_sided['integrated_scans'] == 0 and one_sided['queued_transforms'] == 1)

            transforms.publish(transform(12, (2.2, .8, .1), 135.))
            interpolated = remember('after_bracketing_transform', wait_status(
                lambda current: current['integrated_scans'] == 1, 'interpolated cloud fusion'))
            check('new_transform_recovers_pending_cloud', source(interpolated)['pending_scans'] == 0 and
                  interpolated['received_transforms'] == 2 and interpolated['queued_transforms'] == 2)
            first_pose = expected_pose((1.7, .3, .1), 90.)
            first = snapshot_at(11)
            check_return('midpoint_yaw_translation_and_both_extrinsics', first, first_pose, local_return)
            endpoint = world_point(first_pose, local_return)
            max_distance_error = max(abs(distance - expected_distance(first_pose[0], endpoint, point))
                                     for point, distance in first.values())
            summary['observations']['first_ray_max_tsdf_error_m'] = max_distance_error
            check('first_ray_tsdf_matches_independent_projection', max_distance_error < 2e-5 and
                  all(abs(point[0] - 1.1) < 2e-5 and abs(point[2] - .1) < 2e-5
                      for point, _ in first.values()))

            # Same acquisition stamp is valid on the independent freespace
            # stream. Its sideways ray adds cells absent from the first return.
            free_cell = cell((.5, .5, .1))
            check('freespace_target_initially_unknown', free_cell not in first)
            freespace.publish(cloud(11, (0., 1.6, 0.)))
            free_status = remember('independent_freespace', wait_status(
                lambda current: current['freespace']['integrated_clouds'] == 1, 'freespace queue fusion'))
            free_map = snapshot_at(11, lambda records: free_cell in records)
            new_free_cells = set(free_map) - set(first)
            summary['observations']['freespace_geometry'] = {
                'target_voxel': list(free_cell), 'target_distance_m': free_map[free_cell][1],
                'new_voxels': len(new_free_cells),
                'minimum_new_distance_m': min(free_map[index][1] for index in new_free_cells),
            }
            check('freespace_uses_same_transform_and_independent_stream',
                  free_status['integrated_scans'] == 1 and free_status['received_transforms'] == 2 and
                  free_status['freespace']['pending_clouds'] == 0 and
                  free_status['freespace']['throttled_clouds'] == 0 and
                  abs(free_map[free_cell][1] - truncation) < 2e-5 and
                  all(free_map[index][1] > resolution for index in new_free_cells))

            ordinary.publish(cloud(13, local_return))
            wait_status(lambda current: source(current)['pending_scans'] == 1, 'future cloud held for bracketing')
            pause(.08, 'future sample remains pending across queue retries')
            future_wait = remember('beyond_newest_transform', status())
            check('newest_sample_is_not_extrapolated', future_wait['integrated_scans'] == 1 and
                  source(future_wait)['pending_scans'] == 1 and source(future_wait)['rejected_scans'] == 0)
            transforms.publish(transform(14, (3.2, 1.8, .1), 225.))
            final = remember('after_future_bracket', wait_status(
                lambda current: current['integrated_scans'] == 2, 'future transform recovers cloud'))
            final_map = snapshot_at(13)
            check_return('recovered_future_cloud_has_correct_pose', final_map,
                         expected_pose((2.7, 1.3, .1), 180.), local_return)
            check('queue_prunes_only_samples_before_used_bracket', final['received_transforms'] == 3 and
                  final['queued_transforms'] == 2 and source(final)['pending_scans'] == 0)
            check('all_inputs_processed_without_rejections', final['rejected_transforms'] == 0 and
                  final['rejected_scans'] == 0 and source(final)['queue_dropped_scans'] == 0 and
                  final['freespace']['rejected_clouds'] == 0 and final['freespace']['integrated_clouds'] == 1)
            summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
    finally:
        stop_process()
        summary['mapper_exit_code'] = None if process is None else process.returncode
        if process is not None:
            summary['checks'].setdefault('mapper_exited_cleanly', process.returncode == 0)
        if log is not None:
            log.close()
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        summary['last_mapper_status'] = latest_status
        summary['map_snapshots'] = [{'stamp_ns': stamp, 'observed_voxels': len(records)}
                                    for stamp, records in sorted(snapshots.items())]
        if not all(summary['checks'].values()):
            summary['status'] = 'failed'
        summary['elapsed_seconds'] = round(time.monotonic() - started, 3)
        args.output.write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
