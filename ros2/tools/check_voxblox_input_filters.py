#!/usr/bin/env python3
"""Check acquisition-time filtering and delayed-TF recovery in a native mapper."""
import argparse
import json
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
    os.environ.setdefault('ROS_DOMAIN_ID', str(140 + os.getpid() % 40))

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_srvs.srv import Trigger
    from tf2_ros import StaticTransformBroadcaster

    summary = {
        'status': 'failed',
        'scope': 'native ROS2 mapper; acquisition-time admission, delayed TF and sensor-pose ESDF clearing',
        'ros2_adaptation': 'A periodic retry drains pending TF queues without a successor cloud.',
        'checks': {}, 'observations': {},
    }
    process, probe, log = None, None, None
    started = time.monotonic()
    deadline = started + 25.
    latest_status, snapshots = {}, []

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
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=1.)

    try:
        with tempfile.TemporaryDirectory(prefix='gbplanner-input-filter-') as temporary:
            params = Path(temporary) / 'params.yaml'
            params.write_text(json.dumps({'voxblox_mapper': {'ros__parameters': {
                'frame_id': 'map',
                'input_sensors': ['main', 'delayed'],
                'input.main.topic': 'points', 'input.main.frame': 'lidar',
                'input.delayed.topic': 'delayed_points', 'input.delayed.frame': 'delayed_lidar',
                'pointcloud_queue_size': 32,
                'min_time_between_msgs_sec': .25,
                # This compatibility value belongs to the non-TF transformer
                # in ROS1; it must not relax TF-mode acquisition admission.
                'timestamp_tolerance_sec': 100.,
                'use_freespace_pointcloud': True, 'freespace.sensor_frames': ['lidar'],
                'map.storage': 'dense', 'map.resolution': .25,
                'map.origin': [-2., -2., -2.], 'map.dimensions': [16, 16, 16],
                'tsdf.method': 'simple', 'color_mode': 'none', 'update_mesh_every_n_sec': 0.,
                'esdf.enabled': True, 'esdf.clear_sphere_for_planning': True,
                'esdf.clear_sphere_radius': .4, 'esdf.occupied_sphere_radius': 1.,
            }}}))
            log_path = args.output.with_suffix('.log')
            log = log_path.open('w')
            rclpy.init(args=[])
            probe = Node('voxblox_input_filter_probe')
            # Reliable publishers match best-effort inputs. The larger depth
            # avoids losing either cloud in the intentional two-message burst.
            publisher_qos = QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE)
            main_clouds = probe.create_publisher(PointCloud2, 'points', publisher_qos)
            delayed_clouds = probe.create_publisher(PointCloud2, 'delayed_points', publisher_qos)
            free_clouds = probe.create_publisher(PointCloud2, 'freespace_pointcloud', publisher_qos)
            durable = QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)

            def record_snapshot(message):
                snapshots.append({
                    'stamp_ns': message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
                    'cells': message.width * message.height,
                })

            probe.create_subscription(PointCloud2, 'voxel_map', record_snapshot, durable)
            status_client = probe.create_client(Trigger, 'mapper_status')
            tf = StaticTransformBroadcaster(probe)

            def transform(frame, x=0.):
                value = TransformStamped()
                value.header.frame_id, value.child_frame_id = 'map', frame
                value.header.stamp = probe.get_clock().now().to_msg()
                value.transform.translation.x, value.transform.rotation.w = x, 1.
                return value

            main_tf = transform('lidar')
            tf.sendTransform(main_tf)
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
                while not predicate():
                    ensure_running(label)
                    rclpy.spin_once(probe, timeout_sec=.01)

            def status():
                nonlocal latest_status
                request = status_client.call_async(Trigger.Request())
                spin_until(request.done, 'mapper_status response')
                response = request.result()
                if response is None:
                    raise RuntimeError('empty mapper status response')
                # success=false is expected before the first cloud has fused.
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

            def source(current, name='main'):
                return current['input_sources'][name]

            def cloud(stamp_sec, frame='lidar'):
                stamp_ns = int(round(stamp_sec * 1_000_000_000))
                message = PointCloud2()
                message.header.frame_id = frame
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
                message.height, message.width = 1, 1
                message.point_step, message.row_step = 12, 12
                message.is_dense = True
                message.fields = [PointField(name=name, offset=offset,
                                              datatype=PointField.FLOAT32, count=1)
                                  for name, offset in [('x', 0), ('y', 4), ('z', 8)]]
                message.data = struct.pack('<fff', .75, 0., 0.)
                return message

            def wait_source_count(field, count, label, name='main'):
                return wait_status(lambda current: source(current, name)[field] >= count, label)

            def remember(label, current):
                summary['observations'][label] = current
                return current

            spin_until(status_client.service_is_ready, 'mapper_status discovery')
            spin_until(lambda: all(pub.get_subscription_count() > 0
                                  for pub in (main_clouds, delayed_clouds, free_clouds)),
                       'all pointcloud subscription discovery')
            tf.sendTransform(main_tf)
            pause(.1, 'initial static TF delivery')
            subscriptions = probe.get_subscriptions_info_by_topic('/points')
            # Some RMW graph-discovery implementations report depth zero
            # because they do not expose history depth in endpoint discovery.
            # Preserve that observation and check the running subscription's
            # actual QoS separately through mapper_status below.
            summary['main_subscription_depths'] = [entry.qos_profile.depth for entry in subscriptions
                                                    if entry.node_name == 'voxblox_mapper']

            # Acquisition times differ by .5 s, but publishing is consecutive.
            burst_started = time.monotonic()
            main_clouds.publish(cloud(10.))
            main_clouds.publish(cloud(10.5))
            summary['burst_publication_seconds'] = time.monotonic() - burst_started
            burst = remember('rapid_acquisition_separated_clouds', wait_source_count(
                'integrated_scans', 2, 'rapid clouds were not both integrated'))

            pause(.30, 'wall time beyond the .25 s threshold')
            main_clouds.publish(cloud(10.625))
            wall_wait = remember('wall_wait_but_short_acquisition_interval', wait_source_count(
                'throttled_scans', 1, 'short acquisition interval was not throttled'))
            # .25 is exactly representable; equality must fail strict >.
            main_clouds.publish(cloud(10.75))
            boundary = remember('strict_interval_boundary', wait_source_count(
                'throttled_scans', 2, 'equal acquisition interval was not throttled'))
            main_clouds.publish(cloud(9.))
            reversed_stamp = remember('reversed_timestamp', wait_source_count(
                'throttled_scans', 3, 'reversed timestamp was not throttled'))
            main_clouds.publish(cloud(10.875))
            recovered = remember('after_throttled_stamps', wait_source_count(
                'integrated_scans', 3, 'later acquisition cloud did not integrate'))

            # Free-space is independent even with the same sensor frame and an
            # older timestamp than the latest ordinary cloud.
            free_clouds.publish(cloud(10.5))
            free_first = remember('independent_freespace_first', wait_status(
                lambda current: current['freespace']['integrated_clouds'] >= 1,
                'free-space did not use an independent time baseline'))
            free_clouds.publish(cloud(10.625))
            wait_status(lambda current: current['freespace']['throttled_clouds'] >= 1,
                        'short free-space interval was not throttled')
            free_clouds.publish(cloud(10.75))
            free_boundary = remember('freespace_strict_interval_boundary', wait_status(
                lambda current: current['freespace']['throttled_clouds'] >= 2,
                'equal free-space interval was not throttled'))
            free_clouds.publish(cloud(10.875))
            free_recovered = remember('freespace_after_throttled_stamps', wait_status(
                lambda current: current['freespace']['integrated_clouds'] >= 2,
                'later free-space cloud did not integrate'))

            # Missing TF retains a separate source's cloud and advances its
            # admission baseline, while the main source must keep working.
            delayed_clouds.publish(cloud(20., 'delayed_lidar'))
            pending = remember('delayed_tf_pending', wait_source_count(
                'pending_scans', 1, 'missing-TF cloud was not retained', name='delayed'))
            main_clouds.publish(cloud(11.25))
            main_while_pending = remember('main_integrates_while_other_source_waits', wait_source_count(
                'integrated_scans', 4, 'missing TF on delayed source blocked main input'))
            delayed_clouds.publish(cloud(20.125, 'delayed_lidar'))
            pending_guard = remember('pending_cloud_already_advanced_guard', wait_source_count(
                'throttled_scans', 1, 'pending cloud did not advance admission stamp', name='delayed'))

            # Reach the ROS1 internal limit of 10: drop the oldest, retain 9.
            for admitted_count, stamp in enumerate((20.5, 21., 21.5, 22., 22.5, 23., 23.5, 24., 24.5), 2):
                delayed_clouds.publish(cloud(stamp, 'delayed_lidar'))
                wait_status(lambda current, total=admitted_count:
                            source(current, 'delayed')['pending_scans'] +
                            source(current, 'delayed')['queue_dropped_scans'] >= total,
                            'missing-TF queue did not receive admitted cloud')
            queue_full = remember('missing_tf_queue_limit', status())

            # Only TF is published now. Periodic retry must drain the retained
            # clouds without needing a successor pointcloud on this source.
            tf.sendTransform([main_tf, transform('delayed_lidar', .5)])
            final = remember('after_delayed_tf_without_successor_cloud', wait_source_count(
                'integrated_scans', 9, 'retained clouds did not integrate after TF arrived', name='delayed'))
            spin_until(lambda: snapshots and snapshots[-1]['stamp_ns'] == 24_500_000_000,
                       'delayed cloud final map snapshot missing')

            main_final, delayed_final, free_final = source(final), source(final, 'delayed'), final['freespace']
            summary['checks'] = {
                'subscriber_queue_depth_applied': main_final['subscription_depth'] == 32
                    and delayed_final['subscription_depth'] == 32,
                'configured_acquisition_interval': final['pointcloud_queue_size'] == 32
                    and final['min_time_between_msgs_sec'] == .25
                    and final['timestamp_tolerance_sec'] == 100.,
                'rapid_clouds_with_sufficient_acquisition_gap_integrate': source(burst)['integrated_scans'] == 2
                    and source(burst)['throttled_scans'] == 0 and summary['burst_publication_seconds'] < .25,
                'wall_wait_does_not_admit_short_acquisition_gap': source(wall_wait)['integrated_scans'] == 2
                    and source(wall_wait)['throttled_scans'] == 1,
                'strict_acquisition_boundary': source(boundary)['integrated_scans'] == 2
                    and source(boundary)['throttled_scans'] == 2,
                'reverse_stamp_throttled_not_rejected': source(reversed_stamp)['integrated_scans'] == 2
                    and source(reversed_stamp)['throttled_scans'] == 3
                    and source(reversed_stamp)['rejected_scans'] == 0,
                'filtered_stamps_do_not_advance_baseline': source(recovered)['integrated_scans'] == 3,
                'freespace_has_independent_guard': free_first['freespace']['integrated_clouds'] == 1
                    and free_first['freespace']['throttled_clouds'] == 0,
                'freespace_uses_same_strict_guard': free_boundary['freespace']['integrated_clouds'] == 1
                    and free_boundary['freespace']['throttled_clouds'] == 2
                    and free_recovered['freespace']['integrated_clouds'] == 2,
                'missing_tf_is_pending_not_rejected': source(pending, 'delayed')['pending_scans'] == 1
                    and source(pending, 'delayed')['integrated_scans'] == 0
                    and source(pending, 'delayed')['rejected_scans'] == 0,
                'pending_source_does_not_block_main': source(main_while_pending)['integrated_scans'] == 4
                    and source(main_while_pending, 'delayed')['pending_scans'] == 1,
                'admission_updates_guard_before_tf_success': source(pending_guard, 'delayed')['pending_scans'] == 1
                    and source(pending_guard, 'delayed')['throttled_scans'] == 1,
                'internal_tf_queue_retains_nine_at_limit': source(queue_full, 'delayed')['pending_scans'] == 9
                    and source(queue_full, 'delayed')['queue_dropped_scans'] == 1,
                'delayed_tf_recovers_without_successor_cloud': delayed_final['integrated_scans'] == 9
                    and delayed_final['pending_scans'] == 0 and delayed_final['queue_dropped_scans'] == 1
                    and snapshots[-1]['cells'] > 0,
                'final_counts_match_inputs': main_final['integrated_scans'] == 4
                    and main_final['throttled_scans'] == 3 and main_final['pending_scans'] == 0
                    and main_final['queue_dropped_scans'] == 0
                    and free_final['integrated_clouds'] == 2 and free_final['throttled_clouds'] == 2
                    and free_final['pending_clouds'] == 0 and free_final['queue_dropped_clouds'] == 0
                    and final['integrated_scans'] == 13,
                'no_rejected_clouds': final['rejected_scans'] == 0
                    and delayed_final['rejected_scans'] == 0 and free_final['rejected_clouds'] == 0,
                'sensor_pose_clears_esdf_once_per_successful_cloud': final['esdf_clear_sphere_operations']
                    == final['integrated_scans'] + free_final['integrated_clouds'] == 15,
            }
            summary['status'] = 'passed' if all(summary['checks'].values()) else 'failed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
    finally:
        stop_process()
        summary['mapper_exit_code'] = None if process is None else process.returncode
        if process is not None and process.returncode != 0:
            summary['status'] = 'failed'
            summary.setdefault('error', f'mapper exited with code {process.returncode}')
        if log is not None:
            log.close()
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        summary['last_status'] = latest_status
        summary['map_snapshots'] = snapshots
        summary['elapsed_seconds'] = round(time.monotonic() - started, 3)
        args.output.write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
