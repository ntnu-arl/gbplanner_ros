#!/usr/bin/env python3
"""Check the mapper's ROS2 odometry path for Voxblox ESDF clear spheres."""
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
    parser.add_argument('--primary-default-overrides-legacy', action='store_true',
                        help='check that an explicit namespaced default is not replaced by a legacy alias')
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(180 + os.getpid() % 30))

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from std_srvs.srv import Trigger
    from tf2_ros import StaticTransformBroadcaster
    from geometry_msgs.msg import TransformStamped

    summary = {'status': 'failed', 'scope': 'native ROS2 ESDF planning clear spheres', 'checks': {}}
    process = None
    probe = None
    started = time.monotonic()
    deadline = started + 20.

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
        with tempfile.TemporaryDirectory(prefix='gbplanner-esdf-clear-') as temporary:
            params = Path(temporary) / 'params.yaml'
            params.write_text(json.dumps({'voxblox_mapper': {'ros__parameters': {
                'frame_id': 'map', 'sensor_frame': 'lidar',
                'map.storage': 'dense', 'map.resolution': .2,
                'map.origin': [-2., -2., -2.], 'map.dimensions': [20, 20, 20],
                'esdf.enabled': True,
                'esdf.clear_sphere_pose_source': 'odometry',
                # Exercise the original ROS1 parameter names. The node also
                # accepts the namespaced ROS2 spellings when both are absent.
                'esdf_max_distance_m': 2.,
                'esdf.clear_sphere_for_planning': False if args.primary_default_overrides_legacy else True,
                'clear_sphere_for_planning': True,
                'clear_sphere_radius': .4,
                'occupied_sphere_radius': 1.,
                'color_mode': 'none', 'update_mesh_every_n_sec': 0.,
            }}}))
            log_path = args.output.with_suffix('.log')
            log = log_path.open('w')
            rclpy.init(args=[])
            probe = Node('esdf_clear_sphere_probe')
            odometry = probe.create_publisher(Odometry, 'odometry', 10)
            status_client = probe.create_client(Trigger, 'mapper_status')
            snapshots = []
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            probe.create_subscription(PointCloud2, 'esdf_voxels', snapshots.append, durable)
            tf = StaticTransformBroadcaster(probe)
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = 'map', 'lidar'
            transform.header.stamp = probe.get_clock().now().to_msg()
            transform.transform.rotation.w = 1.
            tf.sendTransform(transform)
            process = subprocess.Popen(
                ['ros2', 'run', 'gbplanner_voxblox', 'voxblox_mapper_node',
                 '--ros-args', '--params-file', str(params)],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                env=dict(os.environ))

            def spin_until(predicate, label):
                while not predicate():
                    if process.poll() is not None:
                        raise RuntimeError(f'mapper exited ({process.returncode}); see {log_path}')
                    if time.monotonic() >= deadline:
                        raise TimeoutError(label)
                    rclpy.spin_once(probe, timeout_sec=.02)

            spin_until(status_client.service_is_ready, 'mapper_status discovery')
            spin_until(lambda: odometry.get_subscription_count() > 0, 'odometry discovery')

            def publish_odom():
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                message.header.stamp = probe.get_clock().now().to_msg()
                message.pose.pose.orientation.w = 1.
                odometry.publish(message)

            def status():
                future = status_client.call_async(Trigger.Request())
                spin_until(future.done, 'mapper_status response')
                response = future.result()
                if response is None or (not response.success and not args.primary_default_overrides_legacy):
                    raise RuntimeError(response.message if response else 'empty mapper status')
                return json.loads(response.message)

            for _ in range(4):
                publish_odom()
                time.sleep(.04)
                rclpy.spin_once(probe, timeout_sec=.01)
            if args.primary_default_overrides_legacy:
                spin_until(lambda: status().get('esdf_clear_sphere_operations', 0) == 0,
                           'legacy alias incorrectly overrides explicit namespaced default')
            else:
                spin_until(lambda: status().get('esdf_clear_sphere_operations', 0) > 0,
                           'ESDF clear sphere operation')
                spin_until(lambda: bool(snapshots), 'ESDF snapshot publication')
            mapper = status()
            summary['observations'] = {
                'mapper_status': mapper,
                'snapshot_points': 0,
            }
            summary['checks'] = {
                'explicit_namespaced_default_wins': (
                    mapper['esdf_clear_sphere_operations'] == 0
                    if args.primary_default_overrides_legacy else
                    mapper['esdf_clear_sphere_operations'] > 0),
            }
            if not args.primary_default_overrides_legacy:
                message = snapshots[-1]
                distance_offset = next(field.offset for field in message.fields if field.name == 'distance')
                distances = [struct.unpack_from('<f', message.data,
                                                 row * message.row_step + col * message.point_step + distance_offset)[0]
                             for row in range(message.height) for col in range(message.width)]
                positive = sum(value > 0. for value in distances)
                negative = sum(value < 0. for value in distances)
                summary['observations'].update({
                    'snapshot_points': len(distances),
                    'positive_distance_voxels': positive,
                    'negative_distance_voxels': negative,
                })
                summary['checks'].update({
                    'inner_sphere_published_as_free_distance': positive > 0,
                    'outer_sphere_published_as_occupied_distance': negative > 0,
                })
            summary['status'] = 'passed' if all(summary['checks'].values()) else 'failed'
            log.close()
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
    finally:
        stop_process()
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        summary['elapsed_seconds'] = round(time.monotonic() - started, 3)
        args.output.write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
