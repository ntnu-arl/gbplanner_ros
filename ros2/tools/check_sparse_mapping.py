#!/usr/bin/env python3
"""Short native mapper/planner check outside the former dense map window."""
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
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))
    import rclpy
    from gbplanner_voxblox.srv import FilePath
    from geometry_msgs.msg import Pose, TransformStamped
    from nav_msgs.msg import Odometry
    from planner_msgs.srv import ValidatePath
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField
    from std_srvs.srv import Trigger
    from tf2_ros import StaticTransformBroadcaster

    def pose(x, y=0., z=0.):
        value = Pose()
        value.position.x, value.position.y, value.position.z = float(x), float(y), float(z)
        value.orientation.w = 1.
        return value

    def records(message):
        fields = {field.name: field.offset for field in message.fields}
        result = {}
        for row in range(message.height):
            for col in range(message.width):
                offset = row*message.row_step + col*message.point_step
                point = tuple(struct.unpack_from('<f', message.data, offset+fields[name])[0]
                              for name in ('x', 'y', 'z'))
                status = struct.unpack_from('b', message.data, offset+fields['occupancy'])[0]
                distance, weight = (struct.unpack_from('<f', message.data, offset+fields[name])[0]
                                    for name in ('tsdf_distance', 'tsdf_weight'))
                result[point] = (status, distance, weight)
        return result

    summary = {'status': 'failed', 'scope': 'native sparse mapper and planner; no dynamics',
               'corridors': [], 'checks': {}}
    processes, raw, accepted = [], [], []
    started = time.monotonic()
    deadline = started + 20.
    probe = None
    with tempfile.TemporaryDirectory(prefix='gbplanner-sparse-') as temporary:
        params = Path(temporary)/'params.yaml'
        common = {'map.storage': 'sparse', 'map.resolution': .25}
        params.write_text(json.dumps({
            'gbplanner_local': {'ros__parameters': dict(common, **{
                'robot.size': [.4, .4, .4],
                'planning.global_min': [-50., -10., -5.],
                'planning.global_max': [50., 10., 5.],
            })},
            'voxblox_mapper': {'ros__parameters': dict(common, **{
                'sensor_frame': 'far_lidar', 'robot.size': [4., 2., 2.],
                'tsdf.method': 'simple', 'update_mesh_every_n_sec': 0.,
            })},
        }))

        def spin_until(predicate, label):
            while not predicate():
                for process in processes:
                    if process.poll() is not None:
                        raise RuntimeError(f'Node exited ({process.returncode}): {args.output.with_suffix(".log")}')
                if time.monotonic() >= deadline:
                    raise TimeoutError(label)
                rclpy.spin_once(probe, timeout_sec=.03)

        def call(client, request):
            spin_until(client.service_is_ready, 'Service discovery')
            future = client.call_async(request)
            spin_until(future.done, 'Service response')
            return future.result()

        def service_status(client):
            reply = call(client, Trigger.Request())
            return json.loads(reply.message)

        def accepted_update(before):
            spin_until(lambda: len(raw) > before, 'Mapper snapshot publication')
            stamp = raw[-1].header.stamp
            spin_until(lambda: any(message.header.stamp == stamp for message in accepted),
                       'Planner did not accept the complete mapper snapshot')
            return next(message for message in reversed(accepted) if message.header.stamp == stamp)

        def validate(path):
            request = ValidatePath.Request()
            request.header.frame_id, request.bound_mode, request.path = 'map', 3, path
            return call(validation, request)

        def stop(process):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=2.)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=1.)

        try:
            rclpy.init(args=[])
            probe = Node('sparse_mapping_probe')
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            probe.create_subscription(PointCloud2, 'voxel_map', raw.append, durable)
            probe.create_subscription(PointCloud2, 'accepted_voxel_map', accepted.append, durable)
            odometry = probe.create_publisher(Odometry, 'odometry', 10)
            clouds = probe.create_publisher(PointCloud2, 'points', qos_profile_sensor_data)
            mapper_status = probe.create_client(Trigger, 'mapper_status')
            planner_status = probe.create_client(Trigger, 'map_status')
            clear = probe.create_client(Trigger, 'clear_robot_unknown_box')
            reset = probe.create_client(Trigger, 'reset_map')
            save = probe.create_client(FilePath, 'save_map')
            load = probe.create_client(FilePath, 'load_map')
            validation = probe.create_client(ValidatePath, 'validate_path')
            current = [40.]

            def publish_odometry():
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                message.header.stamp = probe.get_clock().now().to_msg()
                message.pose.pose = pose(current[0])
                odometry.publish(message)

            probe.create_timer(.03, publish_odometry)
            with args.output.with_suffix('.log').open('w') as log:
                for package, executable in [('gbplanner_local', 'local_planner_node'),
                                            ('gbplanner_voxblox', 'voxblox_mapper_node')]:
                    processes.append(subprocess.Popen([
                        'ros2', 'run', package, executable, '--ros-args', '--params-file', str(params)
                    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=dict(os.environ)))
            spin_until(lambda: clear.service_is_ready() and validation.service_is_ready()
                       and odometry.get_subscription_count() >= 2, 'Mapper/planner discovery')
            for x in (40., -40.):
                current[0] = x
                ready = time.monotonic()+.12
                spin_until(lambda: time.monotonic() >= ready, 'Fresh odometry delivery')
                before = len(raw)
                cleared = call(clear, Trigger.Request())
                if not cleared.success:
                    raise RuntimeError(cleared.message)
                message = accepted_update(before)
                known = records(message)
                local = [key for key, value in known.items()
                         if abs(key[0]-x) < 2.2 and value[0] == 0]
                free = validate([pose(x-1.), pose(x+1.)])
                unknown = validate([pose(x), pose(x, 2.5)])
                item = {'center_x': x, 'free_cells': len(local), 'accepted_snapshot_cells': message.width,
                        'free_path_valid': free.valid, 'free_path_reason': free.reason,
                        'unknown_path_valid': unknown.valid, 'unknown_path_reason': unknown.reason}
                summary['corridors'].append(item)
                if not local or not free.valid or unknown.valid or unknown.reason != 'path_not_free':
                    raise AssertionError(item)

            # The second region must preserve the first without filling the
            # unobserved 80 m gap between them.
            both_paths = [validate([pose(x-1.), pose(x+1.)]).valid for x in (40., -40.)]
            gap = validate([pose(-40.), pose(40.)])
            summary['separate_regions'] = {
                'both_corridor_paths_valid': both_paths,
                'gap_path_valid': gap.valid, 'gap_path_reason': gap.reason,
            }

            # The static acquisition pose is 40 m from the original grid.
            tf = StaticTransformBroadcaster(probe)
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = 'map', 'far_lidar'
            transform.header.stamp = probe.get_clock().now().to_msg()
            transform.transform.translation.x = 40.
            transform.transform.rotation.w = 1.
            tf.sendTransform(transform)
            ready = time.monotonic()+.15
            spin_until(lambda: clouds.get_subscription_count() > 0 and time.monotonic() >= ready,
                       'Static sensor transform discovery')
            cloud = PointCloud2()
            cloud.header.frame_id, cloud.header.stamp = 'far_lidar', probe.get_clock().now().to_msg()
            cloud.width, cloud.height, cloud.point_step = 49, 1, 12
            cloud.row_step, cloud.is_dense = cloud.width*cloud.point_step, True
            cloud.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                            for i, name in enumerate(('x', 'y', 'z'))]
            cloud.data = b''.join(struct.pack('<fff', 4.1, row*.1, col*.1)
                                  for row in range(-3, 4) for col in range(-3, 4))
            before = len(raw)
            clouds.publish(cloud)
            integrated = accepted_update(before)
            published = next(message for message in reversed(raw)
                             if message.header.stamp == integrated.header.stamp)
            same_snapshot = (integrated.fields == published.fields and
                             integrated.data == published.data and
                             integrated.width == published.width and
                             integrated.height == published.height)
            before_save = records(integrated)
            beyond = [key for key, value in before_save.items() if key[0] > 43. and value[2] > .01]
            mapper = service_status(mapper_status)
            planner = service_status(planner_status)
            summary['fusion'] = {'mapper': mapper, 'planner': planner,
                                 'observed_tsdf_cells_x_over_43': len(beyond)}
            if mapper['integrated_scans'] != 1 or mapper['accepted_points'] != 49 or not beyond:
                raise AssertionError(summary['fusion'])
            snapshot = args.output.parent/'sparse_map.vxblx'
            saved = call(save, FilePath.Request(file_path=str(snapshot)))
            if not saved.success:
                raise RuntimeError(saved.message)
            before = len(raw)
            if not call(reset, Trigger.Request()).success:
                raise RuntimeError('Map reset failed')
            empty = accepted_update(before)
            empty_status = service_status(planner_status)
            reset_path = validate([pose(-41.), pose(-39.)])
            before = len(raw)
            loaded = call(load, FilePath.Request(file_path=str(snapshot)))
            if not loaded.success:
                raise RuntimeError(loaded.message)
            restored = accepted_update(before)
            after_load = records(restored)
            restored_paths = [validate([pose(x-1.), pose(x+1.)]).valid for x in (40., -40.)]
            summary['storage'] = {'snapshot_bytes': snapshot.stat().st_size,
                                  'reset_cells': empty.width, 'reset_known': empty_status['known_cells'],
                                  'reset_path_valid': reset_path.valid,
                                  'before_cells': len(before_save), 'restored_cells': len(after_load),
                                  'exact_records_restored': before_save == after_load,
                                  'restored_corridor_paths_valid': restored_paths}
            summary['checks'] = {
                'both_far_corridors': len(summary['corridors']) == 2 and all(
                    row['free_path_valid'] and not row['unknown_path_valid'] for row in summary['corridors']),
                'separate_regions_remain_disconnected': all(both_paths) and not gap.valid
                    and gap.reason == 'path_not_free',
                'native_cloud_fused_outside_old_window': bool(beyond),
                'planner_snapshot_matches_mapper': same_snapshot,
                'planner_accepted_tsdf': planner['tsdf_cells'] == mapper['known_cells'] > 0,
                'reset_cleared_sparse_snapshot': empty.width == empty_status['known_cells'] == 0 and not reset_path.valid,
                'save_load_preserved_full_sparse_map': before_save == after_load and all(restored_paths),
                'no_map_rejections': planner['rejected_maps'] == 0,
            }
            summary['status'] = 'passed' if all(summary['checks'].values()) else 'failed'
        except Exception as error:
            summary['error'] = f'{type(error).__name__}: {error}'
        finally:
            summary['elapsed_seconds'] = round(time.monotonic()-started, 3)
            for process in reversed(processes):
                stop(process)
            if probe is not None:
                probe.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            args.output.write_text(json.dumps(summary, indent=2)+'\n')
            print(json.dumps(summary, indent=2))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
