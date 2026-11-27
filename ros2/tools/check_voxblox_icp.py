#!/usr/bin/env python3
"""Check native ICP correction, its stamped TF tree and actual corrected TSDF fusion."""
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


def rotate(quaternion, point):
    x, y, z, w = quaternion
    length = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(length) or length < 1e-10:
        raise AssertionError('invalid output quaternion')
    x, y, z, w = (value / length for value in (x, y, z, w))
    return ((1. - 2. * (y * y + z * z)) * point[0] + 2. * (x * y - z * w) * point[1] +
            2. * (x * z + y * w) * point[2],
            2. * (x * y + z * w) * point[0] + (1. - 2. * (x * x + z * z)) * point[1] +
            2. * (y * z - x * w) * point[2],
            2. * (x * z - y * w) * point[0] + 2. * (y * z + x * w) * point[1] +
            (1. - 2. * (x * x + y * y)) * point[2])


def apply(transform, point):
    rotated = rotate(transform['quaternion_xyzw'], point)
    return tuple(rotated[i] + transform['translation'][i] for i in range(3))


def same_transform(left, right, tolerance=2e-5):
    probes = ((0., 0., 0.), (1., 0., 0.), (0., 1., 0.), (0., 0., 1.))
    return all(abs(a - b) <= tolerance for point in probes
               for a, b in zip(apply(left, point), apply(right, point)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--axis-aligned', action='store_true',
                        help='Exercise the original failing zero-direction ray after the core fix')
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
    from tf2_msgs.msg import TFMessage
    from voxblox_msgs.msg import Block, Layer

    resolution, side, truncation = .05, 8, .15
    icp_parameters = {
        'enable_icp': True, 'icp_num_threads': 1, 'icp_mini_batch_size': 20,
        'icp_min_match_ratio': .8, 'icp_subsample_keep_ratio': 1., 'icp_refine_roll_pitch': False,
        # Use a weak initial-pose prior for this analytic alignment fixture;
        # these are explicit test settings, not claims about default weights.
        'icp_inital_translation_weighting': 1., 'icp_inital_rotation_weighting': 1.,
        'accumulate_icp_corrections': True,
    }
    summary = {
        'status': 'failed',
        'scope': 'native ROS2 mapper; analytic-plane ICP and corrected freespace integration after reset',
        'parameters': icp_parameters, 'checks': {}, 'observations': {},
    }
    process = probe = log = None
    started = time.monotonic()
    deadline = started + 25.
    latest_status, snapshots, corrections, tf_links = {}, {}, {}, {}
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

    def stamp_ns(message):
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

    def decode_transform(message):
        position, quaternion = message.transform.translation, message.transform.rotation
        value = {
            'translation': [float(position.x), float(position.y), float(position.z)],
            'quaternion_xyzw': [float(quaternion.x), float(quaternion.y), float(quaternion.z), float(quaternion.w)],
        }
        if not all(math.isfinite(v) for values in value.values() for v in values):
            raise AssertionError('ICP published a nonfinite transform')
        rotate(value['quaternion_xyzw'], (0., 0., 0.))
        return value

    def float_word(value):
        return struct.unpack('<I', struct.pack('<f', value))[0]

    try:
        with tempfile.TemporaryDirectory(prefix='gbplanner-icp-') as temporary:
            params = Path(temporary) / 'params.yaml'
            params.write_text(json.dumps({'voxblox_mapper': {'ros__parameters': {
                'frame_id': 'map', 'sensor_frame': 'lidar',
                'use_tf_transforms': False, 'timestamp_tolerance_sec': .001,
                'use_freespace_pointcloud': True, 'freespace.sensor_frames': ['lidar'],
                'pointcloud_queue_size': 32, 'min_time_between_msgs_sec': 0.,
                'map.storage': 'sparse', 'map.resolution': resolution, 'tsdf.voxels_per_side': side,
                'tsdf.method': 'simple', 'tsdf.truncation_distance': truncation,
                'tsdf.use_weight_dropoff': False, 'color_mode': 'none', 'esdf.enabled': False,
                'publish_pointclouds_on_update': True,
                'update_mesh_every_n_sec': 0., 'publish_map_every_n_sec': 0., **icp_parameters,
            }}}))
            log = log_path.open('w')
            rclpy.init(args=[])
            probe = Node('voxblox_icp_probe')
            reliable = QoSProfile(depth=40, reliability=ReliabilityPolicy.RELIABLE)
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            ordinary = probe.create_publisher(PointCloud2, 'points', reliable)
            freespace = probe.create_publisher(PointCloud2, 'freespace_pointcloud', reliable)
            transforms = probe.create_publisher(TransformStamped, 'transform', reliable)
            layers = probe.create_publisher(Layer, 'tsdf_map_in', reliable)
            clients = {name: probe.create_client(Trigger, name) for name in ('mapper_status', 'reset_map')}

            def record_snapshot(message):
                fields = {field.name: field for field in message.fields}
                if (message.header.frame_id != 'map' or set(fields) != {'x', 'y', 'z', 'intensity'} or
                        len(fields) != len(message.fields) or message.height != 1 or
                        message.row_step != message.width * message.point_step or
                        len(message.data) != message.row_step):
                    raise AssertionError('malformed TSDF pointcloud')
                if any(field.datatype != PointField.FLOAT32 or field.count != 1 or
                       field.offset + 4 > message.point_step for field in fields.values()):
                    raise AssertionError('TSDF fields must be scalar float32')
                records, endian = {}, ('>' if message.is_bigendian else '<')
                for i in range(message.width):
                    values = tuple(struct.unpack_from(endian + 'f', message.data,
                                                      i * message.point_step + fields[name].offset)[0]
                                   for name in ('x', 'y', 'z', 'intensity'))
                    if not all(math.isfinite(value) for value in values):
                        raise AssertionError('nonfinite TSDF record')
                    key = cell(values[:3])
                    if key in records:
                        raise AssertionError('TSDF snapshot repeats a voxel')
                    records[key] = (values[:3], values[3])
                snapshots[stamp_ns(message)] = records

            def record_correction(message):
                if message.header.frame_id != 'map' or message.child_frame_id != 'icp_corrected':
                    raise AssertionError('ICP correction has incorrect parent/child frames')
                corrections[stamp_ns(message)] = decode_transform(message)

            def record_tf(message):
                for value in message.transforms:
                    key = (value.header.frame_id, value.child_frame_id)
                    if key in (('map', 'icp_corrected'), ('icp_corrected', 'pose_corrected')):
                        tf_links.setdefault(stamp_ns(value), {})[key] = decode_transform(value)

            subscriptions = [
                probe.create_subscription(PointCloud2, 'tsdf_pointcloud', record_snapshot, durable),
                probe.create_subscription(TransformStamped, 'icp_transform', record_correction, durable),
                probe.create_subscription(TFMessage, '/tf', record_tf, reliable),
            ]
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

            def service(name):
                future = clients[name].call_async(Trigger.Request())
                spin_until(future.done, name)
                response = future.result()
                if response is None or (name != 'mapper_status' and not response.success):
                    raise RuntimeError(f'{name} failed: {response}')
                return response

            def status():
                nonlocal latest_status
                latest_status = json.loads(service('mapper_status').message)
                return latest_status

            def wait_status(predicate, label):
                while True:
                    ensure_running(label)
                    current = status()
                    if predicate(current):
                        return current
                    rclpy.spin_once(probe, timeout_sec=.01)

            def check(name, condition):
                summary['checks'][name] = bool(condition)
                if not condition:
                    raise AssertionError(name)

            def raw_pose(stamp, translation):
                message = TransformStamped()
                message.header.frame_id, message.child_frame_id = 'map', 'lidar'
                message.header.stamp.sec = int(stamp)
                (message.transform.translation.x, message.transform.translation.y,
                 message.transform.translation.z) = translation
                message.transform.rotation.w = 1.
                return message

            def cloud(stamp, points):
                message = PointCloud2()
                message.header.frame_id, message.header.stamp.sec = 'lidar', int(stamp)
                message.width, message.height = len(points), 1
                message.point_step, message.row_step = 12, len(points) * 12
                message.is_dense, message.is_bigendian = True, False
                message.fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                                  for i, name in enumerate(('x', 'y', 'z'))]
                message.data = b''.join(struct.pack('<fff', *point) for point in points)
                return message

            def analytic_plane():
                message = Layer()
                message.voxel_size, message.voxels_per_side = resolution, side
                message.layer_type, message.action = 'tsdf', Layer.ACTION_RESET
                blocks = []
                for bx in range(3, 7):
                    for by in range(-3, 3):
                        for bz in range(-3, 3):
                            block, data = Block(), []
                            block.x_index, block.y_index, block.z_index = bx, by, bz
                            for _z in range(side):
                                for _y in range(side):
                                    for x in range(side):
                                        world_x = (bx * side + x + .5) * resolution
                                        data.extend((float_word(2. - world_x), float_word(1.), 0xA0A0A0FF))
                            block.data = data
                            blocks.append(block)
                message.blocks = blocks
                return message

            def stamped_outputs(stamp):
                key = int(stamp) * 1_000_000_000
                spin_until(lambda: key in corrections and key in tf_links and len(tf_links[key]) == 2,
                           f'ICP correction and both TF links at {stamp}s')
                return corrections[key], tf_links[key]

            def check_tf(name, correction, links, translation):
                expected_raw = {'translation': list(translation), 'quaternion_xyzw': [0., 0., 0., 1.]}
                check(name, same_transform(correction, links[('map', 'icp_corrected')]) and
                      same_transform(expected_raw, links[('icp_corrected', 'pose_corrected')]))

            spin_until(lambda: all(client.service_is_ready() for client in clients.values()), 'services discovery')
            spin_until(lambda: all(publisher.get_subscription_count() > 0
                                  for publisher in (ordinary, freespace, transforms, layers)), 'inputs discovery')
            spin_until(lambda: all(probe.count_publishers(topic) > 0
                                  for topic in ('tsdf_pointcloud', 'icp_transform', '/tf')), 'outputs discovery')
            initial = status()
            check('icp_enabled_in_transform_topic_mode', initial['enable_icp'] is True and
                  initial['use_tf_transforms'] is False and initial['icp_runs'] == 0)
            imported = analytic_plane()
            layers.publish(imported)
            summary['observations']['imported_plane'] = {
                'blocks': len(imported.blocks), 'voxels': len(imported.blocks) * side ** 3,
                'distance_definition': '2 - x',
            }
            wait_status(lambda current: current['received_layers'] == 1, 'analytic TSDF plane import')
            first_raw = (.12, 0., 0.)
            transforms.publish(raw_pose(10, first_raw))
            wait_status(lambda current: current['received_transforms'] == 1, 'first raw pose')
            plane_points = [(2., y * .1, z * .1) for y in range(-8, 9) for z in range(-8, 9)]
            ordinary.publish(cloud(10, plane_points))
            aligned = wait_status(lambda current: current['integrated_scans'] == 1, 'ICP plane integration')
            summary['observations']['after_plane_cloud'] = aligned
            correction, links = stamped_outputs(10)
            check_tf('first_tf_tree_matches_correction_and_raw_pose', correction, links, first_raw)
            before = sum(abs(point[0] + first_raw[0] - 2.) for point in plane_points) / len(plane_points)
            corrected_points = [apply(correction, tuple(point[i] + first_raw[i] for i in range(3)))
                                for point in plane_points]
            after = sum(abs(point[0] - 2.) for point in corrected_points) / len(corrected_points)
            summary['observations']['alignment'] = {
                'input_stamp_ns': 10_000_000_000, 'points': len(plane_points),
                'raw_mean_plane_error_m': before, 'corrected_mean_plane_error_m': after,
                'correction': correction,
            }
            check('icp_successful_batches_reduce_actual_plane_error', aligned['icp_runs'] == 1 and
                  aligned['icp_last_successful_batches'] > 0 and aligned['icp_successful_batches'] > 0 and
                  after < before * .5 and correction['translation'][0] < -.06)
            check('correction_does_not_introduce_roll_or_pitch',
                  abs(correction['quaternion_xyzw'][0]) < 1e-6 and abs(correction['quaternion_xyzw'][1]) < 1e-6)
            first_map = spin_until(lambda: snapshots.get(10_000_000_000), 'plane integration TSDF snapshot')
            minimum_x = min(point[0] for point, _ in first_map.values())
            summary['observations']['integrated_plane_map'] = {'observed_voxels': len(first_map), 'minimum_x': minimum_x}
            check('plane_cloud_actually_integrates_new_voxels', aligned['accepted_points'] == len(plane_points) and
                  minimum_x < 1.0)

            # Reset clears map evidence but keeps accumulated ICP correction,
            # matching the original mapper. A new empty-map scan cannot match.
            service('reset_map')
            reset = status()
            summary['observations']['after_reset'] = reset
            check('reset_clears_map_and_keeps_icp_history', reset['allocated_blocks'] == 0 and
                  reset['integrated_scans'] == 0 and reset['icp_runs'] == 1 and
                  reset['icp_successful_batches'] == aligned['icp_successful_batches'])
            second_raw = (.145, .025, .025)
            transforms.publish(raw_pose(20, second_raw))
            wait_status(lambda current: current['received_transforms'] == 2, 'second raw pose')
            # The pinned upstream RayCaster tests abs(direction) < 0.0 before
            # division, so exactly zero components yield infinities/NaNs and
            # can repeat one voxel. Keep the nonaxial case for comparison;
            # --axis-aligned verifies the minimal build-time core correction.
            free_local = (0., 1., 0.) if args.axis_aligned else (.03, 1., .04)
            summary['ray_case'] = 'axis_aligned' if args.axis_aligned else 'nonaxial'
            freespace.publish(cloud(20, [free_local]))
            final = wait_status(lambda current: current['freespace']['integrated_clouds'] == 1,
                                'empty-map freespace integration despite zero ICP matches')
            summary['observations']['after_empty_map_freespace'] = final
            free_correction, free_links = stamped_outputs(20)
            check_tf('second_tf_tree_uses_second_input_stamp_and_pose', free_correction, free_links, second_raw)
            check('zero_match_scan_retains_accumulated_correction', final['icp_runs'] == 2 and
                  final['icp_last_successful_batches'] == 0 and
                  final['icp_successful_batches'] == aligned['icp_successful_batches'] and
                  same_transform(correction, free_correction))
            free_map = spin_until(lambda: snapshots.get(20_000_000_000), 'corrected freespace TSDF snapshot')
            corrected_origin = apply(free_correction, second_raw)
            corrected_endpoint = apply(free_correction, tuple(second_raw[i] + free_local[i] for i in range(3)))
            local_interior = tuple(.4 * value for value in free_local)
            corrected_interior = apply(free_correction, tuple(second_raw[i] + local_interior[i] for i in range(3)))
            raw_interior = tuple(second_raw[i] + local_interior[i] for i in range(3))
            corrected_cell, raw_cell = cell(corrected_interior), cell(raw_interior)
            distance = free_map.get(corrected_cell)
            summary['observations']['corrected_freespace_geometry'] = {
                'input_stamp_ns': 20_000_000_000, 'correction': free_correction,
                'sensor_endpoint': list(free_local),
                'corrected_sensor_origin': list(corrected_origin), 'corrected_endpoint': list(corrected_endpoint),
                'corrected_interior_cell': list(corrected_cell), 'raw_interior_cell': list(raw_cell),
                'corrected_cell_distance_m': None if distance is None else distance[1],
                'raw_cell_observed': raw_cell in free_map, 'observed_voxels': len(free_map),
                'minimum_distance_m': min(value for _, value in free_map.values()),
            }
            check('corrected_pose_is_used_for_actual_freespace_integration', corrected_cell != raw_cell and
                  distance is not None and raw_cell not in free_map and
                  len(free_map) >= 10 and abs(distance[1] - truncation) < 2e-5 and
                  all(value > .1 for _, value in free_map.values()))
            check('zero_match_freespace_is_not_rejected', final['integrated_scans'] == 0 and
                  final['freespace']['accepted_points'] == 1 and final['freespace']['pending_clouds'] == 0 and
                  final['freespace']['rejected_clouds'] == 0 and final['rejected_transforms'] == 0)
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
