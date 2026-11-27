#!/usr/bin/env python3
"""Exercise TSDF/ESDF Layer transport through a real ROS 2 mapper process."""
import argparse
from collections import deque
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
    os.environ.setdefault('ROS_DOMAIN_ID', str(110 + os.getpid() % 20))

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_srvs.srv import Trigger
    from tf2_ros import StaticTransformBroadcaster
    from voxblox_msgs.msg import Block, Layer

    summary = {'status': 'failed', 'scope': 'native ROS2 TSDF/ESDF Layer transport',
               'checks': {}, 'observations': {}}
    process = probe = log = None
    started = time.monotonic()
    deadline = started + 35.
    latest_status = {}

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
        with tempfile.TemporaryDirectory(prefix='gbplanner-layer-transport-') as temporary:
            params = Path(temporary) / 'params.yaml'
            params.write_text(json.dumps({'voxblox_mapper': {'ros__parameters': {
                'frame_id': 'map', 'sensor_frame': 'lidar',
                'publish_tsdf_map': True, 'publish_esdf_map': True,
                'publish_map_every_n_sec': .1,
                'map.storage': 'sparse', 'map.resolution': .2,
                'tsdf.voxels_per_side': 2, 'tsdf.method': 'simple',
                'esdf.enabled': True, 'color_mode': 'colors',
                'update_mesh_every_n_sec': 0.,
            }}}))
            log_path = args.output.with_suffix('.log')
            log = log_path.open('w')
            rclpy.init(args=[])
            probe = Node('voxblox_layer_transport_probe')
            layer_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.VOLATILE)
            cloud_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
            received = {name: deque(maxlen=64) for name in
                        ('tsdf', 'esdf', 'late_tsdf', 'tsdf_cloud', 'esdf_cloud', 'voxel_map')}
            subscriptions = [
                probe.create_subscription(Layer, 'tsdf_map_out', received['tsdf'].append, layer_qos),
                probe.create_subscription(Layer, 'esdf_map_out', received['esdf'].append, layer_qos),
                probe.create_subscription(PointCloud2, 'tsdf_voxels', received['tsdf_cloud'].append, cloud_qos),
                probe.create_subscription(PointCloud2, 'esdf_voxels', received['esdf_cloud'].append, cloud_qos),
                probe.create_subscription(PointCloud2, 'voxel_map', received['voxel_map'].append, cloud_qos),
            ]
            tsdf_input = probe.create_publisher(Layer, 'tsdf_map_in', layer_qos)
            esdf_input = probe.create_publisher(Layer, 'esdf_map_in', layer_qos)
            points = probe.create_publisher(PointCloud2, 'points', layer_qos)
            clients = {name: probe.create_client(Trigger, name)
                       for name in ('mapper_status', 'publish_map', 'reset_map')}
            broadcaster = StaticTransformBroadcaster(probe)
            process = subprocess.Popen(
                ['ros2', 'run', 'gbplanner_voxblox', 'voxblox_mapper_node',
                 '--ros-args', '--params-file', str(params)],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                env=dict(os.environ))

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
                    rclpy.spin_once(probe, timeout_sec=.02)

            def service(name):
                future = clients[name].call_async(Trigger.Request())
                spin_until(future.done, f'{name} response')
                response = future.result()
                if response is None:
                    raise RuntimeError(f'empty {name} response')
                if name != 'mapper_status' and not response.success:
                    raise RuntimeError(f'{name}: {response.message}')
                return response

            def status():
                nonlocal latest_status
                latest_status = json.loads(service('mapper_status').message)
                return latest_status

            def wait_status(predicate, label):
                def poll():
                    current = status()
                    return current if predicate(current) else None
                return spin_until(poll, label)

            def check(name, predicate):
                summary['checks'][name] = bool(predicate)
                if not predicate:
                    raise AssertionError(name)

            def words(distance, weight=None, color=(0, 0, 0, 255)):
                bits = struct.unpack('<I', struct.pack('<f', distance))[0]
                if weight is None:
                    return [bits, 1]  # ESDF observed, zero parent/other flags.
                weight_bits = struct.unpack('<I', struct.pack('<f', weight))[0]
                rgba = color[0] << 24 | color[1] << 16 | color[2] << 8 | color[3]
                return [bits, weight_bits, rgba]

            def block(index, data):
                message = Block()
                message.x_index, message.y_index, message.z_index = index
                message.data = data * 8
                return message

            def layer(kind, action, blocks):
                message = Layer()
                message.voxel_size, message.voxels_per_side = .2, 2
                message.layer_type, message.action, message.blocks = kind, action, blocks
                return message

            def block_index(value):
                return value.x_index, value.y_index, value.z_index

            def matches(message, expected, action=None):
                return (action is None or message.action == action) and {
                    block_index(value): list(value.data) for value in message.blocks
                } == expected

            def wait_layer(kind, expected, action=None):
                return spin_until(lambda: next((m for m in received[kind]
                                                if matches(m, expected, action)), None),
                                  f'{kind} Layer contents')

            def cloud_rows(message):
                fields = {field.name: field.offset for field in message.fields}
                endian = '>' if message.is_bigendian else '<'
                result = []
                for row in range(message.height):
                    for col in range(message.width):
                        offset = row * message.row_step + col * message.point_step
                        # This probe reads floating fields, excluding occupancy
                        # which is INT8 in the mapper's combined voxel cloud.
                        result.append({name: struct.unpack_from(endian + 'f', message.data,
                                                               offset + fields[name])[0]
                                       for name in ('x', 'y', 'z', 'distance', 'weight')
                                       if name in fields})
                return result

            def wait_cloud(kind, expected):
                # Every synthetic block has eight observed voxels. The index
                # comes from spatial point coordinates, independent of payload order.
                def find():
                    for message in reversed(received[kind]):
                        rows = cloud_rows(message)
                        if len(rows) != 8 * len(expected):
                            continue
                        counts = {index: 0 for index in expected}
                        good = True
                        for row in rows:
                            index = tuple(math.floor(row[axis] / .4 + 1e-6)
                                          for axis in ('x', 'y', 'z'))
                            target = expected.get(index)
                            if target is None or not math.isclose(row['distance'], target[0], abs_tol=1e-6):
                                good = False
                                break
                            if len(target) > 1 and not math.isclose(row['weight'], target[1], abs_tol=1e-6):
                                good = False
                                break
                            counts[index] += 1
                        if good and all(count == 8 for count in counts.values()):
                            return rows
                    return None
                return spin_until(find, f'{kind} numeric distance cloud')

            def send(publisher, message):
                previous = status()['received_layers']
                for queue in received.values():
                    queue.clear()
                publisher.publish(message)
                return wait_status(lambda current: current['received_layers'] == previous + 1,
                                   'Layer reception')

            spin_until(lambda: all(c.service_is_ready() for c in clients.values()),
                       'mapper service discovery')
            spin_until(lambda: all(p.get_subscription_count() > 0
                                  for p in (tsdf_input, esdf_input, points)),
                       'mapper input subscription discovery')
            spin_until(lambda: all(probe.count_publishers(s.topic_name) > 0 for s in subscriptions),
                       'mapper output publication discovery')

            main_index, second_index, esdf_index = (1, 0, 0), (3, 0, 0), (-2, 0, 0)
            initial_data = words(.125, 1., (20, 40, 60, 80))
            second_data = words(.25, 2., (10, 20, 30, 255))
            imported = layer('tsdf', Layer.ACTION_RESET, [block(main_index, initial_data)])
            send(tsdf_input, imported)
            wait_layer('tsdf', {main_index: initial_data * 8})
            reset_rows = wait_cloud('tsdf_cloud', {main_index: (.125, 1.)})
            check('tsdf_reset_updates_layer_and_numeric_cloud', len(reset_rows) == 8)

            send(tsdf_input, layer('tsdf', Layer.ACTION_UPDATE, [block(second_index, second_data)]))
            expected_tsdf = {main_index: initial_data * 8, second_index: second_data * 8}
            wait_layer('tsdf', expected_tsdf)
            wait_cloud('tsdf_cloud', {main_index: (.125, 1.), second_index: (.25, 2.)})
            check('tsdf_update_preserves_unlisted_block', True)

            send(tsdf_input, layer('tsdf', Layer.ACTION_MERGE,
                                   [block(main_index, words(.375, 3., (100, 80, 60, 200)))]))
            merged_data = words(.3125, 4., (80, 70, 60, 170))
            expected_tsdf[main_index] = merged_data * 8
            wait_layer('tsdf', expected_tsdf)
            wait_cloud('tsdf_cloud', {main_index: (.3125, 4.), second_index: (.25, 2.)})
            check('tsdf_merge_combines_distance_weight_and_color', True)

            # One malformed RESET checks the transport boundary without repeating
            # the core suite: the existing layer must survive intact.
            before_reject = status()
            malformed = layer('tsdf', Layer.ACTION_RESET, [block(main_index, initial_data)])
            malformed.blocks[0].data = list(malformed.blocks[0].data)[:-1]
            received['tsdf'].clear()
            tsdf_input.publish(malformed)
            rejected = wait_status(lambda current: current['rejected_layers'] ==
                                   before_reject['rejected_layers'] + 1, 'malformed Layer rejection')
            service('publish_map')
            wait_layer('tsdf', expected_tsdf, Layer.ACTION_RESET)
            check('invalid_reset_keeps_map_and_received_counter',
                  rejected['received_layers'] == before_reject['received_layers'])

            esdf_data = words(-.75)
            send(esdf_input, layer('esdf', Layer.ACTION_RESET, [block(esdf_index, esdf_data)]))
            expected_esdf = {esdf_index: esdf_data * 8}
            wait_layer('esdf', expected_esdf)
            esdf_rows = wait_cloud('esdf_cloud', {esdf_index: (-.75,)})
            check('esdf_input_updates_layer_and_numeric_cloud', len(esdf_rows) == 8)

            # A volatile late reader cannot receive old history. Its first full
            # RESET must be generated by the mapper's subscriber detection.
            late = probe.create_subscription(Layer, 'tsdf_map_out',
                                             received['late_tsdf'].append, layer_qos)
            subscriptions.append(late)
            wait_layer('late_tsdf', expected_tsdf, Layer.ACTION_RESET)
            check('late_subscriber_receives_full_reset', True)
            received['tsdf'].clear()
            received['esdf'].clear()
            service('publish_map')
            wait_layer('tsdf', expected_tsdf, Layer.ACTION_RESET)
            wait_layer('esdf', expected_esdf, Layer.ACTION_RESET)
            check('publish_map_forces_both_full_resets', True)

            for queue in received.values():
                queue.clear()
            service('reset_map')
            wait_layer('tsdf', {}, Layer.ACTION_RESET)
            wait_layer('esdf', {}, Layer.ACTION_RESET)
            spin_until(lambda: all(any(m.width * m.height == 0 for m in received[name])
                                   for name in ('tsdf_cloud', 'esdf_cloud', 'voxel_map')),
                       'empty distance and occupancy clouds after reset')
            check('reset_map_clears_layers_and_published_clouds', True)

            # Re-import after reset so the next scan specifically exercises the
            # integrator's pointers after replacement of the destination layer.
            send(tsdf_input, imported)
            wait_cloud('tsdf_cloud', {main_index: (.125, 1.)})
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = 'map', 'lidar'
            transform.header.stamp = probe.get_clock().now().to_msg()
            transform.transform.rotation.w = 1.
            broadcaster.sendTransform(transform)
            cloud = PointCloud2()
            cloud.header.frame_id = 'lidar'
            cloud.header.stamp = probe.get_clock().now().to_msg()
            cloud.height = cloud.width = 1
            cloud.point_step = cloud.row_step = 12
            cloud.is_dense = True
            cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                            for name, offset in (('x', 0), ('y', 4), ('z', 8))]
            cloud.data = struct.pack('<fff', 1.5, .1, .1)
            before_scan = status()
            received['tsdf_cloud'].clear()
            points.publish(cloud)
            final = wait_status(lambda current: current['integrated_scans'] ==
                                before_scan['integrated_scans'] + 1, 'pointcloud integration after Layer import')

            def integrated_rows():
                for message in reversed(received['tsdf_cloud']):
                    rows = cloud_rows(message)
                    if len(rows) > 8 and any(row['x'] > 1.25 and row['weight'] > 0. for row in rows):
                        return rows
                return None

            rows = spin_until(integrated_rows, 'new measured voxels after Layer import')
            check('pointcloud_integrates_after_layer_replacement',
                  final['allocated_blocks'] > 1 and final['rejected_scans'] == before_scan['rejected_scans'])
            summary['observations'] = {
                'tsdf_initial_voxels': len(reset_rows), 'tsdf_merged_distance': .3125,
                'tsdf_merged_weight': 4., 'tsdf_merged_rgba': [80, 70, 60, 170],
                'imported_esdf_distance': -.75, 'imported_esdf_voxels': len(esdf_rows),
                'tsdf_voxels_after_measurement': len(rows),
                'final_mapper_status': final,
            }
            summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
    finally:
        stop_process()
        if log is not None:
            log.close()
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        summary['last_status'] = latest_status
        summary['mapper_exit_code'] = process.returncode if process is not None else None
        summary['elapsed_seconds'] = round(time.monotonic() - started, 3)
        args.output.write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
