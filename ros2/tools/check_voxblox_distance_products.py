#!/usr/bin/env python3
"""Check distance slices and traversability through a native ROS 2 mapper."""
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
    os.environ.setdefault('ROS_DOMAIN_ID', str(130 + os.getpid() % 20))

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_srvs.srv import Trigger
    from voxblox_msgs.msg import Block, Layer

    summary = {'status': 'failed', 'scope': 'native ROS2 distance products',
               'checks': {}, 'observations': {}}
    process = probe = log = None
    latest_status = {}
    started = time.monotonic()
    deadline = started + 20.
    topics = ('tsdf_slice', 'esdf_slice', 'traversable')
    fields = ('x', 'y', 'z', 'intensity')

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
        with tempfile.TemporaryDirectory(prefix='gbplanner-distance-products-') as temporary:
            params = Path(temporary) / 'params.yaml'
            params.write_text(json.dumps({'voxblox_mapper': {'ros__parameters': {
                'frame_id': 'map', 'map.storage': 'dense', 'map.resolution': .2,
                'map.origin': [-1., -1., -1.], 'map.dimensions': [10, 10, 10],
                'tsdf.voxels_per_side': 2, 'esdf.enabled': True,
                'publish_slices': True, 'publish_traversable': True,
                'slice_level': .1, 'traversability_radius': .5,
                'update_mesh_every_n_sec': 0., 'publish_map_every_n_sec': 0.,
            }}}))
            log_path = args.output.with_suffix('.log')
            log = log_path.open('w')
            rclpy.init(args=[])
            probe = Node('voxblox_distance_products_probe')
            input_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.VOLATILE)
            product_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
            received = {topic: deque(maxlen=16) for topic in topics}
            late_received = {topic: deque(maxlen=4) for topic in topics}
            subscriptions = [probe.create_subscription(
                PointCloud2, topic, received[topic].append, product_qos) for topic in topics]
            inputs = {kind: probe.create_publisher(Layer, kind + '_map_in', input_qos)
                      for kind in ('tsdf', 'esdf')}
            clients = {name: probe.create_client(Trigger, name)
                       for name in ('mapper_status', 'publish_pointclouds', 'reset_map')}
            process = subprocess.Popen(
                ['ros2', 'run', 'gbplanner_voxblox', 'voxblox_mapper_node',
                 '--ros-args', '--params-file', str(params)],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                env=dict(os.environ))

            def spin_until(predicate, label):
                while True:
                    if process.poll() is not None:
                        raise RuntimeError(f'mapper exited ({process.returncode}); see {log_path}')
                    if time.monotonic() >= deadline:
                        raise TimeoutError(label)
                    value = predicate()
                    if value:
                        return value
                    rclpy.spin_once(probe, timeout_sec=.02)

            def service(name):
                future = clients[name].call_async(Trigger.Request())
                spin_until(future.done, f'{name} response')
                response = future.result()
                if response is None or (name != 'mapper_status' and not response.success):
                    raise RuntimeError(f'{name}: {response}')
                return response

            def status():
                nonlocal latest_status
                latest_status = json.loads(service('mapper_status').message)
                return latest_status

            def check(name, condition):
                summary['checks'][name] = bool(condition)
                if not condition:
                    raise AssertionError(name)

            def float_word(value):
                return struct.unpack('<I', struct.pack('<f', value))[0]

            # Linear voxel order is x + 2*y + 4*z. Expected centers below are
            # defined directly, independently of the mapper's slice helpers.
            distances = [.25, .5, .75, .25, .5, .75, .25, .75]
            centers = [(x, y, z) for z in (.1, .3) for y in (.1, .3)
                       for x in (4.1, 4.3)]
            unknown_index = 7  # Above the slice; its large distance must not pass traversability.
            rows = [(*center, distance) for center, distance in zip(centers, distances)]
            expected = {
                'tsdf_slice': rows[:4],
                'esdf_slice': rows[:4],
                'traversable': [rows[i] for i in (1, 2, 4, 5)],
            }

            def layer(kind):
                block = Block()
                block.x_index, block.y_index, block.z_index = 10, 0, 0
                data = []
                for index, distance in enumerate(distances):
                    if kind == 'tsdf':
                        data.extend((float_word(distance), float_word(1.), 0x14283CFF))
                    else:
                        # ESDF's low flag bit is observed; parent and other flags are zero.
                        data.extend((float_word(distance), int(index != unknown_index)))
                block.data = data
                message = Layer()
                message.voxel_size, message.voxels_per_side = .2, 2
                message.layer_type, message.action = kind, Layer.ACTION_RESET
                message.blocks = [block]
                return message

            def stamp(message):
                return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec

            def cloud_rows(message):
                offsets = {field.name: field.offset for field in message.fields
                           if field.datatype == PointField.FLOAT32 and field.count == 1}
                if any(name not in offsets for name in fields):
                    raise AssertionError('product requires float32 x/y/z/intensity fields')
                if message.header.frame_id != 'map':
                    raise AssertionError('product frame must be map')
                endian = '>' if message.is_bigendian else '<'
                return [tuple(struct.unpack_from(endian + 'f', message.data,
                              row * message.row_step + column * message.point_step + offsets[name])[0]
                              for name in fields)
                        for row in range(message.height) for column in range(message.width)]

            def matches(actual, target):
                return len(actual) == len(target) and all(
                    all(math.isclose(value, wanted, rel_tol=0., abs_tol=1e-6)
                        for value, wanted in zip(row, expected_row))
                    for row, expected_row in zip(sorted(actual), sorted(target)))

            def wait_products(queues, targets, label, since=0):
                def find():
                    found = {}
                    for topic in topics:
                        message = next((message for message in reversed(queues[topic])
                                        if stamp(message) >= since and
                                        matches(cloud_rows(message), targets[topic])), None)
                        if message is None:
                            return None
                        found[topic] = cloud_rows(message)
                    return found
                return spin_until(find, label)

            spin_until(lambda: all(client.service_is_ready() for client in clients.values()),
                       'mapper service discovery')
            spin_until(lambda: all(publisher.get_subscription_count() > 0
                                  for publisher in inputs.values()), 'Layer input discovery')
            spin_until(lambda: all(probe.count_publishers(topic) > 0 for topic in topics),
                       'distance product discovery')

            for kind in ('tsdf', 'esdf'):
                previous = status()['received_layers']
                inputs[kind].publish(layer(kind))
                spin_until(lambda: status()['received_layers'] == previous + 1,
                           kind + ' Layer reception')
            actual = wait_products(received, expected, 'slice and traversable values')
            check('both_slices_have_four_expected_voxel_centers_and_distances',
                  len(actual['tsdf_slice']) == len(actual['esdf_slice']) == 4)
            check('products_include_voxels_outside_dense_planner_window',
                  all(row[0] > 1. for values in actual.values() for row in values))
            check('traversable_includes_radius_boundary_and_excludes_unknown_voxel',
                  matches(actual['traversable'], expected['traversable']) and
                  any(row[3] == .5 for row in actual['traversable']) and
                  all(not all(math.isclose(value, unknown, rel_tol=0., abs_tol=1e-6)
                              for value, unknown in zip(row[:3], centers[unknown_index]))
                      for row in actual['traversable']))

            forced_since = probe.get_clock().now().nanoseconds
            service('publish_pointclouds')
            wait_products(received, expected, 'forced product publication', since=forced_since)
            check('publish_pointclouds_service_republishes_enabled_products', True)

            # No new map input or publish request follows these subscriptions.
            subscriptions.extend(probe.create_subscription(
                PointCloud2, topic, late_received[topic].append, product_qos) for topic in topics)
            wait_products(late_received, expected, 'latched products for late subscribers')
            check('late_subscribers_receive_all_products_without_new_input',
                  status()['received_layers'] == 2)

            reset_since = probe.get_clock().now().nanoseconds
            service('reset_map')
            empty = {topic: [] for topic in topics}
            wait_products(received, empty, 'empty products after reset', since=reset_since)
            wait_products(late_received, empty, 'empty products for late subscribers', since=reset_since)
            check('reset_map_publishes_empty_products_to_all_subscribers', True)
            summary['observations'] = {
                'fixture_block_index': [10, 0, 0], 'dense_window_x': [-1., 1.],
                'slice_level': .1, 'traversability_radius': .5,
                'unobserved_esdf_center': list(centers[unknown_index]),
                'unobserved_esdf_distance': distances[unknown_index],
                'products_before_reset': actual, 'final_mapper_status': status(),
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
        summary['checks']['mapper_exits_normally'] = process is not None and process.returncode == 0
        if not summary['checks']['mapper_exits_normally']:
            summary['status'] = 'failed'
        summary['elapsed_seconds'] = round(time.monotonic() - started, 3)
        args.output.write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
