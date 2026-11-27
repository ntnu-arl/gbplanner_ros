#!/usr/bin/env python3
"""Check scan-driven TSDF/ESDF pruning and existing mesh removal without timers."""
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
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_srvs.srv import Trigger
    from visualization_msgs.msg import Marker
    from voxblox_msgs.msg import Block, Layer, Mesh

    summary = {'status': 'failed', 'scope': 'native ROS2 scan-driven block pruning',
               'checks': {}, 'observations': {}, 'events': []}
    process = probe = log = None
    started = time.monotonic()
    deadline = started + 25.
    log_path = args.output.with_suffix('.log')
    latest_status = {}
    layers = {'tsdf': {}, 'esdf': {}}
    layer_messages = {'tsdf': deque(maxlen=16), 'esdf': deque(maxlen=16)}
    displayed, tombstones = {}, set()
    markers = deque(maxlen=16)
    cloud_topics = ('surface_pointcloud', 'tsdf_pointcloud', 'esdf_pointcloud')
    clouds = {topic: deque(maxlen=16) for topic in cloud_topics}
    resolution, side, edge, radius = .25, 4, 1., 1.
    quantum = 2. * edge / 65535.
    mesh_service_calls = 0

    def indices(values):
        return [list(value) for value in sorted(values)]

    def bounds(points):
        if not points:
            return None
        return {'min': [min(point[axis] for point in points) for axis in range(3)],
                'max': [max(point[axis] for point in points) for axis in range(3)]}

    def within_radius(index, origin):
        return sum((index[axis] * edge - origin[axis]) ** 2 for axis in range(3)) <= radius ** 2 + 1e-9

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

    try:
        with tempfile.TemporaryDirectory(prefix='gbplanner-block-pruning-') as temporary:
            params = Path(temporary) / 'params.yaml'
            params.write_text(json.dumps({'voxblox_mapper': {'ros__parameters': {
                'frame_id': 'map', 'sensor_frame': 'lidar', 'input_sensors': ['main'],
                'input.main.topic': 'points', 'input.main.frame': 'lidar',
                'use_tf_transforms': False, 'use_freespace_pointcloud': True,
                'freespace.sensor_frames': ['lidar'], 'min_time_between_msgs_sec': 0.,
                'map.storage': 'sparse', 'map.resolution': resolution,
                'tsdf.voxels_per_side': side, 'tsdf.method': 'simple',
                'tsdf.truncation_distance': .25, 'tsdf.use_weight_dropoff': False,
                'esdf.enabled': True, 'color_mode': 'colors',
                'max_block_distance_from_body': radius,
                'update_mesh_every_n_sec': 0., 'publish_map_every_n_sec': 0.,
                'publish_tsdf_map': True, 'publish_esdf_map': True,
                'publish_pointclouds_on_update': True,
            }}}))
            log = log_path.open('w')
            rclpy.init(args=[])
            probe = Node('voxblox_block_pruning_probe')
            reliable = QoSProfile(depth=40, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.VOLATILE)
            durable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)

            def consume_layer(kind, message):
                if (message.layer_type != kind or message.voxels_per_side != side or
                        not math.isclose(message.voxel_size, resolution, abs_tol=1e-9)):
                    raise AssertionError(f'{kind} Layer geometry changed')
                if message.action == Layer.ACTION_RESET:
                    layers[kind].clear()
                elif message.action != Layer.ACTION_UPDATE:
                    raise AssertionError('unexpected outgoing Layer action')
                keys = []
                for block in message.blocks:
                    key = (int(block.x_index), int(block.y_index), int(block.z_index))
                    layers[kind][key] = block
                    keys.append(key)
                layer_messages[kind].append(message)
                summary['events'].append({'topic': kind + '_map_out', 'action': int(message.action),
                                          'blocks': indices(keys), 'receiver_blocks': indices(layers[kind])})

            def consume_mesh(message):
                if message.header.frame_id != 'map' or not math.isclose(message.block_edge_length, edge):
                    raise AssertionError('invalid Mesh frame or block edge length')
                updated, removed = [], []
                for block in message.mesh_blocks:
                    key = tuple(int(value) for value in block.index)
                    size = len(block.x)
                    if size % 3 or any(len(values) != size for values in
                                       (block.y, block.z, block.r, block.g, block.b)):
                        raise AssertionError('malformed MeshBlock payload')
                    if not size:
                        displayed.pop(key, None)
                        tombstones.add(key)
                        removed.append(key)
                        continue
                    points = [tuple((2. * int(coordinate[i]) / 65535. + key[axis]) * edge
                                    for axis, coordinate in enumerate((block.x, block.y, block.z)))
                              for i in range(size)]
                    displayed[key] = points
                    updated.append({'index': list(key), 'triangles': size // 3, 'bounds': bounds(points)})
                summary['events'].append({'topic': 'mesh', 'updated': updated, 'removed': indices(removed)})

            def consume_marker(message):
                markers.append(message)
                points = [(float(point.x), float(point.y), float(point.z)) for point in message.points]
                summary['events'].append({'topic': 'surface_mesh', 'action': int(message.action),
                                          'triangles': len(points) // 3, 'bounds': bounds(points)})

            def consume_cloud(topic, message):
                fields = {field.name: field for field in message.fields}
                if (message.header.frame_id != 'map' or not {'x', 'y', 'z'} <= set(fields) or
                        message.height != 1 or message.row_step != message.width * message.point_step or
                        len(message.data) != message.row_step):
                    raise AssertionError(f'malformed {topic}')
                for axis in ('x', 'y', 'z'):
                    field = fields[axis]
                    if field.datatype != PointField.FLOAT32 or field.count != 1 or field.offset + 4 > message.point_step:
                        raise AssertionError(f'invalid coordinate field in {topic}')
                endian = '>' if message.is_bigendian else '<'
                points = [tuple(struct.unpack_from(endian + 'f', message.data,
                                                    i * message.point_step + fields[axis].offset)[0]
                                for axis in ('x', 'y', 'z')) for i in range(message.width)]
                if any(not math.isfinite(value) for point in points for value in point):
                    raise AssertionError(f'nonfinite coordinates in {topic}')
                keys = {tuple(int(math.floor(value / edge)) for value in point) for point in points}
                record = {'stamp_sec': int(message.header.stamp.sec), 'points': points, 'blocks': keys}
                clouds[topic].append(record)
                summary['events'].append({'topic': topic, 'stamp_sec': record['stamp_sec'],
                                          'points': len(points), 'blocks': indices(keys), 'bounds': bounds(points)})

            subscriptions = [probe.create_subscription(Layer, kind + '_map_out',
                              lambda message, kind=kind: consume_layer(kind, message), reliable)
                             for kind in layers]
            subscriptions.extend([
                probe.create_subscription(Mesh, 'mesh', consume_mesh, reliable),
                probe.create_subscription(Marker, 'surface_mesh', consume_marker, durable),
            ])
            subscriptions.extend(probe.create_subscription(PointCloud2, topic,
                                 lambda message, topic=topic: consume_cloud(topic, message), durable)
                                 for topic in cloud_topics)
            layer_input = probe.create_publisher(Layer, 'tsdf_map_in', reliable)
            ordinary = probe.create_publisher(PointCloud2, 'points', reliable)
            freespace = probe.create_publisher(PointCloud2, 'freespace_pointcloud', reliable)
            transforms = probe.create_publisher(TransformStamped, 'transform', reliable)
            clients = {name: probe.create_client(Trigger, name) for name in ('mapper_status', 'generate_mesh')}
            process = subprocess.Popen(
                ['ros2', 'run', 'gbplanner_voxblox', 'voxblox_mapper_node', '--ros-args', '--params-file', str(params)],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=dict(os.environ))

            def spin_until(predicate, label):
                while True:
                    if process.poll() is not None:
                        raise RuntimeError(f'mapper exited ({process.returncode}); see {log_path}')
                    if time.monotonic() >= deadline:
                        raise TimeoutError(label)
                    value = predicate()
                    if value:
                        return value
                    rclpy.spin_once(probe, timeout_sec=.01)

            def service(name):
                nonlocal mesh_service_calls
                if name == 'generate_mesh':
                    mesh_service_calls += 1
                future = clients[name].call_async(Trigger.Request())
                spin_until(future.done, name + ' response')
                response = future.result()
                if response is None or (name != 'mapper_status' and not response.success):
                    raise RuntimeError(f'{name} failed: {None if response is None else response.message}')
                return json.loads(response.message)

            def status():
                nonlocal latest_status
                latest_status = service('mapper_status')
                return latest_status

            def wait_status(predicate, label):
                def poll():
                    current = status()
                    return current if predicate(current) else None
                return spin_until(poll, label)

            def check(name, condition):
                summary['checks'][name] = bool(condition)
                if not condition:
                    raise AssertionError(name)

            def plane_layer():
                message = Layer(voxel_size=resolution, voxels_per_side=side,
                                layer_type='tsdf', action=Layer.ACTION_RESET)
                for x_block in range(4):
                    block = Block(x_index=x_block, y_index=0, z_index=0)
                    words = []
                    for z in range(side):
                        distance = (z + .5) * resolution - .5
                        distance_word = struct.unpack('<I', struct.pack('<f', distance))[0]
                        for _ in range(side * side):
                            words.extend((distance_word, 0x3F800000, 0x14283CFF))
                    block.data = words
                    message.blocks.append(block)
                return message

            def scan(stamp, origin, point, publisher):
                transform = TransformStamped()
                transform.header.frame_id, transform.child_frame_id = 'map', 'tracking'
                transform.header.stamp.sec = stamp
                (transform.transform.translation.x, transform.transform.translation.y,
                 transform.transform.translation.z) = origin
                transform.transform.rotation.w = 1.
                before = status()['received_transforms']
                transforms.publish(transform)
                wait_status(lambda current: current['received_transforms'] == before + 1,
                            'exact acquisition transform')
                message = PointCloud2()
                message.header.frame_id, message.header.stamp.sec = 'lidar', stamp
                message.height, message.width = 1, 1
                message.point_step, message.row_step = 12, 12
                message.is_dense = True
                message.fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                                  for i, name in enumerate(('x', 'y', 'z'))]
                message.data = struct.pack('<fff', *point)
                summary['events'].append({'input': publisher.topic_name, 'stamp_sec': stamp,
                                          'raw_origin': list(origin), 'sensor_point': list(point)})
                publisher.publish(message)

            def wait_pruned_outputs(stamp, origin, removed):
                for kind in layers:
                    spin_until(lambda kind=kind: layer_messages[kind] and
                               layer_messages[kind][-1].action == Layer.ACTION_RESET and
                               not (set(layers[kind]) & removed) and layers[kind] and
                               all(within_radius(key, origin) for key in layers[kind]),
                               kind + ' RESET removes distant blocks from receiver')
                products = {}
                for topic in cloud_topics:
                    record = spin_until(lambda topic=topic: next((record for record in reversed(clouds[topic])
                                              if record['stamp_sec'] == stamp), None), topic + ' updated after pruning')
                    check(f'{stamp}_{topic}_contains_no_pruned_blocks',
                          not (record['blocks'] & removed) and
                          all(within_radius(key, origin) for key in record['blocks']))
                    products[topic] = {'points': len(record['points']), 'blocks': indices(record['blocks'])}
                return products

            def remember(name, current, products):
                summary['observations'][name] = {
                    'mapper': current, 'layers': {kind: indices(values) for kind, values in layers.items()},
                    'displayed_mesh': [{'index': list(key), 'triangles': len(points) // 3, 'bounds': bounds(points)}
                                       for key, points in sorted(displayed.items())],
                    'mesh_tombstones': indices(tombstones), 'clouds': products,
                }

            spin_until(lambda: all(client.service_is_ready() for client in clients.values()), 'service discovery')
            spin_until(lambda: all(publisher.get_subscription_count() > 0
                                   for publisher in (layer_input, ordinary, freespace, transforms)), 'input discovery')
            spin_until(lambda: all(probe.count_publishers(subscription.topic_name) > 0
                                   for subscription in subscriptions), 'output discovery')
            initial = status()
            check('configured_radius_and_disabled_mesh_timer',
                  initial['max_block_distance_from_body'] == radius and initial['update_mesh_every_n_sec'] == 0.)
            layer_input.publish(plane_layer())
            wait_status(lambda current: current['received_layers'] == 1, 'plane Layer import')
            initial_blocks = {(x, 0, 0) for x in range(4)}
            spin_until(lambda: all(set(values) == initial_blocks for values in layers.values()), 'four imported layer blocks')
            generation = service('generate_mesh')
            spin_until(lambda: set(displayed) == initial_blocks and any(marker.action == Marker.ADD for marker in markers),
                       'initial four-block mesh')
            initial_neighbor_max = max(point[0] for point in displayed[(1, 0, 0)])
            check('fixture_has_cross_block_triangles_before_pruning', initial_neighbor_max > 2.12)
            summary['observations']['initial_mesh'] = {'generation': generation,
                'neighbor_block_max_x_m': initial_neighbor_max, 'blocks': indices(displayed)}

            for messages in layer_messages.values():
                messages.clear()
            markers.clear()
            tombstones.clear()
            scan(21, (0., 0., 0.), (.4, .2, .2), ordinary)
            first = wait_status(lambda current: current['integrated_scans'] == 1, 'ordinary scan and first pruning')
            first_removed = {(2, 0, 0), (3, 0, 0)}
            products = wait_pruned_outputs(21, (0., 0., 0.), first_removed)
            spin_until(lambda: first_removed <= tombstones and not (set(displayed) & first_removed) and
                       (1, 0, 0) in displayed and max(point[0] for point in displayed[(1, 0, 0)]) <= 1.875 + quantum,
                       'removed mesh tombstones and retained boundary remeshing')
            marker = spin_until(lambda: next((marker for marker in reversed(markers)
                                if marker.action == Marker.ADD and marker.points and
                                max(point.x for point in marker.points) <= 1.875 + 1e-6), None),
                                'complete Marker drops removed and cross-boundary triangles')
            marker_points = [(float(point.x), float(point.y), float(point.z)) for point in marker.points]
            native_points = [point for points in displayed.values() for point in points]
            # Shared boundary vertices have slightly different compressed
            # coordinates in adjacent blocks. Lexicographic sorting can reorder
            # their y values; match points one-to-one within the wire quantum.
            unmatched = list(native_points)
            marker_error = 0. if len(marker_points) == len(unmatched) else math.inf
            if math.isfinite(marker_error):
                for point in marker_points:
                    candidate = min(range(len(unmatched)), key=lambda i:
                                    sum((a - b) ** 2 for a, b in zip(point, unmatched[i])))
                    other = unmatched.pop(candidate)
                    marker_error = max(marker_error, max(abs(a - b) for a, b in zip(point, other)))
            summary['observations']['marker_coordinate_error_m'] = marker_error
            check('exact_radius_boundary_block_is_retained', all((1, 0, 0) in values for values in layers.values()))
            check('existing_boundary_mesh_and_full_marker_agree', marker_error <= quantum + 1e-6)
            check('ordinary_scan_prunes_all_three_products', all(first[name] >= 2 for name in
                  ('pruned_tsdf_blocks', 'pruned_esdf_blocks', 'pruned_mesh_blocks')))
            check('first_scan_succeeds_without_freespace_or_timer', first['freespace']['integrated_clouds'] == 0 and
                  first['rejected_scans'] == 0 and mesh_service_calls == 1)
            remember('ordinary_scan_pruning', first, products)

            for messages in layer_messages.values():
                messages.clear()
            markers.clear()
            tombstones.clear()
            scan(22, (3., 0., 0.), (.6, .2, .2), freespace)
            second = wait_status(lambda current: current['freespace']['integrated_clouds'] == 1,
                                 'moving freespace scan and second pruning')
            old_blocks = {(0, 0, 0), (1, 0, 0)}
            products = wait_pruned_outputs(22, (3., 0., 0.), old_blocks)
            spin_until(lambda: old_blocks <= tombstones and not displayed, 'old cached mesh removed after moving')
            spin_until(lambda: any(marker.action == Marker.DELETE for marker in markers), 'empty full Marker deletion')
            latest_tsdf = next(record for record in reversed(clouds['tsdf_pointcloud']) if record['stamp_sec'] == 22)
            check('freespace_observes_new_block_after_old_map_pruned', (3, 0, 0) in layers['tsdf'] and
                  (3, 0, 0) in latest_tsdf['blocks'] and len(latest_tsdf['points']) >= 2)
            check('freespace_pruning_counters_advance', all(second[name] >= first[name] + 2 for name in
                  ('pruned_tsdf_blocks', 'pruned_esdf_blocks', 'pruned_mesh_blocks')))
            check('both_inputs_finish_without_rejection_or_extra_mesh_generation',
                  second['integrated_scans'] == 1 and second['freespace']['integrated_clouds'] == 1 and
                  second['rejected_scans'] == 0 and second['freespace']['rejected_clouds'] == 0 and
                  second['received_layers'] == 1 and second['received_transforms'] == 2 and
                  second['rejected_transforms'] == 0 and mesh_service_calls == 1)
            remember('moving_freespace_pruning', second, products)
            summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
    finally:
        stop_process()
        summary['mapper_exit_code'] = None if process is None else process.returncode
        if process is not None:
            summary['checks'].setdefault('mapper_exited_cleanly', process.returncode == 0)
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if log is not None:
            log.close()
        summary['last_mapper_status'] = latest_status
        summary['final_receiver_layers'] = {kind: indices(values) for kind, values in layers.items()}
        summary['final_mesh_blocks'] = indices(displayed)
        summary['final_mesh_tombstones'] = indices(tombstones)
        summary['generate_mesh_service_calls'] = mesh_service_calls
        summary['elapsed_seconds'] = round(time.monotonic() - started, 3)
        if not all(summary['checks'].values()):
            summary['status'] = 'failed'
        args.output.write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
