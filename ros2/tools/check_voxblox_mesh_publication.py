#!/usr/bin/env python3
"""Check native Mesh, full-layer visualization products and reset removal."""
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


def triangle_area(points):
    total = 0.
    if len(points) % 3:
        raise AssertionError('triangle vertex count is not divisible by three')
    for offset in range(0, len(points), 3):
        a, b, c = points[offset:offset + 3]
        u = [b[i] - a[i] for i in range(3)]
        v = [c[i] - a[i] for i in range(3)]
        cross = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                 u[0] * v[1] - u[1] * v[0])
        total += .5 * math.sqrt(sum(value * value for value in cross))
    return total


def read_ply(path):
    lines = path.read_text().splitlines()
    if lines[:2] != ['ply', 'format ascii 1.0']:
        raise AssertionError('expected original Voxblox ASCII PLY')
    end = lines.index('end_header')
    vertex_count = next(int(line.split()[2]) for line in lines[:end]
                        if line.startswith('element vertex '))
    face_count = next(int(line.split()[2]) for line in lines[:end]
                      if line.startswith('element face '))
    vertices = [tuple(float(value) for value in line.split()[:3])
                for line in lines[end + 1:end + 1 + vertex_count]]
    faces = [list(map(int, line.split()))
             for line in lines[end + 1 + vertex_count:]]
    if len(vertices) != vertex_count or len(faces) != face_count:
        raise AssertionError('PLY payload counts disagree with its header')
    if any(len(face) != 4 or face[0] != 3 or
           any(index < 0 or index >= vertex_count for index in face[1:]) for face in faces):
        raise AssertionError('PLY faces must contain three valid vertex indices')
    triangle_points = [vertices[index] for face in faces for index in face[1:]]
    return {'vertices': vertex_count, 'triangles': face_count,
            'area_m2': triangle_area(triangle_points),
            'plane_max_error_m': max(abs(vertex[2] - .2) for vertex in vertices),
            'bytes': path.stat().st_size}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(150 + os.getpid() % 20))

    import rclpy
    from gbplanner_voxblox.srv import FilePath
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_srvs.srv import Trigger
    from visualization_msgs.msg import Marker, MarkerArray
    from voxblox_msgs.msg import Block, Layer, Mesh

    summary = {'status': 'failed', 'scope': 'native ROS2 mapper Mesh lifecycle; no RViz GUI',
               'checks': {}, 'observations': {}}
    process = probe = log = None
    started = time.monotonic()
    deadline = started + 25.
    last_status = {}
    resolution, side, plane = .2, 4, .2
    index = (-1, 0, 0)
    color = (20, 40, 60)
    received = {'early': deque(maxlen=64), 'late': deque(maxlen=64)}
    displayed = {'early': {}, 'late': {}}
    removed = {'early': set(), 'late': set()}
    marker_messages = deque(maxlen=32)
    product_clouds = {topic: deque(maxlen=32) for topic in
                      ('surface_pointcloud', 'tsdf_pointcloud', 'esdf_pointcloud')}
    occupied_messages = deque(maxlen=32)

    def block_index(block):
        # ROS fixed-size numeric arrays may expose numpy scalar values.
        # Normalize at the message boundary so observations remain JSON numbers.
        return tuple(int(value) for value in block.index)

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
            summary['checks']['mapper_exited_cleanly'] = False

    try:
        with tempfile.TemporaryDirectory(prefix='gbplanner-mesh-publication-') as temporary:
            params = Path(temporary) / 'params.yaml'
            params.write_text(json.dumps({'voxblox_mapper': {'ros__parameters': {
                'frame_id': 'map', 'sensor_frame': 'lidar',
                'map.storage': 'sparse', 'map.resolution': resolution,
                'tsdf.voxels_per_side': side, 'tsdf.method': 'simple',
                'esdf.enabled': True, 'color_mode': 'colors',
                'update_mesh_every_n_sec': .2, 'publish_map_every_n_sec': 0.,
                'publish_pointclouds_on_update': True,
            }}}))
            log_path = args.output.with_suffix('.log')
            log = log_path.open('w')
            rclpy.init(args=[])
            probe = Node('voxblox_mesh_publication_probe')
            volatile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.VOLATILE)
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)

            def consume(reader, message):
                received[reader].append(message)
                # Reproduce the original receiver's per-index update semantics.
                # An empty overall message does not clear an existing map.
                for block in message.mesh_blocks:
                    key = block_index(block)
                    if len(block.x):
                        displayed[reader][key] = block
                    else:
                        removed[reader].add(key)
                        displayed[reader].pop(key, None)

            subscriptions = [
                probe.create_subscription(Mesh, 'mesh', lambda msg: consume('early', msg), volatile),
                probe.create_subscription(Marker, 'surface_mesh', marker_messages.append, durable),
                probe.create_subscription(MarkerArray, 'occupied_nodes', occupied_messages.append, durable),
            ]
            subscriptions.extend(probe.create_subscription(PointCloud2, topic, messages.append, durable)
                                 for topic, messages in product_clouds.items())
            layer_input = probe.create_publisher(Layer, 'tsdf_map_in', volatile)
            clients = {name: probe.create_client(Trigger, name)
                       for name in ('mapper_status', 'generate_mesh', 'reset_map')}
            export = probe.create_client(FilePath, 'export_mesh')
            process = subprocess.Popen(
                ['ros2', 'run', 'gbplanner_voxblox', 'voxblox_mapper_node',
                 '--ros-args', '--params-file', str(params)], stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True, env=dict(os.environ))

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

            def call(client, request, label):
                future = client.call_async(request)
                spin_until(future.done, label)
                response = future.result()
                if response is None:
                    raise RuntimeError(f'empty response: {label}')
                return response

            def service(name):
                response = call(clients[name], Trigger.Request(), name)
                if name != 'mapper_status' and not response.success:
                    raise RuntimeError(f'{name}: {response.message}')
                return response

            def status():
                nonlocal last_status
                last_status = json.loads(service('mapper_status').message)
                return last_status

            def check(name, condition):
                summary['checks'][name] = bool(condition)
                if not condition:
                    raise AssertionError(name)

            def float_word(value):
                return struct.unpack('<I', struct.pack('<f', value))[0]

            def layer(all_positive=False, block_coordinates=index):
                block = Block()
                block.x_index, block.y_index, block.z_index = block_coordinates
                data = []
                # Voxblox linear order: x + side*y + side*side*z. Each isolated
                # observed block has 3x3 crossing cells, giving 18 triangles.
                for z in range(side):
                    for _y in range(side):
                        for _x in range(side):
                            signed_distance = .5 if all_positive else (z + .5) * resolution - plane
                            data.extend((float_word(signed_distance), float_word(1.), 0x14283CFF))
                block.data = data
                message = Layer()
                message.voxel_size, message.voxels_per_side = resolution, side
                message.layer_type, message.action = 'tsdf', Layer.ACTION_RESET
                message.blocks = [block]
                return message

            def import_layer(message):
                previous = status()['received_layers']
                layer_input.publish(message)
                spin_until(lambda: status()['received_layers'] == previous + 1, 'TSDF Layer reception')

            def decode(message):
                if message.header.frame_id != 'map':
                    raise AssertionError('Mesh frame is not map')
                points, colors, blocks = [], [], set()
                for block in message.mesh_blocks:
                    key = block_index(block)
                    size = len(block.x)
                    if len(block.y) != size or len(block.z) != size or size % 3:
                        raise AssertionError('MeshBlock coordinate arrays are malformed')
                    if any(len(values) != size for values in (block.r, block.g, block.b)):
                        raise AssertionError('colored MeshBlock arrays are incomplete')
                    if size:
                        blocks.add(key)
                    for i in range(size):
                        points.append(tuple((2. * int(coordinate[i]) / 65535. + key[axis]) *
                                            float(message.block_edge_length)
                                            for axis, coordinate in enumerate((block.x, block.y, block.z))))
                        colors.append((int(block.r[i]), int(block.g[i]), int(block.b[i])))
                return points, colors, blocks

            def decode_cloud(message, color_field=False):
                expected_fields = {'x', 'y', 'z', 'rgb' if color_field else 'intensity'}
                fields = {field.name: field for field in message.fields}
                if (message.header.frame_id != 'map' or set(fields) != expected_fields or
                        len(fields) != len(message.fields) or message.height != 1 or
                        message.row_step != message.width * message.point_step or
                        len(message.data) != message.row_step):
                    raise AssertionError('visualization cloud header, fields or layout is malformed')
                for field in fields.values():
                    if (field.datatype != PointField.FLOAT32 or field.count != 1 or
                            field.offset + 4 > message.point_step):
                        raise AssertionError('visualization cloud needs scalar float32 fields')
                endian = '>' if message.is_bigendian else '<'
                records = []
                for i in range(message.width):
                    base = i * message.point_step
                    xyz = tuple(struct.unpack_from(endian + 'f', message.data, base + fields[axis].offset)[0]
                                for axis in ('x', 'y', 'z'))
                    if not all(math.isfinite(value) for value in xyz):
                        raise AssertionError('visualization cloud has nonfinite coordinates')
                    if color_field:
                        # RGB is a float32 carrier for packed PCL 0xAARRGGBB;
                        # interpreting those bits as a numeric float loses color.
                        packed = struct.unpack_from(endian + 'I', message.data, base + fields['rgb'].offset)[0]
                        value = ((packed >> 16) & 255, (packed >> 8) & 255, packed & 255)
                    else:
                        value = struct.unpack_from(endian + 'f', message.data, base + fields['intensity'].offset)[0]
                        if not math.isfinite(value):
                            raise AssertionError('distance cloud has nonfinite intensity')
                    records.append((xyz, value))
                return records

            def coordinate_set(points):
                return {tuple(round(value, 6) for value in point) for point in points}

            expected_centers = {
                tuple(round((index[axis] * side + voxel[axis] + .5) * resolution, 6)
                      for axis in range(3))
                for z in range(side) for y in range(side) for x in range(side)
                for voxel in ((x, y, z),)
            }
            near_centers = {point for point in expected_centers if point[2] < .4}

            spin_until(lambda: all(client.service_is_ready() for client in clients.values())
                       and export.service_is_ready(), 'mesh services discovery')
            spin_until(lambda: layer_input.get_subscription_count() > 0, 'Layer input discovery')
            spin_until(lambda: all(probe.count_publishers(topic) > 0
                                  for topic in ('mesh', 'surface_mesh', 'occupied_nodes', *product_clouds)),
                       'mesh and visualization publishers discovery')
            import_layer(layer())
            product_messages = {
                topic: spin_until(lambda messages=messages: next((message for message in reversed(messages)
                                                                 if message.width), None),
                                  f'nonempty {topic}')
                for topic, messages in product_clouds.items()
            }
            tsdf_records = decode_cloud(product_messages['tsdf_pointcloud'])
            surface_records = decode_cloud(product_messages['surface_pointcloud'], color_field=True)
            esdf_records = decode_cloud(product_messages['esdf_pointcloud'])
            check('tsdf_full_layer_has_all_sixty_four_voxels', len(tsdf_records) == 64 and
                  coordinate_set(point for point, _ in tsdf_records) == expected_centers)
            check('tsdf_intensity_is_signed_distance',
                  all(abs(distance - (point[2] - plane)) <= 1e-6 for point, distance in tsdf_records))
            check('surface_has_thirty_two_near_surface_voxels', len(surface_records) == 32 and
                  coordinate_set(point for point, _ in surface_records) == near_centers)
            check('surface_preserves_rgb', all(rgb == color for _, rgb in surface_records))
            check('esdf_distance_cloud_is_nonempty', len(esdf_records) > 0)
            occupied_array = spin_until(
                lambda: next((message for message in reversed(occupied_messages)
                              if any(marker.action == Marker.ADD and marker.points
                                     for marker in message.markers)), None), 'occupied voxel cubes')
            occupied_markers = [marker for marker in occupied_array.markers if marker.action == Marker.ADD]
            occupied_points = [(point.x, point.y, point.z)
                               for marker in occupied_markers for point in marker.points]
            check('occupied_has_thirty_two_voxels_at_default_threshold', len(occupied_points) == 32 and
                  coordinate_set(occupied_points) == near_centers)
            check('occupied_cubes_have_map_frame_resolution_and_colors', all(
                marker.header.frame_id == 'map' and marker.type == Marker.CUBE_LIST and
                all(math.isclose(value, resolution, abs_tol=1e-7)
                    for value in (marker.scale.x, marker.scale.y, marker.scale.z)) and
                len(marker.colors) == len(marker.points) and
                all(0. <= channel <= 1. for rgba in marker.colors for channel in (rgba.r, rgba.g, rgba.b)) and
                all(rgba.a > 0. for rgba in marker.colors)
                for marker in occupied_markers))
            occupied_ids = {(marker.ns, marker.id) for marker in occupied_markers}
            summary['observations']['visualization_products'] = {
                'tsdf_points': len(tsdf_records), 'surface_points': len(surface_records),
                'esdf_points': len(esdf_records), 'occupied_cubes': len(occupied_points),
                'surface_distinct_rgb': [list(rgb) for rgb in sorted({rgb for _, rgb in surface_records})],
                'tsdf_distance_range_m': [min(distance for _, distance in tsdf_records),
                                          max(distance for _, distance in tsdf_records)],
                'point_steps': {topic: int(message.point_step) for topic, message in product_messages.items()},
            }
            marker_messages.clear()
            received['early'].clear()
            generation = json.loads(service('generate_mesh').message)
            mesh_message = spin_until(lambda: next((m for m in reversed(received['early'])
                                                    if any(len(b.x) for b in m.mesh_blocks)), None),
                                      'nonempty uint16 Mesh')
            marker = spin_until(lambda: next((m for m in reversed(marker_messages)
                                              if m.action == Marker.ADD and len(m.points)), None),
                                'surface_mesh triangle Marker')
            points, colors, blocks = decode(mesh_message)
            marker_points = [(p.x, p.y, p.z) for p in marker.points]
            quantum = 2. * float(mesh_message.block_edge_length) / 65535.
            plane_error = max(abs(p[2] - plane) for p in points)
            marker_error = (max(abs(a - b) for p, q in zip(sorted(points), sorted(marker_points))
                                for a, b in zip(p, q)) if len(points) == len(marker_points) else math.inf)
            area, marker_area = triangle_area(points), triangle_area(marker_points)
            check('negative_block_index_and_edge_length', blocks == {index} and
                  math.isclose(mesh_message.block_edge_length, resolution * side, abs_tol=1e-7))
            check('plane_has_eighteen_triangles', len(points) == 54 and generation['triangles'] == 18)
            check('uint16_decoding_matches_plane_and_marker', plane_error <= quantum + 1e-6 and
                  marker_error <= quantum + 1e-6)
            check('mesh_preserves_rgb', all(all(abs(c[i] - color[i]) <= 1 for i in range(3)) for c in colors))
            check('mesh_and_marker_preserve_plane_area', math.isclose(marker_area, .36, abs_tol=1e-6)
                  and abs(area - marker_area) <= 4. * quantum)
            summary['observations']['plane'] = {
                'generation': generation, 'blocks': [list(key) for key in sorted(blocks)],
                'triangles': len(points) // 3, 'quantization_unit_m': quantum,
                'plane_max_error_m': plane_error, 'marker_max_coordinate_error_m': marker_error,
                'mesh_area_m2': area, 'marker_area_m2': marker_area,
                'distinct_rgb': [list(c) for c in sorted(set(colors))],
            }

            ply_path = Path(temporary) / 'plane.ply'
            exported = call(export, FilePath.Request(file_path=str(ply_path)), 'export_mesh')
            check('export_service_succeeded', exported.success and ply_path.is_file())
            ply = read_ply(ply_path)
            check('ply_contains_same_eighteen_triangles', ply['triangles'] == len(points) // 3
                  and math.isclose(ply['area_m2'], marker_area, abs_tol=1e-6)
                  and ply['plane_max_error_m'] <= 1e-6)
            summary['observations']['ply'] = ply

            # New reader uses VOLATILE, so its full map must be actively sent
            # by the mapper's reader-GID watcher, not transient local history.
            before_late = status()
            subscriptions.append(probe.create_subscription(
                Mesh, 'mesh', lambda msg: consume('late', msg), volatile))
            late = spin_until(lambda: next((m for m in received['late']
                                            if {block_index(b) for b in m.mesh_blocks if len(b.x)} == blocks), None),
                              'late reader full mesh without new TSDF data')
            late_points, late_colors, _ = decode(late)
            after_late = status()
            check('late_reader_receives_complete_identical_mesh', sorted(late_points) == sorted(points)
                  and sorted(late_colors) == sorted(colors))
            check('late_resend_needs_no_new_integration',
                  after_late['integrated_scans'] == before_late['integrated_scans'] == 0
                  and after_late['received_layers'] == before_late['received_layers'])

            # The first block's kMesh flag was consumed above. Importing a
            # nonadjacent second block must retain that first surface when the
            # periodic update rebuilds the cleared MeshLayer. Do not call the
            # full-generation service here: that would conceal the regression.
            second_index = (2, 0, 0)
            expected_blocks = blocks | {second_index}
            before_update = status()
            marker_messages.clear()
            update = layer(block_coordinates=second_index)
            update.action = Layer.ACTION_UPDATE
            import_layer(update)
            spin_until(lambda: all(set(displayed[reader]) == expected_blocks
                                   for reader in ('early', 'late')),
                       'periodic mesh preserves both blocks after incremental Layer import')
            combined_marker = spin_until(
                lambda: next((message for message in reversed(marker_messages)
                              if message.action == Marker.ADD and len(message.points) == 108), None),
                'periodic full Marker with thirty-six triangles')
            combined_points = [(point.x, point.y, point.z) for point in combined_marker.points]
            after_update = status()
            check('incremental_layer_preserves_both_blocks_for_both_readers',
                  all(set(displayed[reader]) == expected_blocks for reader in ('early', 'late')))
            check('periodic_marker_contains_both_complete_planes',
                  len(combined_points) == 108 and
                  math.isclose(triangle_area(combined_points), .72, abs_tol=1e-6) and
                  all(abs(point[2] - plane) <= 1e-6 for point in combined_points))
            check('incremental_mesh_update_needs_no_generate_service_or_scan',
                  after_update['mesh_updates'] > before_update['mesh_updates'] and
                  after_update['received_layers'] == before_update['received_layers'] + 1 and
                  after_update['integrated_scans'] == before_update['integrated_scans'] == 0)
            summary['observations']['incremental_layer_mesh'] = {
                'displayed_blocks': {
                    reader: [list(key) for key in sorted(displayed[reader])]
                    for reader in ('early', 'late')},
                'marker_triangles': len(combined_points) // 3,
                'marker_area_m2': triangle_area(combined_points),
                'mesh_updates_before': before_update['mesh_updates'],
                'mesh_updates_after': after_update['mesh_updates'],
            }

            for value in removed.values():
                value.clear()
            marker_messages.clear()
            for messages in product_clouds.values():
                messages.clear()
            occupied_messages.clear()
            service('reset_map')
            spin_until(lambda: all(expected_blocks <= removed[reader] and not displayed[reader]
                                   for reader in ('early', 'late')), 'MeshBlock tombstones after reset')
            spin_until(lambda: any(m.action == Marker.DELETE for m in marker_messages),
                       'legacy Marker deletion after reset')
            check('reset_deletes_previously_displayed_blocks_for_both_readers', True)
            for topic, messages in product_clouds.items():
                empty = spin_until(lambda messages=messages: next((message for message in reversed(messages)
                                                                   if message.width == 0), None),
                                   f'{topic} cleared after reset')
                check(f'reset_clears_{topic}', not decode_cloud(empty, color_field=topic == 'surface_pointcloud'))
            deleted_occupied_ids = spin_until(
                lambda: (occupied_ids & {(marker.ns, marker.id) for message in occupied_messages
                                        for marker in message.markers if marker.action == Marker.DELETE}),
                'occupied_nodes deletion after reset')
            check('reset_deletes_occupied_marker', deleted_occupied_ids == occupied_ids)
            summary['observations']['reset_removed_indices'] = {
                reader: [list(key) for key in sorted(value)] for reader, value in removed.items()}

            import_layer(layer(all_positive=True))
            marker_messages.clear()
            empty_generation = json.loads(service('generate_mesh').message)
            spin_until(lambda: any(m.action == Marker.DELETE for m in marker_messages),
                       'no-surface generation clears legacy Marker')
            check('positive_tsdf_has_no_surface_or_residual_mesh',
                  empty_generation['triangles'] == 0 and all(not value for value in displayed.values()))
            summary['observations']['positive_tsdf_generation'] = empty_generation
            summary['observations']['final_mapper'] = status()
            summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
        summary['observations']['last_mapper_status'] = last_status
    finally:
        stop_process()
        if process is not None:
            summary['mapper_exit_code'] = process.returncode
            summary['checks'].setdefault('mapper_exited_cleanly', process.returncode == 0)
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if log is not None:
            log.close()
        if not all(summary['checks'].values()):
            summary['status'] = 'failed'
        summary['elapsed_seconds'] = round(time.monotonic() - started, 3)
        args.output.write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
