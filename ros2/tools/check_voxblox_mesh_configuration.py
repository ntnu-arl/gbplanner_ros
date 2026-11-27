#!/usr/bin/env python3
"""Check mesh parameter aliases, strict weight filtering and independent TSDF color."""
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
    os.environ.setdefault('ROS_DOMAIN_ID', str(170 + os.getpid() % 20))

    import rclpy
    from gbplanner_voxblox.srv import FilePath
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_srvs.srv import Trigger
    from visualization_msgs.msg import Marker
    from voxblox_msgs.msg import Block, Layer, Mesh

    names = ('legacy', 'namespace')
    thresholds = {'legacy': .5, 'namespace': .75}
    specific_params = {
        'legacy': {'mesh_min_weight': .5, 'mesh_use_color': False},
        'namespace': {'mesh.min_weight': .75, 'mesh_min_weight': .25,
                      'mesh.use_color': False, 'mesh_use_color': True},
    }
    summary = {'status': 'failed', 'scope': 'two native ROS2 mesh configurations',
               'checks': {}, 'observations': {}, 'events': []}
    processes, logs, latest_status = {}, {}, {}
    received = {name: {'layers': deque(maxlen=8), 'meshes': deque(maxlen=16),
                       'markers': deque(maxlen=16)} for name in names}
    probe = None
    started, deadline = time.monotonic(), time.monotonic() + 25.
    resolution, side, color = .25, 4, (30, 90, 150)
    quantum = 2. / 65535.

    def stop_processes():
        for process in processes.values():
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        for name, process in processes.items():
            try:
                process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                summary['checks'][name + '_exited_cleanly'] = False
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=1.)
            summary['checks'].setdefault(name + '_exited_cleanly', process.returncode == 0)

    def float_word(value):
        return struct.unpack('<I', struct.pack('<f', value))[0]

    def read_float(value):
        return struct.unpack('<f', struct.pack('<I', int(value)))[0]

    def plane(weight):
        message = Layer(voxel_size=resolution, voxels_per_side=side,
                        layer_type='tsdf', action=Layer.ACTION_RESET)
        block = Block(x_index=0, y_index=0, z_index=0)
        data = []
        for z in range(side):
            for _ in range(side * side):
                data.extend((float_word((z + .5) * resolution - .5), float_word(weight), 0x1E5A96FF))
        block.data = data
        message.blocks = [block]
        return message

    def tsdf_records(message):
        if (message.layer_type != 'tsdf' or message.voxels_per_side != side or
                not math.isclose(message.voxel_size, resolution, abs_tol=1e-9)):
            raise AssertionError('unexpected TSDF Layer geometry')
        records = {}
        for block in message.blocks:
            if len(block.data) != 3 * side ** 3:
                raise AssertionError('TSDF Layer must have three words per voxel')
            for i in range(side ** 3):
                index = (int(block.x_index) * side + i % side,
                         int(block.y_index) * side + (i // side) % side,
                         int(block.z_index) * side + i // (side * side))
                packed = int(block.data[3 * i + 2])
                records[index] = {'distance': read_float(block.data[3 * i]),
                                  'weight': read_float(block.data[3 * i + 1]),
                                  'rgb': [(packed >> shift) & 255 for shift in (24, 16, 8)]}
        return records

    def ply_description(path):
        lines = path.read_text().splitlines()
        if lines[:2] != ['ply', 'format ascii 1.0']:
            raise AssertionError('expected original Voxblox ASCII PLY')
        header_end = lines.index('end_header')
        header = lines[:header_end]
        vertices = next(int(line.split()[2]) for line in header if line.startswith('element vertex '))
        faces = next((int(line.split()[2]) for line in header if line.startswith('element face ')), 0)
        if len(lines) != header_end + 1 + vertices + faces:
            raise AssertionError('PLY payload counts disagree with header')
        return {'vertices': vertices, 'triangles': faces,
                'color_properties': [line for line in header if line.startswith('property ') and
                                     line.split()[-1] in ('red', 'green', 'blue', 'alpha')],
                'bytes': path.stat().st_size}

    try:
        with tempfile.TemporaryDirectory(prefix='gbplanner-mesh-configuration-') as temporary:
            rclpy.init(args=[])
            probe = Node('voxblox_mesh_configuration_probe')
            reliable = QoSProfile(depth=40, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.VOLATILE)
            durable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            publishers, clients, exports, subscriptions = {}, {}, {}, []

            def consume_layer(name, message):
                records = tsdf_records(message)
                received[name]['layers'].append((message, records))
                summary['events'].append({'node': name, 'topic': 'tsdf_map_out',
                    'action': int(message.action), 'blocks': len(message.blocks),
                    'observed_voxels': sum(record['weight'] > 0 for record in records.values())})

            def consume_mesh(name, message):
                received[name]['meshes'].append(message)
                summary['events'].append({'node': name, 'topic': 'mesh', 'blocks': [
                    {'index': [int(value) for value in block.index], 'triangles': len(block.x) // 3}
                    for block in message.mesh_blocks]})

            def consume_marker(name, message):
                received[name]['markers'].append(message)
                summary['events'].append({'node': name, 'topic': 'surface_mesh',
                                          'action': int(message.action), 'triangles': len(message.points) // 3})

            for name in names:
                prefix = '/' + name + '/'
                publishers[name] = {
                    'layer': probe.create_publisher(Layer, prefix + 'tsdf_map_in', reliable),
                    'points': probe.create_publisher(PointCloud2, prefix + 'points', reliable),
                    'transform': probe.create_publisher(TransformStamped, prefix + 'transform', reliable),
                }
                clients[name] = {service: probe.create_client(Trigger, prefix + service)
                                 for service in ('mapper_status', 'publish_map', 'generate_mesh')}
                exports[name] = probe.create_client(FilePath, prefix + 'export_mesh')
                subscriptions.extend([
                    probe.create_subscription(Layer, prefix + 'tsdf_map_out',
                                              lambda message, name=name: consume_layer(name, message), reliable),
                    probe.create_subscription(Mesh, prefix + 'mesh',
                                              lambda message, name=name: consume_mesh(name, message), reliable),
                    probe.create_subscription(Marker, prefix + 'surface_mesh',
                                              lambda message, name=name: consume_marker(name, message), durable),
                ])
                parameters = {
                    'frame_id': 'map', 'sensor_frame': 'lidar', 'input_sensors': ['main'],
                    'input.main.topic': 'points', 'input.main.frame': 'lidar', 'use_tf_transforms': False,
                    'map.storage': 'sparse', 'map.resolution': resolution, 'tsdf.voxels_per_side': side,
                    'tsdf.method': 'simple', 'tsdf.truncation_distance': .5, 'tsdf.use_weight_dropoff': False,
                    'esdf.enabled': False, 'color_mode': 'gray', 'publish_tsdf_map': True,
                    'publish_map_every_n_sec': 0., 'update_mesh_every_n_sec': 0., **specific_params[name],
                }
                params = Path(temporary) / (name + '.yaml')
                params.write_text(json.dumps({'/' + name + '/voxblox_mapper': {'ros__parameters': parameters}}))
                logs[name] = args.output.with_name(args.output.stem + '_' + name + '.log').open('w')
                processes[name] = subprocess.Popen(
                    ['ros2', 'run', 'gbplanner_voxblox', 'voxblox_mapper_node', '--ros-args',
                     '-r', '__ns:=/' + name, '--params-file', str(params)],
                    stdout=logs[name], stderr=subprocess.STDOUT, start_new_session=True, env=dict(os.environ))

            def spin_until(predicate, label):
                while True:
                    for name, process in processes.items():
                        if process.poll() is not None:
                            raise RuntimeError(f'{name} mapper exited ({process.returncode})')
                    if time.monotonic() >= deadline:
                        raise TimeoutError(label)
                    value = predicate()
                    if value:
                        return value
                    rclpy.spin_once(probe, timeout_sec=.01)

            def call(client, request, label, allow_false=False):
                future = client.call_async(request)
                spin_until(future.done, label)
                response = future.result()
                if response is None or (not allow_false and not response.success):
                    raise RuntimeError(f'{label}: {None if response is None else response.message}')
                return response

            def service(name, operation):
                return call(clients[name][operation], Trigger.Request(), name + ' ' + operation,
                            allow_false=operation == 'mapper_status')

            def status(name):
                latest_status[name] = json.loads(service(name, 'mapper_status').message)
                return latest_status[name]

            def check(name, condition):
                summary['checks'][name] = bool(condition)
                if not condition:
                    raise AssertionError(name)

            def send_plane(name, weight):
                previous = status(name)['received_layers']
                received[name]['layers'].clear()
                received[name]['meshes'].clear()
                received[name]['markers'].clear()
                publishers[name]['layer'].publish(plane(weight))
                spin_until(lambda: status(name)['received_layers'] == previous + 1, name + ' plane import')
                records = spin_until(lambda: received[name]['layers'][-1][1] if received[name]['layers'] else None,
                                     name + ' plane TSDF output')
                check(name + '_plane_weight_' + str(weight) + '_preserves_tsdf_color',
                      len(records) == 64 and all(abs(record['weight'] - weight) < 1e-6 and
                                               record['rgb'] == list(color) for record in records.values()))
                return json.loads(service(name, 'generate_mesh').message)

            spin_until(lambda: all(client.service_is_ready() for group in clients.values() for client in group.values())
                       and all(client.service_is_ready() for client in exports.values()), 'service discovery')
            spin_until(lambda: all(publisher.get_subscription_count() > 0
                                   for group in publishers.values() for publisher in group.values()), 'input discovery')
            spin_until(lambda: all(probe.count_publishers(subscription.topic_name) > 0 for subscription in subscriptions),
                       'output discovery')

            # A genuine colored scan precedes the analytic layer. The storage
            # check therefore exercises the input RGB decoder, not just import.
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = 'map', 'tracking'
            transform.header.stamp.sec = 21
            transform.transform.translation.x = transform.transform.translation.y = transform.transform.translation.z = .125
            transform.transform.rotation.w = 1.
            points = [(x * .125, y * .125, 1.) for y in range(-5, 6) for x in range(-5, 6)]
            cloud = PointCloud2()
            cloud.header.frame_id, cloud.header.stamp.sec = 'lidar', 21
            cloud.height, cloud.width, cloud.point_step = 1, len(points), 16
            cloud.row_step = cloud.width * cloud.point_step
            cloud.is_dense = True
            cloud.fields = [PointField(name=axis, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                            for i, axis in enumerate(('x', 'y', 'z'))]
            cloud.fields.append(PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1))
            cloud.data = b''.join(struct.pack('<fffI', *point, 0xFF1E5A96) for point in points)
            for name in names:
                resolved = status(name)['mesh_configuration']
                check(name + '_mesh_parameters_resolve', resolved['use_color'] is False and
                      math.isclose(resolved['min_weight'], thresholds[name], abs_tol=1e-9))
                publishers[name]['transform'].publish(transform)
                publishers[name]['points'].publish(cloud)
            for name in names:
                spin_until(lambda name=name: status(name)['integrated_scans'] == 1, name + ' colored scan integration')
                service(name, 'publish_map')
                records = spin_until(lambda name=name: received[name]['layers'][-1][1]
                                     if received[name]['layers'] else None, name + ' measured TSDF Layer')
                measured = records.get((0, 0, 4))  # Centre of the direct z=1 m return.
                check(name + '_mesh_color_disabled_preserves_measured_rgb', measured is not None and
                      measured['weight'] > 0 and all(abs(measured['rgb'][axis] - color[axis]) <= 1 for axis in range(3)))
                ply_path = Path(temporary) / (name + '.ply')
                call(exports[name], FilePath.Request(file_path=str(ply_path)), name + ' measured mesh export')
                ply = ply_description(ply_path)
                check(name + '_measured_ply_has_surface_without_rgb', ply['vertices'] > 0 and
                      ply['triangles'] > 0 and not ply['color_properties'])
                summary['observations'][name + '_measured_scan'] = {'accepted_points': len(points),
                    'target_voxel': [0, 0, 4], 'stored_tsdf': measured, 'ply': ply}

            for name in names:
                equal = send_plane(name, thresholds[name])
                spin_until(lambda name=name: any(marker.action == Marker.DELETE for marker in received[name]['markers']),
                           name + ' equal-weight Marker deletion')
                spin_until(lambda name=name: any(tuple(int(v) for v in block.index) == (0, 0, 0) and not len(block.x)
                           for message in received[name]['meshes'] for block in message.mesh_blocks),
                           name + ' equal-weight empty MeshBlock')
                check(name + '_weight_equal_to_minimum_produces_no_surface', equal['triangles'] == 0)

                above = send_plane(name, 1.)
                mesh = spin_until(lambda name=name: next((message for message in reversed(received[name]['meshes'])
                    if len(message.mesh_blocks) == 1 and len(message.mesh_blocks[0].x) == 54), None),
                    name + ' eighteen-triangle Mesh')
                marker = spin_until(lambda name=name: next((message for message in reversed(received[name]['markers'])
                    if message.action == Marker.ADD and len(message.points) == 54), None),
                    name + ' eighteen-triangle Marker')
                block = mesh.mesh_blocks[0]
                check(name + '_weight_above_minimum_produces_eighteen_triangles', above['triangles'] == 18 and
                      tuple(int(value) for value in block.index) == (0, 0, 0) and
                      all(abs(2. * int(value) / 65535. - .5) <= quantum for value in block.z) and
                      all(abs(point.z - .5) < 1e-6 for point in marker.points))
                check(name + '_gray_wire_and_marker_do_not_require_mesh_rgb',
                      all(len(values) == 54 and all(int(value) == 127 for value in values)
                          for values in (block.r, block.g, block.b)) and len(marker.colors) == 54 and
                      all(abs(channel - .5) < 1e-6 for rgba in marker.colors for channel in (rgba.r, rgba.g, rgba.b)))
                summary['observations'][name + '_thresholds'] = {'minimum_weight': thresholds[name],
                    'equal_weight_generation': equal, 'above_weight_generation': above,
                    'mesh_triangles': len(block.x) // 3, 'marker_triangles': len(marker.points) // 3}
                final = status(name)
                check(name + '_all_inputs_completed', final['integrated_scans'] == 1 and final['received_layers'] == 2 and
                      final['rejected_scans'] == 0 and final['rejected_layers'] == 0 and final['rejected_transforms'] == 0)
            summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
    finally:
        stop_processes()
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for log in logs.values():
            log.close()
        summary['mapper_exit_codes'] = {name: process.returncode for name, process in processes.items()}
        summary['last_mapper_status'] = latest_status
        summary['elapsed_seconds'] = round(time.monotonic() - started, 3)
        if not all(summary['checks'].values()):
            summary['status'] = 'failed'
        args.output.write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
