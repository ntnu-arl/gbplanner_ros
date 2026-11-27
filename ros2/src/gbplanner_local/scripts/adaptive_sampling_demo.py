#!/usr/bin/env python3
"""Change corridor observations on one native planner and inspect its PCA box."""
import argparse
import csv
import itertools
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from planner_msgs.msg import Graph
from planner_msgs.srv import ValidatePath
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger

from local_demo import Demo, xyz

ORIGIN = (-6.0, -6.0, -2.4)
DIMENSIONS = (60, 60, 24)
RESOLUTION = 0.2
ROOT = (0.0, 0.0, 0.0)
FALLBACK = {'minimum': [-2.2, -1.1, -0.6], 'maximum': [2.2, 1.1, 0.6],
            'rotations': [0.0, 0.0, 0.0], 'mean': [0.0, 0.0, 0.0], 'scale': [1.0, 1.0, 0.5]}
SCENES = [
    {'name': 'corridor_a', 'yaw': 0.2, 'half_length': 5.0, 'half_width': 1.15,
     'half_height': 0.8, 'unknown_start': 1.8},
    {'name': 'corridor_b', 'yaw': 1.2, 'half_length': 5.0, 'half_width': 1.6,
     'half_height': 1.1, 'unknown_start': 1.8},
    {'name': 'free_space', 'all_free': True},
]


def cell_center(index):
    return [base + (value + 0.5) * RESOLUTION for base, value in zip(ORIGIN, index)]


def status_at(point, scene):
    if scene.get('all_free'):
        return 0
    x, y, z = point
    c, s = math.cos(scene['yaw']), math.sin(scene['yaw'])
    along, across = c * x + s * y, -s * x + c * y
    if (abs(along) >= scene['half_length'] or abs(across) >= scene['half_width']
            or abs(z) >= scene['half_height']):
        return 100
    return -1 if along >= scene['unknown_start'] else 0


def make_scene(scene, stamp):
    message = PointCloud2()
    message.header.frame_id = 'map'
    message.header.stamp = stamp
    message.height = 1
    message.width = math.prod(DIMENSIONS)
    message.point_step = 16
    message.row_step = message.width * message.point_step
    message.is_dense = True
    message.fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(('x', 'y', 'z'))]
    message.fields.append(PointField(name='occupancy', offset=12, datatype=PointField.INT8, count=1))
    cells = {}
    data = bytearray(message.row_step)
    offset = 0
    for z in range(DIMENSIONS[2]):
        for y in range(DIMENSIONS[1]):
            for x in range(DIMENSIONS[0]):
                index = (x, y, z)
                center = cell_center(index)
                status = status_at(center, scene)
                cells[index] = status
                struct.pack_into('<fffbxxx', data, offset, *center, status)
                offset += 16
    message.data = bytes(data)
    return message, cells


def inspect_path(path, cells):
    count = 0
    for a, b in zip(path, path[1:]):
        steps = max(1, math.ceil(math.dist(a, b) / 0.04))
        for part in range(steps + 1):
            center = [start + (end - start) * part / steps for start, end in zip(a, b)]
            for offset in itertools.product((-0.2, -0.1, 0.0, 0.1, 0.2), repeat=3):
                point = [value + shift for value, shift in zip(center, offset)]
                index = tuple(math.floor((value - base) / RESOLUTION + 1e-10)
                              for value, base in zip(point, ORIGIN))
                assert cells.get(index, -1) == 0, ('Path outside observed free cells', point)
                count += 1
    return count


def matvec(matrix, vector):
    return [sum(matrix[3 * row + col] * vector[col] for col in range(3)) for row in range(3)]


def world_from_local(matrix, vector):
    # Adaptive rotation maps world offsets into the sampling frame.
    return [ROOT[row] + sum(matrix[3 * col + row] * vector[col] for col in range(3)) for row in range(3)]


def close_vector(actual, expected):
    assert len(actual) == len(expected)
    assert all(abs(a - b) < 1e-8 for a, b in zip(actual, expected)), (actual, expected)


def geometry(graph, adaptive):
    vertices = []
    for vertex in graph.vertices:
        point = vertex.pose.position
        world = [point.x, point.y, point.z]
        local = matvec(adaptive['rotation'], [a - b for a, b in zip(world, ROOT)])
        assert all(lo - 1e-8 <= value <= hi + 1e-8
                   for lo, value, hi in zip(adaptive['minimum'], local, adaptive['maximum'])), (world, local, adaptive)
        vertices.append({'id': vertex.id, 'world': world, 'sampling': local})
    corners = [world_from_local(adaptive['rotation'], list(corner))
               for corner in itertools.product(*zip(adaptive['minimum'], adaptive['maximum']))]
    return {'vertices': vertices,
            'edges': [[edge.source_id, edge.target_id, edge.weight] for edge in graph.edges],
            'sampling_box_world_corners': corners}


def surface_points(cells):
    steps = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
    return [cell_center(index) for index, status in cells.items() if status == 100
            and any(cells.get(tuple(a + b for a, b in zip(index, step)), 100) != 100 for step in steps)]


def save_slice(cells, path):
    z = DIMENSIONS[2] // 2
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['x', 'y', 'status'])
        for y in range(DIMENSIONS[1]):
            for x in range(DIMENSIONS[0]):
                center = cell_center((x, y, z))
                writer.writerow([center[0], center[1], cells[(x, y, z)]])


def save(path, report):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(report, indent=2) + '\n')
    temporary.replace(path)


def major_axis(adaptive):
    dimensions = [hi - lo for lo, hi in zip(adaptive['minimum'], adaptive['maximum'])]
    row = max(range(3), key=lambda i: dimensions[i])
    return adaptive['rotation'][3 * row:3 * row + 3], dimensions


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('results/adaptive_sampling/summary.json'))
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = {'status': 'running', 'root': ROOT, 'root_yaw': 0.0, 'seed': 42,
              'scope': 'One ROS 2 planner adapts to two observed 3D corridors, then falls back without surfaces',
              'grid': {'origin': ORIGIN, 'dimensions': DIMENSIONS, 'resolution_m': RESOLUTION},
              'configured_sampling': FALLBACK, 'cases': []}
    node = process = timer = log = None
    try:
        config = Path(get_package_share_directory('gbplanner_local')) / 'config/adaptive.yaml'
        log = (args.output.parent / 'nodes.log').open('w')
        process = subprocess.Popen(
            ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args', '--params-file', str(config)],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Demo()
        validator = node.create_client(ValidatePath, 'validate_path')
        sensor_gain = node.create_client(Trigger, 'sensor_gain')
        for client in (validator, sensor_gain):
            assert client.wait_for_service(timeout_sec=15)
        graphs = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Graph, 'local_graph', graphs.append, qos)

        def odometry():
            message = Odometry()
            message.header.frame_id = 'map'
            message.header.stamp = node.get_clock().now().to_msg()
            message.pose.pose.orientation.w = 1.0
            node.odometry.publish(message)

        timer = node.create_timer(0.05, odometry)
        node.spin_until(lambda: node.odometry.get_subscription_count() > 0)
        for scene in SCENES:
            raw, cells = make_scene(scene, node.get_clock().now().to_msg())
            node.publish_map(raw)
            node.spin_until(lambda: node.call(sensor_gain, Trigger.Request()).success)
            old_graphs = len(graphs)
            response, metrics = node.plan()
            adaptive = metrics['adaptive']
            record = {'scene': scene, 'metrics': metrics, 'path': xyz(response.path),
                      'surface_points': surface_points(cells),
                      'voxel_counts': {name: sum(value == status for value in cells.values())
                                       for name, status in [('free', 0), ('occupied', 100), ('unknown', -1)]}}
            report['cases'].append(record)
            save(args.output, report)
            assert adaptive['enabled'], adaptive
            node.spin_until(lambda: len(graphs) > old_graphs)
            record['graph'] = geometry(graphs[-1], adaptive)
            record['map_slice'] = scene['name'] + '_map.csv'
            save_slice(cells, args.output.parent / record['map_slice'])
            if scene.get('all_free'):
                assert not adaptive['applied'] and adaptive['raw_points'] == 0, adaptive
                assert adaptive['reason'] == 'insufficient_points', adaptive
                for key, expected in FALLBACK.items():
                    close_vector(adaptive[key], expected)
                close_vector(adaptive['rotation'], [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
                assert not metrics['success'] and metrics['reason'] == 'no_gain' and not response.path, metrics
            else:
                assert adaptive['applied'] and adaptive['raw_points'] >= adaptive['filtered_points'] >= 4, adaptive
                assert metrics['success'] and metrics['gain'] > 0.0, metrics
                assert len(response.path) >= 2 and math.dist(record['path'][0], ROOT) < 1e-8
                axis, dimensions = major_axis(adaptive)
                corridor_axis = [math.cos(scene['yaw']), math.sin(scene['yaw']), 0.0]
                alignment = abs(sum(a * b for a, b in zip(axis, corridor_axis)))
                assert alignment > math.cos(math.pi / 6.0), ('Sampling long axis misses corridor', alignment, adaptive)
                record.update(sampling_dimensions=dimensions, major_axis_world=axis,
                              corridor_alignment_abs_dot=alignment,
                              independent_free_box_probes=inspect_path(record['path'], cells))
                request = ValidatePath.Request()
                request.header.frame_id = 'map'
                request.path = response.path
                checked = node.call(validator, request)
                assert checked.valid and checked.reason == 'ok', checked.reason
                record['validation'] = {'valid': checked.valid, 'reason': checked.reason,
                                        'map_revision': checked.map_revision}
            save(args.output, report)
            print(json.dumps({'scene': scene['name'], 'adaptive': adaptive['applied'],
                              'reason': adaptive['reason'], 'planning_reason': metrics['reason'],
                              'vertices': metrics['vertices']}), flush=True)
        first, second, fallback = report['cases']
        alignment = abs(sum(a * b for a, b in zip(first['major_axis_world'], second['major_axis_world'])))
        assert alignment < 0.9, ('Adaptive direction did not change', alignment)
        size_change = math.dist(sorted(first['sampling_dimensions']), sorted(second['sampling_dimensions']))
        assert size_change > 0.15, ('Adaptive dimensions did not change', size_change)
        report.update(status='passed', between_corridors_axis_abs_dot=alignment,
                      between_corridors_dimensions_change_m=size_change, checks={
                          'same_node_uses_latest_map_each_request': True,
                          'sampling_orientation_changes_with_corridor': True,
                          'sampling_dimensions_change_with_geometry': True,
                          'all_graph_vertices_inside_reported_adaptive_frame': True,
                          'both_corridors_produce_valid_positive_gain_paths': True,
                          'no_surface_map_restores_all_configured_sampling_values': True,
                          'fallback_does_not_reuse_previous_pca_box': True})
        save(args.output, report)
    except Exception as error:
        report.update(status='failed', error=repr(error))
        save(args.output, report)
        raise
    finally:
        if timer is not None:
            timer.cancel()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        if log is not None:
            log.close()
    print(json.dumps({'status': report['status'], 'output': str(args.output)}), flush=True)


if __name__ == '__main__':
    main()
