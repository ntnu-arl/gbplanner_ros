#!/usr/bin/env python3
"""Plan and home over a full 3D occupied terrain, using external base poses."""
import argparse
import csv
import hashlib
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
from planner_msgs.msg import Graph
from planner_msgs.srv import ValidatePath
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger

from global_demo import GlobalDemo
from local_demo import xyz

ORIGIN = (-4.0, -3.0, -1.0)
DIMENSIONS = (80, 60, 40)
RESOLUTION = 0.1
BASE_HEIGHT = 0.55
PLANNING_HEIGHT = 0.8
ROBOT_SIZE = (0.4, 0.4, 0.4)
MAX_SLOPE = 0.85
UNKNOWN_START_X = 2.5


def ideal_surface(x, y):
    # A rising ramp with a rounded crest and dip; voxelization preserves its
    # observed upper faces instead of feeding a 2D height map to the planner.
    return 0.12 * (x + 2.0) + 0.20 * math.sin(1.2 * (x + 2.0)) + 0.08 * math.sin(0.8 * y)


def make_terrain(stamp):
    cloud = PointCloud2()
    cloud.header.frame_id = 'map'
    cloud.header.stamp = stamp
    cloud.height = 1
    cloud.width = math.prod(DIMENSIONS)
    cloud.point_step = 16
    cloud.row_step = cloud.width * cloud.point_step
    cloud.is_dense = True
    cloud.fields = [PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
                    for index, name in enumerate(('x', 'y', 'z'))]
    cloud.fields.append(PointField(name='occupancy', offset=12, datatype=PointField.INT8, count=1))
    data = bytearray(cloud.row_step)
    offset = 0
    for iz in range(DIMENSIONS[2]):
        z = ORIGIN[2] + (iz + 0.5) * RESOLUTION
        for iy in range(DIMENSIONS[1]):
            y = ORIGIN[1] + (iy + 0.5) * RESOLUTION
            for ix in range(DIMENSIONS[0]):
                x = ORIGIN[0] + (ix + 0.5) * RESOLUTION
                status = -1 if x >= UNKNOWN_START_X else (100 if z <= ideal_surface(x, y) else 0)
                struct.pack_into('<fffbxxx', data, offset, x, y, z, status)
                offset += cloud.point_step
    cloud.data = bytes(data)
    return cloud


class ObservedTerrain:
    def __init__(self, cloud):
        self.cells = {}
        self.surfaces = {}
        self.counts = {'free': 0, 'occupied': 0, 'unknown': 0}
        # Read the bytes actually published, including their float32 centers.
        for offset in range(0, len(cloud.data), cloud.point_step):
            x, y, z, status = struct.unpack_from('<fffb', cloud.data, offset)
            index = tuple(round((value - base) / RESOLUTION - 0.5)
                          for value, base in zip((x, y, z), ORIGIN))
            self.cells[index] = status
            self.counts[{0: 'free', 100: 'occupied', -1: 'unknown'}[status]] += 1
            if status == 100:
                xy = index[:2]
                face = ORIGIN[2] + (index[2] + 1) * RESOLUTION
                self.surfaces[xy] = max(face, self.surfaces.get(xy, -math.inf))

    def index(self, point):
        return tuple(math.floor((value - base) / RESOLUTION + 1e-10)
                     for value, base in zip(point, ORIGIN))

    def support(self, x, y):
        index = self.index((x, y, 0.0))[:2]
        assert index in self.surfaces, ('No observed support beneath path', x, y)
        return self.surfaces[index]

    def base_point(self, x, y):
        return [x, y, self.support(x, y) + BASE_HEIGHT]

    def check_path(self, path, height=BASE_HEIGHT):
        assert path
        max_height_error = 0.0
        max_slope = 0.0
        length = 0.0
        samples = 0
        for a, b in zip(path, path[1:]):
            distance = math.dist(a, b)
            length += distance
            horizontal = math.hypot(b[0] - a[0], b[1] - a[1])
            if distance > 1e-10:
                slope = abs(math.atan2(b[2] - a[2], horizontal))
                max_slope = max(max_slope, slope)
                assert slope <= MAX_SLOPE + 1e-8, ('Terrain segment exceeds slope limit', a, b, slope)
            for part in range(max(1, math.ceil(distance / (RESOLUTION / 2.0))) + 1):
                count = max(1, math.ceil(distance / (RESOLUTION / 2.0)))
                sample = [start + (end - start) * part / count for start, end in zip(a, b)]
                surface = self.support(sample[0], sample[1])
                error = abs(sample[2] - surface - height)
                max_height_error = max(max_height_error, error)
                assert error <= RESOLUTION + 1e-7, ('Path does not follow observed terrain height', sample, surface, height)
                # Check physical base volume even when inspecting an internal
                # planning-height polyline or graph representation.
                base = [sample[0], sample[1], sample[2] + BASE_HEIGHT - height]
                for offset in itertools.product((-0.2, -0.1, 0.0, 0.1, 0.2), repeat=3):
                    probe = [value + delta for value, delta in zip(base, offset)]
                    assert self.cells.get(self.index(probe), -1) == 0, ('Robot box outside observed free space', probe)
                samples += 1
        if len(path) == 1:
            max_height_error = abs(path[0][2] - self.support(*path[0][:2]) - height)
            assert max_height_error <= RESOLUTION + 1e-7
        return {'height_above_support_m': height, 'maximum_height_error_m': max_height_error,
                'maximum_segment_slope_rad': max_slope, 'sampled_positions': samples,
                'path_length_m': length, 'height_span_m': max(p[2] for p in path) - min(p[2] for p in path)}

    def scene_record(self):
        columns = []
        for iy in range(DIMENSIONS[1]):
            for ix in range(DIMENSIONS[0]):
                x = ORIGIN[0] + (ix + 0.5) * RESOLUTION
                y = ORIGIN[1] + (iy + 0.5) * RESOLUTION
                support = self.surfaces.get((ix, iy))
                columns.append([x, y, support, support is None])
        return {'origin': ORIGIN, 'dimensions': DIMENSIONS, 'resolution_m': RESOLUTION,
                'unknown_start_x': UNKNOWN_START_X, 'voxel_counts': self.counts,
                'surface_column_fields': ['x', 'y', 'observed_surface_z', 'unknown_column'],
                'surface_columns': columns,
                'ideal_surface_formula': '0.12*(x+2) + 0.20*sin(1.2*(x+2)) + 0.08*sin(0.8*y)',
                'input': '3D occupied/free/unknown PointCloud2 voxel snapshot'}


def graph_record(graph):
    return {'height_convention': 'planning',
            'vertices': [{'id': vertex.id,
                          'position': [vertex.pose.position.x, vertex.pose.position.y, vertex.pose.position.z],
                          'frontier': vertex.is_frontier} for vertex in graph.vertices],
            'edges': [[edge.source_id, edge.target_id, edge.weight] for edge in graph.edges]}


def save(path, report):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(report, indent=2) + '\n')
    temporary.replace(path)


def save_route(path, points, terrain):
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['x', 'y', 'base_z', 'observed_surface_z', 'height_above_surface'])
        for x, y, z in points:
            surface = terrain.support(x, y)
            writer.writerow([x, y, z, surface, z - surface])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('results/ground_planning/summary.json'))
    parser.add_argument('--start-node', action='store_true', help='Start the configured ground planner')
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = {'status': 'running', 'scope': 'Native ROS 2 terrain planning and homing with synthetic voxel observations and odometry',
              'base_height_m': BASE_HEIGHT, 'planning_height_m': PLANNING_HEIGHT,
              'robot_size_m': ROBOT_SIZE, 'seed': 42}
    node = process = log = None
    try:
        config = Path(get_package_share_directory('gbplanner_local')) / 'config/ground.yaml'
        report['configuration'] = str(config)
        if args.start_node:
            log = (args.output.parent / 'nodes.log').open('w')
            process = subprocess.Popen(
                ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
                 '--params-file', str(config)], stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
        rclpy.init()
        node = GlobalDemo()
        validator = node.create_client(ValidatePath, 'validate_path')
        expand = node.create_client(Trigger, 'expand_global')
        assert validator.wait_for_service(timeout_sec=15) and expand.wait_for_service(timeout_sec=15)
        local_graphs = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Graph, 'local_graph', local_graphs.append, qos)
        raw = make_terrain(node.get_clock().now().to_msg())
        terrain = ObservedTerrain(raw)
        report['scene'] = terrain.scene_record()
        report['input_map_sha256'] = hashlib.sha256(bytes(raw.data)).hexdigest()
        node.publish_map(raw)
        home = terrain.base_point(-2.0, 0.0)
        report['home_base_pose'] = home
        report['home_planning_pose'] = [home[0], home[1], home[2] + PLANNING_HEIGHT - BASE_HEIGHT]
        node.move_odometry(home)
        response, metrics = node.plan()
        report['local'] = {'metrics': metrics, 'path': xyz(response.path)}
        save(args.output, report)
        assert metrics['success'], metrics
        local_path = xyz(response.path)
        assert len(local_path) > 2 and math.dist(local_path[0], home) < 1e-8, local_path
        local_check = terrain.check_path(local_path)
        assert local_check['height_span_m'] >= 2.0 * RESOLUTION - 1e-7, local_check
        assert abs(local_check['path_length_m'] - metrics['path_length_m']) < 1e-6, (local_check, metrics)
        request = ValidatePath.Request()
        request.header.frame_id = 'map'
        request.path = response.path
        validation = node.call(validator, request)
        assert validation.valid and validation.reason == 'ok', validation.reason
        node.spin_until(lambda: bool(local_graphs))
        local_graph = graph_record(local_graphs[-1])
        graph_errors = []
        for vertex in local_graph['vertices']:
            x, y, z = vertex['position']
            error = abs(z - terrain.support(x, y) - PLANNING_HEIGHT)
            assert error <= RESOLUTION + 1e-7, ('Graph uses wrong height convention', vertex, error)
            graph_errors.append(error)
        report['local'] = {'path': local_path, 'metrics': metrics, 'terrain_checks': local_check,
                           'validation': {'valid': validation.valid, 'reason': validation.reason,
                                          'map_revision': validation.map_revision},
                           'graph': local_graph, 'max_graph_planning_height_error_m': max(graph_errors)}
        save_route(args.output.parent / 'local_path.csv', local_path, terrain)
        save(args.output, report)

        # Publish actual base poses along the accepted terrain polyline. These
        # are observation messages; no future root is substituted for odometry.
        odometry = [{'position': home, 'global_graph': node.status()}]
        previous = home
        for point in local_path[1:]:
            yaw = math.atan2(point[1] - previous[1], point[0] - previous[0])
            node.move_odometry(point, yaw)
            odometry.append({'position': point, 'yaw': yaw, 'global_graph': node.status()})
            previous = point
        report['odometry'] = odometry
        endpoint = local_path[-1]
        homing_response, homing_metrics = node.global_call(homing=True)
        report['homing'] = {'metrics': homing_metrics, 'path': xyz(homing_response.path)}
        save(args.output, report)
        assert homing_metrics['success'], homing_metrics
        homing_path = xyz(homing_response.path)
        assert len(homing_path) > 2, homing_path
        assert math.dist(homing_path[0], endpoint) < 1e-8 and math.dist(homing_path[-1], home) < 1e-8
        homing_check = terrain.check_path(homing_path)
        assert homing_check['height_span_m'] >= 2.0 * RESOLUTION - 1e-7, homing_check
        assert abs(homing_check['path_length_m'] - homing_metrics['distance_m']) < 1e-6
        request.path = homing_response.path
        checked = node.call(validator, request)
        assert checked.valid and checked.reason == 'ok', checked.reason
        report['homing'] = {'path': homing_path, 'metrics': homing_metrics, 'terrain_checks': homing_check,
                            'validation': {'valid': checked.valid, 'reason': checked.reason,
                                           'map_revision': checked.map_revision},
                            'global_graph': node.status()}
        save_route(args.output.parent / 'homing_path.csv', homing_path, terrain)
        node.spin_until(lambda: bool(node.global_graphs))
        report['global_graph'] = graph_record(node.global_graphs[-1])
        # Exercise the native background entry once. The retained upstream 5 m
        # exclusion radius may correctly add no vertex in this compact room.
        expansion = node.call(expand, Trigger.Request())
        assert expansion.success, expansion.message
        report['background'] = json.loads(expansion.message)
        report.update(status='passed', checks={
            'full_3d_occupancy_input': True, 'unknown_columns_have_no_invented_support': True,
            'local_path_has_varying_height': True, 'base_path_follows_observed_surface': True,
            'graph_keeps_planning_height': True, 'native_validate_path_accepts_local_and_homing': True,
            'actual_odometry_recorded_along_route': True, 'homing_uses_terrain_polyline': True,
            'continuous_support_and_robot_box_checked': True,
            'background_service_runs_with_ground_mode': True})
        save(args.output, report)
        print(json.dumps({'status': report['status'], 'local_length_m': local_check['path_length_m'],
                          'local_height_span_m': local_check['height_span_m'],
                          'homing_length_m': homing_check['path_length_m'],
                          'homing_height_span_m': homing_check['height_span_m'],
                          'output': str(args.output)}), flush=True)
    except Exception as error:
        report.update(status='failed', error=repr(error))
        save(args.output, report)
        raise
    finally:
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


if __name__ == '__main__':
    main()
