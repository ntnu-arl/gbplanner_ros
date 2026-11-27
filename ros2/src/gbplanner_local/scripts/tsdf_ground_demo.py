#!/usr/bin/env python3
"""Fuse terrain returns in Voxblox, then plan and home using observed TSDF."""
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
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from planner_msgs.msg import Graph
from planner_msgs.srv import ValidatePath
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from global_demo import GlobalDemo
from ground_planning_demo import graph_record, save
from local_demo import xyz

ORIGIN = (-3.0, -3.0, -1.0)
DIMENSIONS = (30, 30, 25)
RESOLUTION = 0.2
BASE_HEIGHT = 0.65
PLANNING_HEIGHT = 0.9
ROBOT_SIZE = 0.3
SCAN_ORIGINS = [(-1.0, 0.0, 3.0), (0.0, 0.0, 3.0), (1.0, 0.0, 3.0)]
ROOT_XY = (-1.4, 0.0)


def surface(x, y):
    return 0.13 + 0.12 * max(0.0, x + 0.6)


def terrain_returns(stamp, origin):
    """Surface returns in sensor coordinates; no occupancy/TSDF is supplied."""
    cloud = PointCloud2()
    cloud.header.frame_id = 'terrain_sensor'
    cloud.header.stamp = stamp
    cloud.height = 1
    cloud.point_step = 12
    cloud.is_dense = True
    cloud.fields = [PointField(name=name, offset=axis * 4,
                               datatype=PointField.FLOAT32, count=1)
                    for axis, name in enumerate(('x', 'y', 'z'))]
    data = bytearray()
    # Dense ground returns fill free space through the original ray integrator.
    # Flat and rising regions belong to one continuous physical surface.
    for ix in range(131):
        x = -2.6 + ix * 0.04
        for iy in range(131):
            y = -2.6 + iy * 0.04
            data.extend(struct.pack('<fff', x - origin[0], y - origin[1],
                                    surface(x, y) - origin[2]))
    cloud.width = len(data) // cloud.point_step
    cloud.row_step = len(data)
    cloud.data = bytes(data)
    return cloud


class ObservedTsdf:
    def __init__(self, cloud):
        fields = {field.name: field for field in cloud.fields}
        assert cloud.point_step == 24, cloud.point_step
        for name, offset in [('tsdf_distance', 16), ('tsdf_weight', 20)]:
            field = fields[name]
            assert field.offset == offset and field.datatype == PointField.FLOAT32 and field.count == 1
        self.cells = {}
        self.columns = {}
        self.rows = []
        for row in range(cloud.height):
            for column in range(cloud.width):
                offset = row * cloud.row_step + column * cloud.point_step
                x, y, z, occupancy, distance, weight = struct.unpack_from('<fffbxxxff', cloud.data, offset)
                assert all(math.isfinite(value) for value in (x, y, z, distance, weight))
                index = tuple(round((value - base) / RESOLUTION - 0.5)
                              for value, base in zip((x, y, z), ORIGIN))
                assert index not in self.cells
                self.cells[index] = (occupancy, distance, weight)
                self.rows.append((x, y, z, occupancy, distance, weight))
                if weight > 0:
                    self.columns.setdefault(index[:2], []).append((z, distance, weight))
        self.surfaces = {}
        for xy, column in self.columns.items():
            column.sort()
            for lower, upper in zip(column, column[1:]):
                if lower[1] <= 0 < upper[1] and upper[0] - lower[0] < RESOLUTION * 1.01:
                    fraction = -lower[1] / (upper[1] - lower[1])
                    self.surfaces[xy] = lower[0] + fraction * (upper[0] - lower[0])
        assert self.surfaces, 'No measured TSDF zero crossing'

    @staticmethod
    def index(point):
        return tuple(math.floor((value - base) / RESOLUTION)
                     for value, base in zip(point, ORIGIN))

    def reconstructed_surface(self, x, y):
        # Bilinear interpolation between independently extracted vertical zero
        # crossings is for analysis only; the planner uses its own TSDF queries.
        lattice = [(value - base) / RESOLUTION - 0.5
                   for value, base in zip((x, y), ORIGIN[:2])]
        lower = [math.floor(value) for value in lattice]
        fraction = [value - index for value, index in zip(lattice, lower)]
        corners = [(lower[0] + ix, lower[1] + iy) for ix, iy in itertools.product((0, 1), repeat=2)]
        if all(corner in self.surfaces for corner in corners):
            return sum(self.surfaces[(lower[0] + ix, lower[1] + iy)] *
                       (fraction[0] if ix else 1 - fraction[0]) *
                       (fraction[1] if iy else 1 - fraction[1])
                       for ix, iy in itertools.product((0, 1), repeat=2))
        return self.surfaces.get(self.index((x, y, 0.0))[:2])

    def check_path(self, path):
        assert len(path) >= 2
        length = 0.0
        errors = []
        source_errors = []
        maximum_slope = 0.0
        samples = 0
        for start, end in zip(path, path[1:]):
            distance = math.dist(start, end)
            length += distance
            maximum_slope = max(maximum_slope, abs(math.atan2(
                end[2] - start[2], math.dist(start[:2], end[:2]))))
            count = max(1, math.ceil(distance / (RESOLUTION / 2)))
            for part in range(count + 1):
                point = [a + (b - a) * part / count for a, b in zip(start, end)]
                estimated = self.reconstructed_surface(*point[:2])
                assert estimated is not None, ('No TSDF surface under route', point)
                error = abs(point[2] - BASE_HEIGHT - estimated)
                errors.append(error)
                source_errors.append(abs(point[2] - BASE_HEIGHT - surface(*point[:2])))
                assert error <= RESOLUTION, ('Route differs from fused surface', point, estimated)
                for offset in itertools.product((-ROBOT_SIZE / 2, 0, ROBOT_SIZE / 2), repeat=3):
                    index = self.index([value + delta for value, delta in zip(point, offset)])
                    assert self.cells.get(index, (-1, 0, 0))[0] == 0, ('Body outside observed free space', point, index)
                samples += 1
        return {'path_length_m': length, 'sampled_positions': samples,
                'height_span_m': max(point[2] for point in path) - min(point[2] for point in path),
                'maximum_segment_slope_rad': maximum_slope,
                'maximum_height_error_from_fused_surface_m': max(errors),
                'maximum_height_error_from_input_surface_m': max(source_errors)}

    def export(self, directory):
        with (directory / 'tsdf_snapshot.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['x', 'y', 'z', 'occupancy', 'tsdf_distance', 'tsdf_weight'])
            writer.writerows(self.rows)
        with (directory / 'surface.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['x', 'y', 'input_surface_z', 'tsdf_surface_z'])
            for (ix, iy), z in sorted(self.surfaces.items()):
                x, y = ORIGIN[0] + (ix + .5) * RESOLUTION, ORIGIN[1] + (iy + .5) * RESOLUTION
                writer.writerow([x, y, surface(x, y), z])


def export_route(path, points, terrain):
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['x', 'y', 'base_z', 'projected_surface_z', 'tsdf_surface_z', 'input_surface_z'])
        for x, y, z in points:
            writer.writerow([x, y, z, z - BASE_HEIGHT,
                             terrain.reconstructed_surface(x, y), surface(x, y)])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--start-node', action='store_true')
    parser.add_argument('--output', type=Path, default=Path('results/tsdf_ground/summary.json'))
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = {'status': 'running', 'backend': 'voxblox_tsdf', 'scans': [],
              'scene': {'origin': ORIGIN, 'dimensions': DIMENSIONS, 'resolution_m': RESOLUTION,
                        'input_surface': 'z = 0.13 + 0.12 * max(0, x + 0.6)',
                        'scan_origins': SCAN_ORIGINS, 'return_spacing_m': .04},
              'base_height_m': BASE_HEIGHT, 'planning_height_m': PLANNING_HEIGHT,
              'robot_size_m': [ROBOT_SIZE] * 3}
    node = process = log = None
    try:
        config = Path(get_package_share_directory('gbplanner_local')) / 'config/tsdf_ground.yaml'
        if args.start_node:
            log = (args.output.parent / 'nodes.log').open('w')
            process = subprocess.Popen(
                ['ros2', 'launch', 'gbplanner_voxblox', 'mapping.launch.py',
                 'sensor_config:=' + str(config), 'global_planning:=true'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = GlobalDemo()
        mapper = node.create_client(Trigger, 'mapper_status')
        gain = node.create_client(Trigger, 'sensor_gain')
        validator = node.create_client(ValidatePath, 'validate_path')
        for client in (mapper, gain, validator):
            assert client.wait_for_service(timeout_sec=15)
        read_mapper = lambda: json.loads(node.call(mapper, Trigger.Request()).message)
        assert read_mapper()['integrated_scans'] == 0, 'This experiment requires an empty Voxblox map'
        clouds = node.create_publisher(PointCloud2, 'points', qos_profile_sensor_data)
        snapshots, local_graphs = [], []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(PointCloud2, 'voxel_map', snapshots.append, qos)
        node.create_subscription(Graph, 'local_graph', local_graphs.append, qos)
        broadcaster = TransformBroadcaster(node)
        node.spin_until(lambda: clouds.get_subscription_count() > 0)
        for ordinal, origin in enumerate(SCAN_ORIGINS, start=1):
            stamp = node.get_clock().now().to_msg()
            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'terrain_sensor'
            transform.header.stamp = stamp
            (transform.transform.translation.x, transform.transform.translation.y,
             transform.transform.translation.z) = origin
            transform.transform.rotation.w = 1.0
            broadcaster.sendTransform(transform)
            ready_at = time.monotonic() + .25
            node.spin_until(lambda: time.monotonic() >= ready_at)
            cloud = terrain_returns(stamp, origin)
            previous = node.revision()['map_revision']
            clouds.publish(cloud)
            node.spin_until(lambda: read_mapper()['integrated_scans'] >= ordinal, seconds=45)
            node.spin_until(lambda: bool(snapshots) and snapshots[-1].header.stamp == stamp
                            and node.revision()['map_revision'] > previous)
            mapper_status = read_mapper()
            assert mapper_status['rejected_scans'] == 0, mapper_status
            report['scans'].append({'origin': origin, 'returns': cloud.width,
                                    'input_sha256': hashlib.sha256(bytes(cloud.data)).hexdigest(),
                                    'mapper': mapper_status})
            save(args.output, report)
            print(json.dumps({'scan': ordinal, 'known_cells': mapper_status['known_cells']}), flush=True)
        raw = snapshots[-1]
        terrain = ObservedTsdf(raw)
        terrain.export(args.output.parent)
        status = node.revision()
        assert status['tsdf_fields'] and status['ground_projection'] == 'tsdf', status
        assert status['tsdf_cells'] == sum(row[5] > 0 for row in terrain.rows), status
        assert status['tsdf_cells'] > 0 and status['rejected_maps'] == 0, status
        surface_errors = [abs(height - surface(ORIGIN[0] + (index[0] + .5) * RESOLUTION,
                                               ORIGIN[1] + (index[1] + .5) * RESOLUTION))
                          for index, height in terrain.surfaces.items()]
        report['map'] = {'status': status, 'sha256': hashlib.sha256(bytes(raw.data)).hexdigest(),
                         'snapshot': 'tsdf_snapshot.csv', 'surface': 'surface.csv',
                         'zero_crossing_columns': len(terrain.surfaces),
                         'surface_error_m': {'maximum': max(surface_errors),
                                             'mean': sum(surface_errors) / len(surface_errors)},
                         'point_step': raw.point_step,
                         'fields': [{'name': field.name, 'offset': field.offset,
                                     'datatype': field.datatype} for field in raw.fields]}

        def move(position, yaw=0.0):
            message = Odometry()
            message.header.frame_id = 'map'
            message.header.stamp = node.get_clock().now().to_msg()
            (message.pose.pose.position.x, message.pose.pose.position.y,
             message.pose.pose.position.z) = [float(value) for value in position]
            message.pose.pose.orientation.z = math.sin(yaw / 2)
            message.pose.pose.orientation.w = math.cos(yaw / 2)
            node.spin_until(lambda: node.odometry.get_subscription_count() > 0)
            node.odometry.publish(message)
            def received():
                result = node.call(gain, Trigger.Request())
                if not result.success:
                    return False
                root = json.loads(result.message)['root']
                expected = [position[0], position[1], position[2] + PLANNING_HEIGHT - BASE_HEIGHT]
                return math.dist(root[:3], expected) < 1e-8
            node.spin_until(received)

        home = [*ROOT_XY, surface(*ROOT_XY) + BASE_HEIGHT]
        move(home)
        response, metrics = node.plan()
        local_path = xyz(response.path)
        report['local'] = {'metrics': metrics, 'path': local_path}
        node.spin_until(lambda: bool(local_graphs))
        report['local']['graph'] = graph_record(local_graphs[-1])
        save(args.output, report)
        assert metrics['success'] and metrics['ground_projection'] == 'tsdf', metrics
        assert metrics['tsdf_cells'] == status['tsdf_cells'], metrics
        assert len(local_path) >= 2 and math.dist(local_path[0][:2], ROOT_XY) < 1e-8
        local_check = terrain.check_path(local_path)
        assert local_check['height_span_m'] > .05, local_check
        request = ValidatePath.Request()
        request.header.frame_id = 'map'
        request.path = response.path
        checked = node.call(validator, request)
        assert checked.valid and checked.map_revision == status['map_revision'], checked.reason
        report['local'].update(terrain_checks=local_check,
                               validation={'valid': checked.valid, 'reason': checked.reason,
                                           'map_revision': checked.map_revision})
        export_route(args.output.parent / 'local_path.csv', local_path, terrain)
        save(args.output, report)

        report['odometry'] = [home]
        previous = home
        for point in local_path[1:]:
            move(point, math.atan2(point[1] - previous[1], point[0] - previous[0]))
            report['odometry'].append(point)
            previous = point
        response, metrics = node.global_call(homing=True)
        homing_path = xyz(response.path)
        report['homing'] = {'metrics': metrics, 'path': homing_path}
        save(args.output, report)
        assert metrics['success'], metrics
        assert math.dist(homing_path[0][:2], local_path[-1][:2]) < 1e-8
        assert math.dist(homing_path[-1][:2], ROOT_XY) < 1e-8
        homing_check = terrain.check_path(homing_path)
        request.path = response.path
        checked = node.call(validator, request)
        assert checked.valid and checked.map_revision == status['map_revision'], checked.reason
        report['homing'].update(terrain_checks=homing_check,
                                validation={'valid': checked.valid, 'reason': checked.reason,
                                            'map_revision': checked.map_revision})
        export_route(args.output.parent / 'homing_path.csv', homing_path, terrain)
        report.update(status='passed', checks={
            'raw_xyz_returns_integrated_by_original_voxblox': True,
            'observed_tsdf_and_occupancy_share_one_snapshot': True,
            'planner_received_tsdf_fields': True,
            'local_planner_uses_tsdf_projection': True,
            'selected_path_changes_height': True,
            'local_and_homing_paths_follow_fused_surface': True,
            'robot_body_stays_in_observed_free_space': True,
            'native_validate_path_accepts_both_routes': True})
        save(args.output, report)
        print(json.dumps({'status': report['status'], 'output': str(args.output),
                          'path_length_m': local_check['path_length_m'],
                          'height_span_m': local_check['height_span_m']}), flush=True)
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
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        if log is not None:
            log.close()


if __name__ == '__main__':
    main()
