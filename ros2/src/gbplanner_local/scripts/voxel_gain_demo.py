#!/usr/bin/env python3
"""Compare native per-voxel and ray gain over one wall-and-frontier scene."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from planner_msgs.srv import ValidatePath
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from local_demo import Demo, xyz
from mapping_demo import inspect_snapshot
from rotated_sampling_demo import verify_path

ORIGIN = (-5.0, -5.0, -1.5)
DIMENSIONS = (40, 40, 12)
RESOLUTION = 0.25
ROOT = (-2.0, 0.0, 0.0)
CAMERA_YAW_FORWARD = 0.0
CAMERA_YAW_BACK = math.pi
WALL_X = 0.0
WALL_GAP_HALF_WIDTH = 1.0


def status_at(index, full_wall=False):
    x, y, z = [base + (value + 0.5) * RESOLUTION
               for base, value in zip(ORIGIN, index)]
    if full_wall and abs(x - WALL_X) <= RESOLUTION / 2.0:
        return 100
    if abs(x - WALL_X) <= RESOLUTION / 2.0 and abs(y) > WALL_GAP_HALF_WIDTH:
        return 100
    # Known floor, ceiling and side limits; free volume around the root.
    if z <= -1.25 or z >= 1.25 or abs(y) >= 4.75 or x <= -4.75 or x >= 4.75:
        return 100
    # Open forward side beyond the wall ends in an unobserved region.
    if x >= 1.5:
        return -1
    return 0


def make_map(stamp, full_wall=False):
    message = PointCloud2()
    message.header.frame_id = 'map'
    message.header.stamp = stamp
    message.height = 1
    message.width = math.prod(DIMENSIONS)
    message.fields = [PointField(name=name, offset=i * 4,
                                 datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(('x', 'y', 'z'))]
    message.fields.append(PointField(name='occupancy', offset=12,
                                     datatype=PointField.INT8, count=1))
    message.point_step = 16
    message.row_step = message.width * message.point_step
    message.is_dense = True
    data = bytearray(message.row_step)
    offset = 0
    for z in range(DIMENSIONS[2]):
        for y in range(DIMENSIONS[1]):
            for x in range(DIMENSIONS[0]):
                center = [base + (value + 0.5) * RESOLUTION
                          for base, value in zip(ORIGIN, (x, y, z))]
                status = status_at((x, y, z), full_wall)
                struct.pack_into('<fffbxxx', data, offset, *center, status)
                offset += message.point_step
    message.data = bytes(data)
    return message


def make_pose(yaw):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = ROOT
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def gain_record(node, yaw, mode):
    response = node.call(node.gain_client, Trigger.Request())
    assert response.success, response.message
    data = json.loads(response.message)
    assert len(data['sensors']) == 1 and data['sensors'][0]['name'] == 'camera', data
    assert data['root'][:3] == list(ROOT), data
    assert abs(math.atan2(math.sin(data['root'][3] - yaw),
                          math.cos(data['root'][3] - yaw))) < 1e-9, data
    sensor = data['sensors'][0]
    assert sensor['gain_model'] == mode, sensor
    if mode == 'voxel':
        assert sensor['rays'] == 0, sensor
        assert 0 < sensor['visibility_queries'] <= sensor['fov_voxel_count'] <= sensor['voxel_count'], sensor
        assert sum(sensor[key] for key in ('unknown', 'free', 'occupied')) <= sensor['visibility_queries'], sensor
    else:
        assert sensor['rays'] > 0, sensor
        assert all(sensor[key] == 0 for key in ('voxel_count', 'fov_voxel_count', 'visibility_queries')), sensor
    return data


def save(path, report):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(report, indent=2) + '\n')
    temporary.replace(path)


def map_slice(snapshot, path):
    with path.open('w', newline='') as stream:
        import csv
        writer = csv.writer(stream)
        writer.writerow(['x', 'y', 'status'])
        for y in range(DIMENSIONS[1]):
            for x in range(DIMENSIONS[0]):
                index = (x, y, int((0.0 - ORIGIN[2]) / RESOLUTION))
                writer.writerow([ORIGIN[0] + (x + .5) * RESOLUTION,
                                 ORIGIN[1] + (y + .5) * RESOLUTION,
                                 snapshot.get(index, -1)])


def run_process(mode, output_dir):
    namespace = '/voxel_gain_' + mode
    command = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
               '-r', '__ns:=' + namespace, '-p', 'sampling.seed:=42',
               '-p', 'global.enable:=false', '-p', 'sensor.names:=[camera]',
               '-p', 'planning.exploration_sensors:=[camera]',
               '-p', 'sensor.camera.type:=kCamera', '-p', 'sensor.camera.range:=6.0',
               '-p', 'sensor.camera.horizontal_fov:=1.5707963267948966',
               '-p', 'sensor.camera.vertical_fov:=1.0471975511965976',
               '-p', 'sensor.camera.horizontal_resolution:=0.2617993877991494',
               '-p', 'sensor.camera.vertical_resolution:=0.2617993877991494',
               '-p', 'gain.unknown:=1.0', '-p', 'gain.free:=0.1', '-p', 'gain.occupied:=0.0',
               '-p', 'sampling.min:=[-2.0, -2.5, -1.0]', '-p', 'sampling.max:=[5.0, 2.5, 1.0]',
               '-p', 'planning.exp_gain_voxel_size:=0.25',
               '-p', f'planning.use_ray_model_for_volumetric_gain:={"true" if mode == "ray" else "false"}']
    node = process = timer = log = None
    try:
        log = (output_dir / (mode + '_nodes.log')).open('w')
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        rclpy.init(args=['--ros-args', '-r', '__ns:=' + namespace])
        node = Demo()
        node.gain_client = node.create_client(Trigger, 'sensor_gain')
        frusta = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(MarkerArray, 'sensor_frusta',
                                 lambda message: frusta.append(message), qos)
        validator = node.create_client(ValidatePath, 'validate_path')
        assert node.gain_client.wait_for_service(timeout_sec=15)
        assert validator.wait_for_service(timeout_sec=15)

        root_pose = make_pose(CAMERA_YAW_FORWARD)
        def publish_odometry():
            message = Odometry()
            message.header.frame_id = 'map'
            message.header.stamp = node.get_clock().now().to_msg()
            message.pose.pose = root_pose
            node.odometry.publish(message)
        timer = node.create_timer(0.05, publish_odometry)
        node.spin_until(lambda: node.odometry.get_subscription_count() > 0)
        snapshot_msg = make_map(node.get_clock().now().to_msg())
        snapshot = inspect_snapshot(snapshot_msg)
        sha = hashlib.sha256(bytes(snapshot_msg.data)).hexdigest()
        node.publish_map(snapshot_msg)
        node.spin_until(lambda: node.call(node.gain_client, Trigger.Request()).success)
        forward = gain_record(node, CAMERA_YAW_FORWARD, mode)
        assert forward['total_gain'] > 0 and forward['sensors'][0]['unknown'] > 0, forward
        node.spin_until(lambda: bool(frusta))
        assert len(frusta[-1].markers) == 1, frusta[-1]
        marker = frusta[-1].markers[0]
        assert marker.header.frame_id == 'map' and marker.type == Marker.LINE_LIST, marker
        assert marker.action == Marker.ADD and len(marker.points) == 16, marker
        assert math.dist([marker.points[0].x, marker.points[0].y, marker.points[0].z], ROOT) < 1e-9
        frustum = {'frame_id': marker.header.frame_id, 'line_count': len(marker.points) // 2,
                   'points': [[point.x, point.y, point.z] for point in marker.points]}

        def set_heading(yaw):
            root_pose.orientation.z = math.sin(yaw / 2.0)
            root_pose.orientation.w = math.cos(yaw / 2.0)
            publish_odometry()
            def heading_received():
                response = node.call(node.gain_client, Trigger.Request())
                if not response.success:
                    return False
                actual = json.loads(response.message)['root'][3]
                return abs(math.atan2(math.sin(actual - yaw), math.cos(actual - yaw))) < 1e-9
            node.spin_until(heading_received)

        orientation = []
        baseline_revision = node.revision()['map_revision']
        for label, yaw in [('forward', CAMERA_YAW_FORWARD), ('back', CAMERA_YAW_BACK)]:
            set_heading(yaw)
            gains = gain_record(node, yaw, mode)
            orientation.append({'heading': label, 'yaw': yaw, 'diagnostics': gains})
        assert orientation[0]['diagnostics']['sensors'][0]['unknown'] > \
               orientation[1]['diagnostics']['sensors'][0]['unknown'], orientation

        # Both wall comparisons and planning use the original forward viewpoint.
        set_heading(CAMERA_YAW_FORWARD)
        plan = None
        if mode == 'voxel':
            response, metrics = node.plan()
            assert metrics['success'] and metrics['gain'] > 0, metrics
            assert metrics['gain_model'] == 'voxel' and metrics['exp_gain_voxel_size'] == RESOLUTION, metrics
            assert 0 < metrics['gain_visibility_queries'] <= metrics['gain_fov_voxel_count'] <= metrics['gain_voxel_count'], metrics
            points = xyz(response.path)
            assert len(points) >= 2 and math.dist(points[0], ROOT) < 1e-9
            verify_path(points, snapshot)
            request = ValidatePath.Request()
            request.header.frame_id = 'map'
            request.path = response.path
            checked = node.call(validator, request)
            assert checked.valid and checked.reason == 'ok', checked.reason
            assert checked.map_revision == baseline_revision
            plan = {'metrics': metrics, 'path': points,
                    'validation': {'valid': checked.valid, 'reason': checked.reason,
                                   'map_revision': checked.map_revision}}

        full_wall = make_map(node.get_clock().now().to_msg(), full_wall=True)
        node.publish_map(full_wall)
        blocked = gain_record(node, CAMERA_YAW_FORWARD, mode)
        assert blocked['sensors'][0]['unknown'] < forward['sensors'][0]['unknown'], (forward, blocked)
        restored_msg = make_map(node.get_clock().now().to_msg(), full_wall=False)
        node.publish_map(restored_msg)
        restored = gain_record(node, CAMERA_YAW_FORWARD, mode)
        assert restored['sensors'][0]['unknown'] == forward['sensors'][0]['unknown'], (forward, restored)
        assert restored['total_gain'] == forward['total_gain']

        result = {'mode': mode, 'command': command, 'map_sha256': sha,
                  'map_revision': baseline_revision, 'forward': forward,
                  'orientation_comparison': orientation,
                  'sensor_frustum': frustum,
                  'blocked_by_wall': blocked, 'restored': restored,
                  'planning': plan, 'map_slice': mode + '_map.csv'}
        map_slice(snapshot, output_dir / result['map_slice'])
        return result
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('results/voxel_gain/summary.json'))
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = {'status': 'running', 'root': ROOT, 'scene': {'origin': ORIGIN,
              'dimensions': DIMENSIONS, 'resolution_m': RESOLUTION,
              'wall_x': WALL_X, 'wall_gap_half_width': WALL_GAP_HALF_WIDTH,
              'unknown_start_x': 1.5, 'camera_fov': [math.pi / 2, math.pi / 3],
              'camera_range_m': 6.0}, 'cases': []}
    try:
        for mode in ('ray', 'voxel'):
            report['cases'].append(run_process(mode, args.output.parent))
            save(args.output, report)
        ray, voxel = report['cases']
        assert ray['map_sha256'] == voxel['map_sha256']
        assert ray['forward']['total_gain'] > 0 and voxel['forward']['total_gain'] > 0
        report.update(status='passed', checks={
            'both_models_use_same_full_3d_map': True,
            'per_voxel_mode_counts_candidates': True,
            'camera_orientation_changes_frontier_gain': True,
            'occupied_wall_occludes_unknown_region': True,
            'unblocked_map_restores_forward_gain': True,
            'voxel_mode_native_plan_has_positive_gain': True,
            'voxel_mode_plan_passes_native_validate_path': True,
            'per_voxel_and_ray_samples_are_model_specific': True})
        report['checks']['camera_frustum_published_from_odometry'] = True
        save(args.output, report)
    except Exception as error:
        report.update(status='failed', error=repr(error))
        save(args.output, report)
        raise
    print(json.dumps({'status': report['status'], 'output': str(args.output)}), flush=True)


if __name__ == '__main__':
    main()
