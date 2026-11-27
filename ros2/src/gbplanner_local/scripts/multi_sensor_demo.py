#!/usr/bin/env python3
"""Fuse two independent half-view lidars, then plan through the observed room."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from planner_msgs.srv import ValidatePath
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from local_demo import Demo, xyz
from mapping_demo import inspect_snapshot, lidar_cloud, verify_path

BODY = (-2.0, 0.0, 0.0)
SENSORS = {
    'front': {'topic': '/front/points', 'frame': 'front_lidar',
              'offset': (0.2, 0.0, 0.0), 'yaw': 0.0},
    'rear': {'topic': '/rear/points', 'frame': 'rear_lidar',
             'offset': (-0.2, 0.0, 0.0), 'yaw': math.pi},
}


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def half_cloud(stamp, sensor):
    # Ray casting uses the physical sensor position; each sensor sees only its
    # own forward hemisphere. No occupancy snapshot is injected into the mapper.
    origin = tuple(a + b for a, b in zip(BODY, sensor['offset']))
    cloud = lidar_cloud(stamp, sensor['frame'], origin, sensor['yaw'])
    data = bytes(cloud.data)
    cloud.data = b''.join(data[i:i + cloud.point_step]
                          for i in range(0, len(data), cloud.point_step)
                          if struct.unpack_from('<f', data, i)[0] >= 0.0)
    cloud.width = len(cloud.data) // cloud.point_step
    cloud.row_step = len(cloud.data)
    return cloud


def save_slice(snapshot, path):
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['x', 'y', 'status'])
        for y in range(40):
            for x in range(40):
                writer.writerow([-5 + (x + 0.5) * 0.25,
                                 -5 + (y + 0.5) * 0.25,
                                 snapshot.get((x, y, 6), -1)])


def write_report(path, report):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(report, indent=2) + '\n')
    temporary.replace(path)


def inspect_gain(message):
    assert message.success, message.message
    gains = json.loads(message.message)
    assert {sensor['name'] for sensor in gains['sensors']} == set(SENSORS), gains
    gain_sum = sum(sensor['gain'] for sensor in gains['sensors'])
    frontier_or = any(sensor['frontier'] for sensor in gains['sensors'])
    assert math.isclose(gain_sum, gains['total_gain'], rel_tol=1e-9, abs_tol=1e-8), gains
    assert frontier_or == gains['frontier'], gains
    for sensor in gains['sensors']:
        expected = SENSORS[sensor['name']]
        assert all(math.isclose(a, b, abs_tol=1e-8)
                   for a, b in zip(sensor['center_offset'], expected['offset'])), sensor
        assert math.isclose(sensor['rotations'][0], expected['yaw'], abs_tol=1e-8), sensor
    gains['summed_voxel_counts'] = {
        key: sum(sensor[key] for sensor in gains['sensors'])
        for key in ('unknown', 'free', 'occupied')
    }
    return gains


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=('occupancy', 'voxblox'), default='occupancy')
    parser.add_argument('--start-node', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or Path('results/multi_sensor') / args.backend / 'summary.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    report = {'status': 'running', 'backend': args.backend,
              'body_pose': [*BODY, 0.0], 'sensors': SENSORS, 'stages': []}
    process = node = log = None
    try:
        if args.start_node:
            log = (output.parent / 'nodes.log').open('w')
            process = subprocess.Popen(
                ['ros2', 'launch', 'gbplanner_local', 'multi_sensor.launch.py',
                 'backend:=' + args.backend], stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
        rclpy.init()
        node = Demo()
        # Demo supplies the standard odometry/planning clients. Its map publisher
        # is deliberately unused: all map cells below come from the two clouds.
        mapper = node.create_client(Trigger, 'mapper_status')
        gain_client = node.create_client(Trigger, 'sensor_gain')
        validation = node.create_client(ValidatePath, 'validate_path')
        for client in (mapper, gain_client, validation):
            assert client.wait_for_service(timeout_sec=15), 'Required service missing'
        read_mapper = lambda: json.loads(node.call(mapper, Trigger.Request()).message)
        initial = read_mapper()
        assert initial['integrated_scans'] == 0, 'Start this demo with an empty mapper'
        assert set(initial['input_sources']) == set(SENSORS), initial
        publishers = {name: node.create_publisher(PointCloud2, sensor['topic'],
                                                  qos_profile_sensor_data)
                      for name, sensor in SENSORS.items()}
        snapshots = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(PointCloud2, 'voxel_map', snapshots.append, qos)
        broadcaster = StaticTransformBroadcaster(node)
        transforms = []
        for name, sensor in SENSORS.items():
            source = initial['input_sources'][name]
            assert source['topic'] == sensor['topic'] and source['frame'] == sensor['frame'], source
            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.child_frame_id = sensor['frame']
            transform.header.stamp = node.get_clock().now().to_msg()
            position = tuple(a + b for a, b in zip(BODY, sensor['offset']))
            (transform.transform.translation.x, transform.transform.translation.y,
             transform.transform.translation.z) = position
            transform.transform.rotation.z = math.sin(sensor['yaw'] / 2.0)
            transform.transform.rotation.w = math.cos(sensor['yaw'] / 2.0)
            transforms.append(transform)
        broadcaster.sendTransform(transforms)

        def publish_odometry():
            odometry = Odometry()
            odometry.header.frame_id = 'map'
            odometry.header.stamp = node.get_clock().now().to_msg()
            (odometry.pose.pose.position.x, odometry.pose.pose.position.y,
             odometry.pose.pose.position.z) = BODY
            odometry.pose.pose.orientation.w = 1.0
            node.odometry.publish(odometry)

        node.create_timer(0.1, publish_odometry)
        node.spin_until(lambda: all(pub.get_subscription_count() for pub in publishers.values())
                        and node.odometry.get_subscription_count() > 0)
        # Allow the reliable static transforms and first odometry sample to reach
        # the independently spinning mapper before publishing exactly one scan.
        ready_at = time.monotonic() + 1.0
        node.spin_until(lambda: time.monotonic() >= ready_at)
        previous_snapshot = {}
        combined_snapshot = None
        for ordinal, (name, sensor) in enumerate(SENSORS.items(), start=1):
            cloud = half_cloud(node.get_clock().now().to_msg(), sensor)
            cloud_stamp = stamp_ns(cloud.header.stamp)
            revision_before = node.revision()['map_revision']
            publishers[name].publish(cloud)
            node.spin_until(lambda: read_mapper()['input_sources'][name]['integrated_scans'] == 1)
            node.spin_until(lambda: any(stamp_ns(s.header.stamp) == cloud_stamp for s in snapshots)
                            and node.revision()['map_revision'] > revision_before)
            raw = next(s for s in reversed(snapshots) if stamp_ns(s.header.stamp) == cloud_stamp)
            snapshot = inspect_snapshot(raw)
            status = read_mapper()
            known = {key for key, value in snapshot.items() if value in (0, 100)}
            previous_known = {key for key, value in previous_snapshot.items() if value in (0, 100)}
            newly_known = len(known - previous_known)
            assert status['integrated_scans'] == ordinal and status['rejected_scans'] == 0, status
            assert status['known_cells'] == len(known), status
            assert newly_known > 0, (name, 'scan did not extend the shared map')
            assert all(source['rejected_scans'] == 0 for source in status['input_sources'].values()), status
            slice_name = 'front_map.csv' if name == 'front' else 'combined_map.csv'
            save_slice(snapshot, output.parent / slice_name)
            stage = {'source': name, 'input_returns': cloud.width,
                     'input_stamp_ns': cloud_stamp, 'map_stamp_ns': stamp_ns(raw.header.stamp),
                     'mapper': status, 'newly_known_cells': newly_known,
                     'free_cells': sum(value == 0 for value in snapshot.values()),
                     'occupied_cells': sum(value == 100 for value in snapshot.values()),
                     'map_slice': slice_name,
                     'sensor_gain': inspect_gain(node.call(gain_client, Trigger.Request()))}
            report['stages'].append(stage)
            write_report(output, report)
            print(json.dumps({'source': name, 'known_cells': len(known),
                              'newly_known_cells': newly_known, 'stamp_ns': cloud_stamp}), flush=True)
            previous_snapshot = combined_snapshot = snapshot

        assert all(source['integrated_scans'] == 1
                   for source in report['stages'][-1]['mapper']['input_sources'].values())
        response, metrics = node.plan()
        assert metrics['success'], metrics
        coordinates = xyz(response.path)
        assert len(coordinates) >= 2 and math.dist(coordinates[0], BODY) < 1e-8, coordinates
        verify_path(coordinates, combined_snapshot)
        request = ValidatePath.Request()
        request.header.frame_id = 'map'
        request.path = response.path
        checked = node.call(validation, request)
        assert checked.valid and checked.reason == 'ok', checked.reason
        assert checked.map_revision == node.revision()['map_revision']
        with (output.parent / 'path.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['x', 'y', 'z'])
            writer.writerows(coordinates)
        report.update(status='passed', planning=metrics, path=coordinates,
                      validation={'valid': checked.valid, 'reason': checked.reason,
                                  'map_revision': checked.map_revision},
                      checks={'independent_point_cloud_topics': True,
                              'different_frames_offsets_and_yaws': True,
                              'rear_scan_adds_observed_cells': True,
                              'each_source_integrated_once': True,
                              'map_preserves_each_acquisition_stamp': True,
                              'gain_is_sensor_sum': True, 'frontier_is_sensor_or': True,
                              'path_in_observed_free_space': True,
                              'independent_wall_check': True})
        write_report(output, report)
        print(json.dumps({'status': report['status'], 'backend': args.backend,
                          'path_length_m': metrics['path_length_m'], 'output': str(output)}), flush=True)
    except Exception as error:
        report.update(status='failed', error=str(error))
        write_report(output, report)
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
