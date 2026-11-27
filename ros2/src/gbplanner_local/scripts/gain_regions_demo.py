#!/usr/bin/env python3
"""Compare native exploration with and without a room-wide no-gain region."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from planner_msgs.msg import Graph
from planner_msgs.srv import ValidatePath
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from local_demo import Demo, make_map, xyz
from mapping_demo import inspect_snapshot
from rotated_sampling_demo import verify_path

ROOT = (-2.0, 0.0, 0.0)
ZONE = {'name': 'room', 'type': 'kCuboid', 'center': [0.0, 0.0, 0.0],
        'minimum': [-5.0, -5.0, -1.5], 'maximum': [5.0, 5.0, 1.5],
        'rotations': [0.0, 0.0, 0.0]}
SAFE_PATH = [ROOT, (-1.0, 0.0, 0.0)]
WALL_PATH = [(-2.0, 2.0, 0.0), (1.0, 2.0, 0.0)]


def pose(point):
    result = Pose()
    result.position.x, result.position.y, result.position.z = point
    result.orientation.w = 1.0
    return result


def save(path, report):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(report, indent=2) + '\n')
    temporary.replace(path)


def graph_geometry(graph):
    return {'vertices': sorted([v.id, v.pose.position.x, v.pose.position.y, v.pose.position.z]
                               for v in graph.vertices),
            'edges': sorted([e.source_id, e.target_id, e.weight] for e in graph.edges)}


def run_case(label, enable_zone, output_directory):
    # Separate namespaces also prevent a previous process's durable output or
    # DDS discovery records from entering the second comparison.
    namespace = '/gain_regions_' + label
    command = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
               '-r', '__ns:=' + namespace, '-p', 'sampling.seed:=42',
               '-p', 'sensor.range:=6.0', '-p', 'global.enable:=false',
               '-p', 'planning.path_direction_penalty:=0.0']
    if enable_zone:
        command.extend(['-p', 'gain.no_gain_zones:=[room]', '-p', 'gain.room.type:=kCuboid',
                        '-p', 'gain.room.center:=[0.0, 0.0, 0.0]',
                        '-p', 'gain.room.minimum:=[-5.0, -5.0, -1.5]',
                        '-p', 'gain.room.maximum:=[5.0, 5.0, 1.5]',
                        '-p', 'gain.room.rotations:=[0.0, 0.0, 0.0]'])
        if label == 'sphere':
            command.extend(['-p', 'gain.room.type:=kSphere', '-p', 'gain.room.radius:=8.0'])
    node = process = log = timer = None
    try:
        log = (output_directory / (label + '_nodes.log')).open('w')
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        rclpy.init(args=['--ros-args', '-r', '__ns:=' + namespace])
        node = Demo()
        diagnostic = node.create_client(Trigger, 'sensor_gain')
        validator = node.create_client(ValidatePath, 'validate_path')
        for client in (diagnostic, validator):
            assert client.wait_for_service(timeout_sec=15), 'Required service missing'
        graphs = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Graph, 'local_graph', graphs.append, qos)

        def publish_odometry():
            message = Odometry()
            message.header.frame_id = 'map'
            message.header.stamp = node.get_clock().now().to_msg()
            message.pose.pose = pose(ROOT)
            node.odometry.publish(message)

        timer = node.create_timer(0.05, publish_odometry)
        node.spin_until(lambda: node.odometry.get_subscription_count() > 0)
        raw = make_map(known=False, wall=True)
        node.publish_map(raw)
        snapshot = inspect_snapshot(raw)
        node.spin_until(lambda: node.call(diagnostic, Trigger.Request()).success)
        gains = json.loads(node.call(diagnostic, Trigger.Request()).message)
        assert gains['root'][:3] == list(ROOT) and gains['root'][3] == 0.0, gains
        before = len(graphs)
        response, metrics = node.plan()
        node.spin_until(lambda: len(graphs) > before)
        coordinates = xyz(response.path)
        if enable_zone:
            assert gains['total_gain'] == 0.0 and not gains['frontier'], gains
            for sensor in gains['sensors']:
                assert all(sensor[key] == 0 for key in ('gain', 'unknown', 'free', 'occupied')), sensor
                assert not sensor['frontier'], sensor
            assert not metrics['success'] and metrics['reason'] == 'no_gain', metrics
            assert not coordinates, coordinates
        else:
            assert gains['total_gain'] > 0.0, gains
            assert metrics['success'] and metrics['gain'] > 0.0, metrics
            assert len(coordinates) >= 2 and math.dist(coordinates[0], ROOT) < 1e-10
            verify_path(coordinates, snapshot)
        # The same region contains this safe line and the physical wall. Reward
        # exclusion must preserve both free traversal and collision rejection.
        verify_path(SAFE_PATH, snapshot)
        checks = {}
        for name, points in [('safe_line', SAFE_PATH), ('through_wall', WALL_PATH)]:
            request = ValidatePath.Request()
            request.header.frame_id = 'map'
            request.path = [pose(point) for point in points]
            checked = node.call(validator, request)
            checks[name] = {'path': points, 'valid': checked.valid, 'reason': checked.reason,
                            'map_revision': checked.map_revision}
            if name == 'safe_line':
                assert checked.valid and checked.reason == 'ok', checked.reason
            else:
                assert not checked.valid and checked.reason == 'path_not_free', checked.reason
        zone = dict(ZONE) if enable_zone else None
        if label == 'sphere':
            zone.update(type='kSphere', radius=8.0)
        result = {'case': label, 'command': command, 'zone': zone,
                  'input_map_data_sha256': hashlib.sha256(bytes(raw.data)).hexdigest(),
                  'map_status': node.revision(), 'sensor_gain': gains, 'metrics': metrics,
                  'path': coordinates, 'validation': checks,
                  'graph': graph_geometry(graphs[-1])}
        print(json.dumps({'case': label, 'gain': gains['total_gain'],
                          'planning_reason': metrics['reason'],
                          'safe_line_valid': checks['safe_line']['valid'],
                          'wall_line_valid': checks['through_wall']['valid']}), flush=True)
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
    parser.add_argument('--output', type=Path, default=Path('results/gain_regions/summary.json'))
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = {'status': 'running', 'root': ROOT, 'root_yaw': 0.0, 'seed': 42,
              'scope': 'Native ROS 2 no-gain region comparison on a fixed wall map',
              'sensor_range_m': 6.0, 'cases': []}
    try:
        for label, enable_zone in [('baseline', False), ('whole_room', True), ('sphere', True)]:
            report['cases'].append(run_case(label, enable_zone, args.output.parent))
            save(args.output, report)
        baseline = report['cases'][0]
        for excluded in report['cases'][1:]:
            assert baseline['input_map_data_sha256'] == excluded['input_map_data_sha256']
            assert baseline['map_status'] == excluded['map_status']
            assert baseline['graph'] == excluded['graph'], 'Gain regions changed collision geometry or sampling'
        report['graph'] = baseline['graph']
        for case in report['cases']:
            del case['graph']
        report.update(status='passed', checks={
            'same_input_map_root_and_seed': True, 'identical_sampled_graph': True,
            'baseline_has_information_gain_and_path': True,
            'whole_map_region_excludes_all_sensor_gain': True,
            'whole_map_region_ends_planning_with_no_gain': True,
            'explicit_sphere_center_and_radius_work': True,
            'safe_path_inside_region_remains_valid': True,
            'physical_wall_inside_region_still_blocks_path': True})
        save(args.output, report)
    except Exception as error:
        report.update(status='failed', error=repr(error))
        save(args.output, report)
        raise
    print(json.dumps({'status': report['status'], 'output': str(args.output)}), flush=True)


if __name__ == '__main__':
    main()
