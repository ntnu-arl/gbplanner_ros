#!/usr/bin/env python3
"""Hold map/root/yaw fixed and change route choice through odometry history."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from planner_msgs.msg import Graph
from planner_msgs.srv import PlanAhead, ValidatePath
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from local_demo import Demo, make_map, xyz
from mapping_demo import inspect_snapshot
from rotated_sampling_demo import verify_path

ROOT = (0.0, 0.0, 0.0)
ROOT_YAW = math.pi / 2.0
HISTORIES = {'positive_x': [-4.0, -3.2, -2.4, -1.6, -0.8, 0.0],
             'negative_x': [4.0, 3.2, 2.4, 1.6, 0.8, 0.0]}


def pose(x=0.0, y=0.0, z=0.0, yaw=ROOT_YAW):
    result = Pose()
    result.position.x, result.position.y, result.position.z = x, y, z
    result.orientation.z = math.sin(yaw / 2.0)
    result.orientation.w = math.cos(yaw / 2.0)
    return result


def pose_yaw(p):
    q = p.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_distance(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def graph_record(message):
    vertices = sorted(({'id': vertex.id,
                        'position': [vertex.pose.position.x, vertex.pose.position.y,
                                     vertex.pose.position.z],
                        'yaw': pose_yaw(vertex.pose)} for vertex in message.vertices),
                      key=lambda item: item['id'])
    edges = sorted([edge.source_id, edge.target_id, edge.weight] for edge in message.edges)
    return {'vertices': vertices, 'edges': edges}


def tracker_signature(status):
    # Holding the same position does not add history points or direction updates.
    return status['heading'], status['samples'], status['updates']


def save(path, report):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(report, indent=2) + '\n')
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('results/direction/summary.json'))
    parser.add_argument('--weight', type=float, default=2.0)
    args = parser.parse_args()
    if not math.isfinite(args.weight) or args.weight <= 0.0:
        raise ValueError('A positive direction weight is required for the comparison')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    command = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
               '-p', f'planning.path_direction_penalty:={args.weight}',
               '-p', 'gain.free:=1.0', '-p', 'global.enable:=false',
               '-p', 'sampling.seed:=42']
    report = {'status': 'running', 'scope': 'Native ROS 2 planning with stamped synthetic odometry history and a fixed free voxel map',
              'root': ROOT, 'root_yaw': ROOT_YAW, 'direction_weight': args.weight,
              'seed': 42, 'command': command, 'histories': HISTORIES, 'runs': []}
    node = process = log = timer = None
    try:
        log = (args.output.parent / 'nodes.log').open('w')
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        rclpy.init()
        node = Demo()
        direction = node.create_client(Trigger, 'exploration_direction')
        validator = node.create_client(ValidatePath, 'validate_path')
        ahead = node.create_client(PlanAhead, 'planner_ahead')
        for client in (direction, validator, ahead):
            assert client.wait_for_service(timeout_sec=15), 'Required service missing'

        def read_direction():
            response = node.call(direction, Trigger.Request())
            return json.loads(response.message)

        graphs = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Graph, 'local_graph', graphs.append, qos)
        current = [pose(HISTORIES['positive_x'][0])]

        def publish_odometry():
            message = Odometry()
            message.header.frame_id = 'map'
            message.header.stamp = node.get_clock().now().to_msg()
            message.pose.pose = current[0]
            node.odometry.publish(message)

        timer = node.create_timer(0.05, publish_odometry)
        node.spin_until(lambda: node.odometry.get_subscription_count() > 0)
        raw_map = make_map(known=True, wall=False)
        node.publish_map(raw_map)
        map_revision = node.revision()['map_revision']
        snapshot = inspect_snapshot(raw_map)
        report['map'] = {'revision': map_revision, 'known_cells': len(snapshot),
                         'occupancy': 'all free', 'resolution_m': 0.25,
                         'dimensions': [40, 40, 12]}

        for label, history in HISTORIES.items():
            history_log = []
            for index, x in enumerate(history):
                before = read_direction()
                current[0] = pose(x)
                publish_odometry()
                if label == 'positive_x' and index == 0:
                    node.spin_until(lambda: read_direction()['samples'] > 0)
                else:
                    node.spin_until(lambda: read_direction()['updates'] > before['updates'])
                accepted = read_direction()
                history_log.append({'position': [x, 0.0, 0.0], 'yaw': ROOT_YAW,
                                    'tracker': accepted})
            tracker = read_direction()
            expected_heading = 0.0 if label == 'positive_x' else math.pi * (1.0 - 0.3 ** 5)
            assert angle_distance(tracker['heading'], expected_heading) < 1e-8, tracker
            assert angle_distance(tracker['heading'], ROOT_YAW) > 1.0, tracker
            graph_before = len(graphs)
            response, metrics = node.plan()
            assert metrics['success'], metrics
            node.spin_until(lambda: len(graphs) > graph_before)
            graph = graph_record(graphs[-1])
            coordinates = xyz(response.path)
            assert len(coordinates) >= 2 and math.dist(coordinates[0], ROOT) < 1e-10, coordinates
            assert angle_distance(pose_yaw(response.path[0]), ROOT_YAW) < 1e-10
            assert metrics['map_revision'] == map_revision
            assert angle_distance(metrics['exploring_direction'], tracker['heading']) < 1e-9, metrics
            assert metrics['unpenalized_gain'] > 0.0 and metrics['direction_distance'] >= 0.0
            expected_score = metrics['unpenalized_gain'] * math.exp(
                -args.weight * metrics['direction_distance'])
            assert math.isclose(metrics['gain'], expected_score, rel_tol=1e-9, abs_tol=1e-8), metrics
            verify_path(coordinates, snapshot)
            request = ValidatePath.Request()
            request.header.frame_id = 'map'
            request.path = response.path
            checked = node.call(validator, request)
            assert checked.valid and checked.reason == 'ok', checked.reason
            assert checked.map_revision == map_revision
            assert tracker_signature(read_direction()) == tracker_signature(tracker)
            run = {'history': label, 'accepted_odometry': history_log, 'tracker': tracker,
                   'metrics': metrics, 'root_yaw': pose_yaw(response.path[0]), 'path': coordinates,
                   'score_recomputed': expected_score,
                   'validation': {'valid': checked.valid, 'reason': checked.reason,
                                  'map_revision': checked.map_revision},
                   'graph': graph}
            report['runs'].append(run)
            save(args.output, report)
            print(json.dumps({'history': label, 'heading': tracker['heading'],
                              'endpoint': coordinates[-1], 'gain': metrics['gain']}), flush=True)

        first, second = report['runs']
        assert first['graph'] == second['graph'], 'History must not change the sampled graph'
        assert first['path'] != second['path'], 'Direction history did not change route selection'
        assert first['path'][-1][0] > 0.0 and second['path'][-1][0] < 0.0, (
            first['path'][-1], second['path'][-1])
        for run in report['runs']:
            delta = [run['path'][-1][i] - ROOT[i] for i in range(2)]
            heading = run['tracker']['heading']
            assert delta[0] * math.cos(heading) + delta[1] * math.sin(heading) > 0.0
        # Both APIs may evaluate other roots, but only received odometry may
        # update the history tracker used by a later real-root request.
        # Stop publishing before these calls so a newer real odometry packet
        # cannot conceal an accidental assignment of a hypothetical root.
        timer.cancel()
        def settle(seconds):
            deadline = time.monotonic() + seconds
            node.spin_until(lambda: time.monotonic() >= deadline)
        settle(0.3)
        before = read_direction()
        hypothetical = pose(0.0, -1.0, 0.0, -math.pi / 2.0)
        explicit_response, explicit_metrics = node.plan(root=hypothetical)
        assert explicit_metrics['success'], explicit_metrics
        assert math.dist(xyz(explicit_response.path)[0], [0.0, -1.0, 0.0]) < 1e-10
        settle(0.55)
        assert tracker_signature(read_direction()) == tracker_signature(before)
        request = PlanAhead.Request()
        request.header.frame_id = 'map'
        request.root_pose = pose(0.0, 1.0, 0.0, -math.pi / 2.0)
        request.arrival_time = 0.5
        ahead_response = node.call(ahead, request)
        assert ahead_response.success and ahead_response.reason == 'ok', ahead_response.reason
        assert math.dist(xyz(ahead_response.path)[0], [0.0, 1.0, 0.0]) < 1e-10
        settle(0.55)
        after = read_direction()
        assert tracker_signature(after) == tracker_signature(before)
        repeated, repeated_metrics = node.plan()
        assert repeated_metrics['success'] and xyz(repeated.path) == second['path']
        assert angle_distance(pose_yaw(repeated.path[0]), ROOT_YAW) < 1e-10
        report['hypothetical_roots'] = {
            'before': before, 'after': after,
            'explicit': {'root': [0.0, -1.0, 0.0, -math.pi / 2.0],
                         'metrics': explicit_metrics, 'path': xyz(explicit_response.path)},
            'plan_ahead': {'root': [0.0, 1.0, 0.0, -math.pi / 2.0],
                           'success': ahead_response.success, 'reason': ahead_response.reason,
                           'map_revision': ahead_response.map_revision,
                           'path': xyz(ahead_response.path)},
            'repeated_actual_root_metrics': repeated_metrics}
        # Retain one common graph so the result can be plotted and inspected.
        report['graph'] = first['graph']
        for run in report['runs']:
            del run['graph']
        for ancestor in Path(__file__).resolve().parents:
            if (ancestor / 'src/gbplanner_local').is_dir():
                relative_sources = ['src/gbplanner_local/src/local_planner.cpp',
                                    'src/gbplanner_local/src/local_planner_node.cpp',
                                    'src/gbplanner_local/src/trajectory.cpp',
                                    'src/gbplanner_local/include/gbplanner_local/exploration_direction.hpp',
                                    'src/gbplanner_local/include/gbplanner_local/parameters.hpp',
                                    'src/gbplanner_local/scripts/direction_demo.py']
                report['source_sha256'] = {
                    name: hashlib.sha256((ancestor / name).read_bytes()).hexdigest()
                    for name in relative_sources}
                break
        report.update(status='passed', checks={
            'actual_odometry_updates_direction': True,
            'same_final_root_and_yaw': True, 'identical_sampled_graph': True,
            'different_selected_route': True, 'route_follows_history_direction': True,
            'exponential_score_matches': True, 'paths_validated': True,
            'explicit_root_keeps_actual_history': True, 'plan_ahead_keeps_actual_history': True,
            'real_root_request_repeats_after_hypothetical_plans': True})
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
