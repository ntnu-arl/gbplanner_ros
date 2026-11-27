#!/usr/bin/env python3
"""Cross-process marker tests; does not claim an RViz GUI render test."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from planner_msgs.msg import Graph
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray, Marker
from global_demo import GlobalDemo
from local_demo import make_map


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--start-node', action='store_true')
    parser.add_argument('--output', type=Path, default=Path('results/visualization_integration.json'))
    args = parser.parse_args()
    processes, node = [], None
    try:
        if args.start_node:
            processes.append(subprocess.Popen(['ros2', 'run', 'gbplanner_local', 'local_planner_node',
                '--ros-args', '-p', 'global.enable:=true'], start_new_session=True))
            processes.append(subprocess.Popen(['ros2', 'run', 'gbplanner_local', 'visualization_node.py',
                '--ros-args', '-p', 'max_map_points:=2000', '-p', 'history_limit:=3'], start_new_session=True))
        rclpy.init()
        node = GlobalDemo()
        client = node.create_client(Trigger, 'visualization_status')
        assert client.wait_for_service(timeout_sec=15)
        status = lambda: json.loads(node.call(client, Trigger.Request()).message)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        snapshots = {kind: [] for kind in ('local', 'global', 'map', 'robot')}
        topics = dict(local='vis/planning_graph', global_='vis/planning_global_graph',
                      map='vis/voxel_map', robot='vis/robot_state')
        topics['global'] = topics.pop('global_')
        for kind, topic in topics.items():
            node.create_subscription(MarkerArray, topic, lambda m, k=kind: snapshots[k].append(m), qos)
        base = make_map()
        node.publish_map(base)
        node.spin_until(lambda: status()['map_updates'] == 1)
        map_stats = status()
        assert map_stats['known_cells'] == node.revision()['known_cells']
        assert map_stats['displayed_cells'] == 2000 and map_stats['map_decimated']
        node.spin_until(lambda: snapshots['map'] and len(snapshots['map'][-1].markers) > 1)
        cells = snapshots['map'][-1].markers
        assert cells[0].action == Marker.DELETEALL
        occupied = [m for m in cells[1:] if m.id == 100][0]
        assert len(occupied.points) == 768, len(occupied.points)
        assert sum(len(m.points) for m in cells) == 2000
        for marker in cells[1:]:
            assert marker.type == Marker.CUBE_LIST and marker.scale.x == .25
            assert marker.header.frame_id == 'map' and marker.color.a > 0

        node.move_odometry([-2, 0, 0])
        response, metric = node.plan()
        assert metric['success']
        node.spin_until(lambda: status()['local_updates'] > 0 and
                        snapshots['local'] and len(snapshots['local'][-1].markers) > 1)
        graph_message = snapshots['local'][-1]
        edges = next(m for m in graph_message.markers if m.ns == 'local' and m.id == 0)
        vertices = next(m for m in graph_message.markers if m.ns == 'local' and m.id == 1)
        assert len(vertices.points) == metric['vertices'], metric
        assert len(edges.points) == 2 * metric['edges'], metric
        assert edges.type == Marker.LINE_LIST
        node.spin_until(lambda: snapshots['global'] and len(snapshots['global'][-1].markers) > 1)
        assert any(m.ns == 'global' and m.id == 2 and m.points for m in snapshots['global'][-1].markers)
        for snapshot in snapshots.values():
            for marker in snapshot[-1].markers:
                assert all(math.isfinite(v) for p in marker.points for v in (p.x, p.y, p.z))

        # No new planning occurs: the subscriber receives the retained scene.
        late = []
        node.create_subscription(MarkerArray, topics['local'], late.append, qos)
        node.spin_until(lambda: bool(late))
        assert late[-1] == graph_message

        for i in range(1, 6):
            before = status()['odom_updates']
            node.move_odometry([-2 + .1*i, 0, 0])
            node.spin_until(lambda: status()['odom_updates'] > before)
        assert status()['history_points'] == 3
        node.spin_until(lambda: snapshots['robot'] and
                       any(m.type == Marker.LINE_STRIP and len(m.points) == 3
                           for m in snapshots['robot'][-1].markers))

        # A rejected raw map never appears on the accepted-map display feed.
        before = status()['map_updates']
        invalid = copy.deepcopy(base)
        invalid.header.frame_id = 'wrong_frame'
        node.reject_map(invalid)
        deadline = time.monotonic() + .3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        assert status()['map_updates'] == before

        # Malformed graph input cannot clear a previously valid scene.
        fake = node.create_publisher(Graph, 'local_graph', qos)
        node.spin_until(lambda: fake.get_subscription_count() > 0)
        bad_graph = Graph()
        bad_graph.header.frame_id = 'wrong_frame'
        previous = status()
        fake.publish(bad_graph)
        node.spin_until(lambda: status()['rejected'] == previous['rejected'] + 1)
        assert status()['local_updates'] == previous['local_updates']
        assert snapshots['local'][-1] == graph_message
        node.destroy_publisher(fake)

        response, metric = node.global_call(homing=True)
        assert metric['success']
        reset = copy.deepcopy(base)
        data = bytearray(reset.data)
        for offset in range(12, len(data), reset.point_step):
            data[offset] = 255
        reset.data = bytes(data)
        node.publish_map(reset)
        node.spin_until(lambda: status()['known_cells'] == 0 and status()['history_points'] == 0)
        node.spin_until(lambda: all(snapshots[k] and len(snapshots[k][-1].markers) == 1
                                   and snapshots[k][-1].markers[0].action == Marker.DELETEALL
                                   for k in snapshots))
        node.spin_until(lambda: node.paths and not node.paths[-1].poses and
                        node.global_paths and not node.global_paths[-1].poses)
        reset_late = []
        node.create_subscription(MarkerArray, topics['local'], reset_late.append, qos)
        node.spin_until(lambda: bool(reset_late))
        assert len(reset_late[-1].markers) == 1
        output = {'status': 'passed', 'scope': 'ROS 2 MarkerArray transport and content; RViz GUI not rendered',
            'local_vertices': len(vertices.points), 'local_edges': len(edges.points)//2,
            'map_before_reset': map_stats, 'final_status': status(),
            'checks': {'real_planner_graph_geometry': True, 'frontier_markers': True,
                'occupied_cells_preserved_under_display_budget': True, 'bounded_map_display': True,
                'bounded_odometry_trace': True, 'late_subscriber_gets_complete_snapshot': True,
                'rejected_map_not_displayed': True, 'malformed_graph_preserves_display': True,
                'reset_clears_markers_and_paths': True, 'late_subscriber_after_reset_is_empty': True}}
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for process in reversed(processes):
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
            assert process.returncode == 0, process.returncode
    output['checks']['clean_child_process_shutdown'] = True
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + '\n')
    print(json.dumps(output, indent=2))


if __name__ == '__main__':
    main()
