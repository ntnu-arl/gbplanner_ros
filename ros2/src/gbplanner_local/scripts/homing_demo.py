#!/usr/bin/env python3
"""Verify original empty-graph-only home setting over ROS 2 services."""
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
from nav_msgs.msg import Odometry
from planner_msgs.srv import PlannerSetHomingPos
from global_demo import GlobalDemo
from local_demo import make_map, xyz


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--start-node', action='store_true')
    parser.add_argument('--output', type=Path, default=Path('results/homing_initialization.json'))
    args = parser.parse_args()
    process = node = None
    try:
        if args.start_node:
            process = subprocess.Popen(['ros2', 'run', 'gbplanner_local', 'local_planner_node',
                '--ros-args', '-p', 'global.enable:=true', '-p', 'global.auto_home:=false'],
                start_new_session=True)
        rclpy.init()
        node = GlobalDemo()
        home_client = node.create_client(PlannerSetHomingPos, 'gbplanner/set_homing_pos')
        assert home_client.wait_for_service(timeout_sec=15)
        set_home = lambda: node.call(home_client, PlannerSetHomingPos.Request()).success
        assert not set_home(), 'Missing map/odometry must not initialize home'
        node.publish_map(make_map())
        assert not set_home(), 'Missing odometry must not initialize home'
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.pose.pose.position.x = -2.
        odom.pose.pose.orientation.w = 1.
        node.spin_until(lambda: node.odometry.get_subscription_count() > 0)
        odom.header.stamp = node.get_clock().now().to_msg()
        node.odometry.publish(odom)
        # Planning acknowledges an accepted odometry sample but cannot create
        # home implicitly when global.auto_home is disabled.
        deadline = time.monotonic() + 5
        while True:
            response, metric = node.plan()
            if metric['reason'] == 'home_not_set':
                break
            assert metric['reason'] == 'odometry_missing_or_invalid', metric
            assert time.monotonic() < deadline
        assert node.status()['vertices'] == 0
        assert set_home()
        assert node.status()['vertices'] == 1
        assert not set_home(), 'Repeated requests must preserve original rejection semantics'
        node.spin_until(lambda: node.global_graphs and len(node.global_graphs[-1].vertices) == 1)
        assert node.global_graphs[-1].vertices[0].pose.position.x == -2.
        response, metric = node.plan()
        assert metric['success'], metric
        for point in ([-1, 0, 0], [0, 0, 0], [1, 0, 0]):
            node.move_odometry(point)
        before = node.status()
        assert not set_home(), 'A populated graph cannot be rebased'
        assert node.status() == before
        response, metric = node.global_call(homing=True)
        original_path = xyz(response.path)
        assert metric['success'] and math.dist(original_path[-1], [-2, 0, 0]) < 1e-8

        reset = make_map()
        data = bytearray(reset.data)
        for i in range(12, len(data), reset.point_step):
            data[i] = 255
        reset.data = bytes(data)
        node.publish_map(reset)
        assert node.status()['vertices'] == 0 and not set_home()
        node.publish_map(make_map())
        assert node.status()['vertices'] == 0, 'Map update must not implicitly set home'
        assert set_home()
        response, metric = node.global_call(homing=True)
        assert metric['success'] and len(response.path) == 1
        assert math.dist(xyz(response.path)[0], [1, 0, 0]) < 1e-8
        node.move_odometry([0, 0, 0])
        response, metric = node.global_call(homing=True)
        new_path = xyz(response.path)
        assert metric['success'] and math.dist(new_path[-1], [1, 0, 0]) < 1e-8
        blocked = make_map()
        data = bytearray(blocked.data)
        import struct
        for i in range(0, len(data), blocked.point_step):
            if .25 < struct.unpack_from('<f', data, i)[0] < .75:
                data[i+12] = 100
        blocked.data = bytes(data)
        node.publish_map(blocked)
        response, metric = node.global_call(homing=True)
        assert not response.path and metric['reason'] == 'global_target_unreachable', metric
        output = {'status': 'passed', 'scope': 'Explicit home initialization; original empty-graph restriction',
            'first_home': [-2, 0, 0], 'first_return_path': original_path,
            'home_after_reset': [1, 0, 0], 'second_return_path': new_path,
            'checks': {'missing_inputs_rejected': True, 'manual_mode_blocks_implicit_home': True,
                'repeated_request_rejected': True, 'populated_graph_preserved': True,
                'map_reset_requires_new_home': True, 'new_home_used_for_homing': True,
                'map_change_still_invalidates_return_route': True}}
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
            assert process.returncode == 0, process.returncode
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + '\n')
    print(json.dumps(output, indent=2))


if __name__ == '__main__':
    main()
