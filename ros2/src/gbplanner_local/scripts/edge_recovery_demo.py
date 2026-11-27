#!/usr/bin/env python3
"""Exercise retained graph recovery through accepted ROS 2 map snapshots."""
import argparse
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import rclpy
from global_demo import GlobalDemo
from local_demo import make_map, xyz
from mapping_demo import inspect_snapshot, verify_path


def barrier(status):
    message = make_map(known=True, wall=False)
    data = bytearray(message.data)
    for offset in range(0, len(data), message.point_step):
        if .25 < struct.unpack_from('<f', data, offset)[0] < .75:
            struct.pack_into('<b', data, offset + 12, status)
    message.data = bytes(data)
    return message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('results/edge_recovery/summary.json'))
    args = parser.parse_args()
    process = node = None
    report = {'status': 'failed'}
    try:
        process = subprocess.Popen(['ros2', 'run', 'gbplanner_local', 'local_planner_node',
                                   '--ros-args', '-p', 'global.enable:=true'], start_new_session=True)
        rclpy.init()
        node = GlobalDemo()
        free = make_map(known=True, wall=False)
        node.publish_map(free)
        for x in [-2, -1, 0, 1, 2]:
            node.move_odometry([x, 0, 0])
        response, metric = node.global_call(homing=True)
        assert metric['success'], metric
        original = node.status()
        original_path = xyz(response.path)
        phases = []
        for label, status, reachable in [('occupied', 100, False), ('unobserved', -1, False),
                                          ('cleared', 0, True), ('blocked_again', 100, False),
                                          ('cleared_again', 0, True)]:
            snapshot = barrier(status)
            node.publish_map(snapshot)
            graph = node.status()
            response, metric = node.global_call(homing=True)
            assert bool(response.path) == reachable, (label, metric)
            assert graph['vertices'] == original['vertices'], (label, graph)
            path = xyz(response.path)
            if reachable:
                assert graph['last_restored_edges'] > 0 and graph['suspended_edges'] == 0, graph
                assert graph['edges'] == original['edges'], graph
                verify_path(path, inspect_snapshot(snapshot))
                assert path == original_path, (path, original_path)
            else:
                assert graph['suspended_edges'] > 0, graph
                assert metric['reason'] == 'global_target_unreachable', metric
            phases.append({'phase': label, 'graph': graph, 'route': metric, 'path': path})
        reset = make_map(known=True, wall=False)
        data = bytearray(reset.data)
        for offset in range(12, len(data), reset.point_step):
            data[offset] = 255
        reset.data = bytes(data)
        node.publish_map(reset)
        cleared = node.status()
        assert cleared['vertices'] == 0 and cleared['suspended_edges'] == 0, cleared
        report = {'status': 'passed', 'scope': 'Map-driven graph recovery; synthetic map/odometry services, no dynamics',
                  'original_graph': original, 'original_path': original_path, 'phases': phases,
                  'reset_graph': cleared,
                  'checks': {'no_new_vertices_required': True, 'occupied_and_unknown_block': True,
                             'same_route_restored': True, 'repeated_cycles_do_not_duplicate_edges': True,
                             'reset_clears_suspended_edges': True}}
    except Exception as error:
        report['error'] = str(error)
        raise
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
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
