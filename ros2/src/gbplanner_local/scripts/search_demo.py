#!/usr/bin/env python3
"""Call PlannerSearch on one ROS node across open, doorway, and sealed maps."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess

from ament_index_python.packages import get_package_share_directory
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry, Path as RosPath
from planner_msgs.msg import Graph
from planner_msgs.srv import PlannerSearch, ValidatePath
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from local_demo import Demo, make_map, xyz
from mapping_demo import inspect_snapshot
from rotated_sampling_demo import verify_path

SOURCE = (-2.0, -0.5, 0.0)
ROBOT = (-3.0, -2.0, 0.0)
TARGET = (2.0, -0.5, 0.0)
NEIGHBORHOOD_TARGET = (0.0, -0.5, 0.0)
TARGET_RADIUS = 0.8


def pose(position):
    value = Pose()
    value.position.x, value.position.y, value.position.z = position
    value.orientation.w = 1.0
    return value


def scene(kind):
    message = make_map(known=True, wall=False)
    if kind != 'open':
        values = bytearray(message.data)
        for offset in range(0, len(values), message.point_step):
            x, y = struct.unpack_from('<ff', values, offset)
            doorway = kind == 'doorway' and 1.0 <= y <= 2.5
            if abs(x) < 0.25 and not doorway:
                values[offset + 12] = 100
        message.data = bytes(values)
    return message


def graph_data(message):
    return {'vertices': [{'id': vertex.id,
                          'position': [vertex.pose.position.x, vertex.pose.position.y,
                                       vertex.pose.position.z]}
                         for vertex in message.vertices],
            'edges': [[edge.source_id, edge.target_id, edge.weight]
                      for edge in message.edges]}


def graph_signature(message):
    # Timestamps can change on publication while the stored graph stays intact.
    return (sorted((v.id, v.pose.position.x, v.pose.position.y, v.pose.position.z,
                    v.pose.orientation.x, v.pose.orientation.y,
                    v.pose.orientation.z, v.pose.orientation.w,
                    v.num_unknown_voxels, v.num_occupied_voxels,
                    v.num_free_voxels, v.is_frontier) for v in message.vertices),
            sorted((e.source_id, e.target_id, e.weight) for e in message.edges))


class SearchDemo(Demo):
    def __init__(self):
        super().__init__()
        self.search_metrics = []
        self.search_paths = []
        self.search_graphs = []
        self.global_graphs = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'search_metrics',
                                 lambda message: self.search_metrics.append(json.loads(message.data)), qos)
        self.create_subscription(RosPath, 'search_path', self.search_paths.append, qos)
        self.create_subscription(Graph, 'search_graph', self.search_graphs.append, qos)
        self.create_subscription(Graph, 'global_graph', self.global_graphs.append, qos)
        self.search_client = self.create_client(PlannerSearch, 'planner_search')
        self.validator = self.create_client(ValidatePath, 'validate_path')
        self.global_status = self.create_client(Trigger, 'global_status')
        for client in (self.search_client, self.validator, self.global_status):
            assert client.wait_for_service(timeout_sec=15), 'Search service unavailable'

    def stored_graph(self):
        response = self.call(self.global_status, Trigger.Request())
        assert response.success, response.message
        return json.loads(response.message)

    def replace_map(self, message):
        previous = len(self.global_graphs)
        self.publish_map(message)
        self.spin_until(lambda: len(self.global_graphs) > previous)
        self.spin_until(lambda: len(self.global_graphs[-1].vertices)
                        == self.stored_graph()['vertices'])

    def search(self, label, map_kind, snapshot, source, target,
               use_current_state=False, expected_success=True, exact=True):
        before = self.stored_graph()
        previous_graph = graph_signature(self.global_graphs[-1])
        previous = (len(self.search_metrics), len(self.search_paths), len(self.search_graphs))
        request = PlannerSearch.Request()
        request.header.frame_id = 'map'
        request.header.stamp = self.get_clock().now().to_msg()
        request.bound_mode = request.K_EXACT_BOUND
        request.use_current_state = use_current_state
        # The current-state case deliberately supplies a different source.
        request.source = pose(SOURCE if use_current_state else source)
        request.target = pose(target)
        response = self.call(self.search_client, request)
        self.spin_until(lambda: len(self.search_metrics) > previous[0]
                        and len(self.search_paths) > previous[1]
                        and len(self.search_graphs) > previous[2])
        metric = self.search_metrics[-1]
        path = xyz(response.path)
        topic = self.search_paths[-1]
        graph = self.search_graphs[-1]
        assert response.success == metric['success'] == expected_success, metric
        assert topic.header.frame_id == graph.header.frame_id == 'map'
        assert xyz([item.pose for item in topic.poses]) == path
        assert len(graph.vertices) == metric['vertices'], metric
        assert metric['map_revision'] == self.revision()['map_revision'], metric
        validation = None
        if expected_success:
            assert len(path) >= 2 and math.dist(path[0], source) < 1e-9, path
            assert metric['exact_target'] == exact, metric
            endpoint_distance = math.dist(path[-1], target)
            assert abs(endpoint_distance - metric['endpoint_distance']) < 1e-8, metric
            if exact:
                assert endpoint_distance < 1e-9, path
            else:
                assert 0.4 < endpoint_distance <= TARGET_RADIUS + 1e-8, metric
                assert metric['reason'] == 'target_neighborhood', metric
            length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
            assert abs(length - metric['path_length_m']) < 1e-8, metric
            verify_path(path, snapshot)
            check = ValidatePath.Request()
            check.header.frame_id = 'map'
            check.path = response.path
            checked = self.call(self.validator, check)
            assert checked.valid and checked.reason == 'ok', checked.reason
            assert checked.map_revision == metric['map_revision']
            validation = {'valid': checked.valid, 'reason': checked.reason,
                          'map_revision': checked.map_revision}
        else:
            assert not path and not topic.poses, (metric, path)
        after = self.stored_graph()
        assert before == after, ('Search changed persistent graph counters', before, after)
        assert previous_graph == graph_signature(self.global_graphs[-1]), \
            'Search changed the persistent graph'
        return {'name': label, 'map': map_kind, 'use_current_state': use_current_state,
                'source': source, 'requested_target': target, 'metrics': metric,
                'path': path, 'validation': validation, 'graph': graph_data(graph),
                'global_before': before, 'global_after': after}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--start-node', action='store_true')
    parser.add_argument('--output', type=Path, default=Path('results/search/summary.json'))
    args = parser.parse_args()
    process = node = timer = log = None
    report = {'status': 'failed', 'runs': []}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    try:
        if args.start_node:
            config = Path(get_package_share_directory('gbplanner_local')) / 'config/search.yaml'
            command = ['ros2', 'run', 'gbplanner_local', 'local_planner_node',
                       '--ros-args', '--params-file', str(config)]
            log = (args.output.parent / 'nodes.log').open('w')
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            report['command'] = command
        rclpy.init()
        node = SearchDemo()

        def publish_odometry():
            message = Odometry()
            message.header.frame_id = 'map'
            message.header.stamp = node.get_clock().now().to_msg()
            message.pose.pose = pose(ROBOT)
            node.odometry.publish(message)

        timer = node.create_timer(.05, publish_odometry)
        node.spin_until(lambda: node.odometry.get_subscription_count() > 0)
        open_map = scene('open')
        node.replace_map(open_map)
        node.spin_until(lambda: node.stored_graph()['vertices'] > 0 and bool(node.global_graphs))
        first = node.search('direct', 'open', inspect_snapshot(open_map), ROBOT, TARGET,
                            use_current_state=True)
        assert first['metrics']['reason'] == 'direct' and first['metrics']['attempts'] == 0
        report['runs'].append(first)

        doorway = scene('doorway')
        node.replace_map(doorway)
        second = node.search('doorway', 'doorway', inspect_snapshot(doorway), SOURCE, TARGET)
        assert second['metrics']['attempts'] > 0, second['metrics']
        assert second['metrics']['path_length_m'] > math.dist(SOURCE, TARGET) + .5
        assert any(point[1] > 1.0 for point in second['path']), second['path']
        report['runs'].append(second)
        report['runs'].append(node.search('target_neighborhood', 'doorway', inspect_snapshot(doorway),
                                         SOURCE, NEIGHBORHOOD_TARGET, exact=False))

        sealed = scene('sealed')
        node.replace_map(sealed)
        report['runs'].append(node.search('sealed', 'sealed', inspect_snapshot(sealed),
                                         SOURCE, TARGET, expected_success=False))
        report.update({'status': 'passed', 'scope': 'Native ROS 2 search on synthetic occupancy snapshots',
                       'map': {'origin': [-5.0, -5.0, -1.5], 'dimensions': [40, 40, 12],
                               'resolution': .25, 'wall_x': [-.25, .25], 'doorway_y': [1.0, 2.5],
                               'robot_size': [.4, .4, .4]},
                       'target_radius': TARGET_RADIUS,
                       'checks': {'current_state_source': True, 'direct_connection': True,
                                  'exact_target_through_doorway': True, 'occupied_target_neighborhood': True,
                                  'sealed_wall_rejected': True, 'native_validate_path': True,
                                  'independent_box_probe': True, 'explicit_search_preserves_global_graph': True,
                                  'failure_clears_search_path': True, 'single_node_map_replacement': True}})
    except Exception as error:
        report['error'] = repr(error)
        raise
    finally:
        temporary = args.output.with_suffix('.tmp')
        temporary.write_text(json.dumps(report, indent=2) + '\n')
        temporary.replace(args.output)
        if timer is not None:
            timer.cancel()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        if log is not None:
            log.close()
    print(json.dumps({'status': report['status'], 'runs': [
        {'name': run['name'], **run['metrics']} for run in report['runs']]}, indent=2))


if __name__ == '__main__':
    main()
