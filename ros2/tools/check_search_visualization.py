#!/usr/bin/env python3
"""Check one real PlannerSearch result through the ROS 2 marker adapter.

Reuses the existing synthetic doorway fixture. The display joins after the
search, then a late marker subscriber joins and the planner resets its map.
No RViz window, simulator, motion command, or additional search is started.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def xyz(point):
    return [point.x, point.y, point.z]


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=80 + os.getpid() % 120)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error('--domain must be between 0 and 232')
    output = args.output.resolve()
    log_path, parameters = output.with_suffix('.log'), output.with_suffix('.yaml')
    if any(path.exists() for path in (output, log_path, parameters)):
        parser.error('Use a new output name to retain previous evidence')
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    namespace = f'/search_display_{os.getpid()}'
    started = time.monotonic()
    deadline = started + 45.
    report = {'status': 'failed', 'scope': __doc__, 'stage': 'startup',
              'domain_id': args.domain, 'namespace': namespace, 'checks': {}, 'commands': []}
    processes, node = [], None
    initialized = False
    with log_path.open('w') as log:
        try:
            import rclpy
            from nav_msgs.msg import Path as RosPath
            from planner_msgs.msg import Graph
            from planner_msgs.srv import PlannerSearch
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import PointCloud2
            from std_msgs.msg import String
            from std_srvs.srv import Trigger
            from visualization_msgs.msg import Marker, MarkerArray

            sys.dont_write_bytecode = True
            sys.path.insert(0, str(workspace / 'src/gbplanner_local/scripts'))
            from local_demo import DIM, ORIGIN, RES
            from search_demo import pose, scene

            options = {
                'map.resolution': RES, 'map.origin': list(ORIGIN), 'map.dimensions': list(DIM),
                'global.enable': False, 'global.background_enable': False,
                'robot.size': [.1, .1, .1], 'planning.overshoot': 0.,
                'planning.path_safety_enhance_enable': False,
                'planning.edge_min': .12, 'planning.edge_max': .8,
                'planning.neighbor_radius': 1.1, 'planning.neighbor_min': .12,
                'planning.neighbor_max': 1.1,
                'search.sampling.min': [-5., -5., -1.],
                'search.sampling.max': [5., 5., 1.], 'search.sampling.seed': 42,
                'search.max_vertices': 500, 'search.max_edges': 10000,
                'search.max_attempts': 12000, 'search.max_target_candidates': 5,
                'search.target_radius': .8,
            }
            parameters.write_text(json.dumps({'/**': {'ros__parameters': options}}, indent=2) + '\n')
            rclpy.init(args=[])
            initialized = True
            node = Node('probe', namespace=namespace)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            accepted, graphs, paths, metrics, markers = [], [], [], [], []
            maps = node.create_publisher(PointCloud2, 'voxel_map', qos)
            client = node.create_client(PlannerSearch, 'planner_search')
            display_status = node.create_client(Trigger, 'visualization_status')
            node.create_subscription(PointCloud2, 'accepted_voxel_map', accepted.append, qos)
            node.create_subscription(Graph, 'search_graph', graphs.append, qos)
            node.create_subscription(RosPath, 'search_path', paths.append, qos)
            node.create_subscription(String, 'search_metrics',
                                     lambda m: metrics.append(json.loads(m.data)), qos)
            node.create_subscription(MarkerArray, 'vis/search_graph', markers.append, qos)

            def start(executable, extra=()):
                command = ['ros2', 'run', 'gbplanner_local', executable, '--ros-args',
                           '-r', '__ns:=' + namespace, *extra]
                report['commands'].append(command)
                processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                                  start_new_session=True))

            def until(condition, label):
                while not condition():
                    for process in processes:
                        require(process.poll() is None, 'Native child exited before ' + label)
                    if time.monotonic() >= deadline:
                        raise TimeoutError(report['stage'] + ': ' + label)
                    rclpy.spin_once(node, timeout_sec=.03)

            def call(service, request, label):
                until(service.service_is_ready, label + ' discovery')
                future = service.call_async(request)
                until(future.done, label + ' response')
                return future.result()

            def is_empty(snapshot):
                return len(snapshot.markers) == 1 and snapshot.markers[0].action == Marker.DELETEALL

            start('local_planner_node', ['--params-file', str(parameters)])
            until(lambda: maps.get_subscription_count() > 0 and client.service_is_ready(), 'planner discovery')
            observed = scene('doorway')
            observed.header.stamp = node.get_clock().now().to_msg()
            maps.publish(observed)
            until(lambda: bool(accepted), 'accepted map')
            require(bytes(accepted[-1].data) == bytes(observed.data), 'Planner changed fixture occupancy')

            report['stage'] = 'sampled_search'
            request = PlannerSearch.Request()
            request.header.frame_id = 'map'
            request.header.stamp = node.get_clock().now().to_msg()
            request.bound_mode = request.K_EXACT_BOUND
            request.use_current_state = False
            request.source, request.target = pose((-2., -2., 0.)), pose((2., -2., 0.))
            response = call(client, request, 'PlannerSearch')
            until(lambda: bool(graphs) and bool(paths) and bool(metrics), 'search outputs')
            require(response.success and metrics[-1]['exact_target'] and len(response.path) > 2,
                    'Expected the sampled doorway detour')
            graph = graphs[-1]
            require(len(graph.vertices) == metrics[-1]['vertices'] > 2 and
                    len(graph.edges) == metrics[-1]['edges'] > 1, 'Search graph/metric counts differ')
            require([xyz(p.position) for p in response.path] ==
                    [xyz(p.pose.position) for p in paths[-1].poses], 'Service/topic route differs')
            report['search'] = metrics[-1]
            report['checks']['native_sampled_search_graph'] = True

            report['stage'] = 'late_display'
            start('visualization_node.py')
            until(lambda: markers and any(m.action == Marker.ADD and m.ns == 'search' and m.id == 1
                                          for m in markers[-1].markers), 'late display graph')
            snapshot = markers[-1]
            require(snapshot.markers[0].action == Marker.DELETEALL, 'Snapshot does not begin with DELETEALL')
            additions = {(m.ns, m.id): m for m in snapshot.markers if m.action == Marker.ADD}
            vertices = {v.id: v for v in graph.vertices}
            expected_dots = [xyz(v.pose.position) for v in graph.vertices]
            expected_lines = [xyz(vertices[identity].pose.position)
                              for edge in graph.edges for identity in (edge.source_id, edge.target_id)]
            require(additions['search', 1].type == Marker.SPHERE_LIST and
                    [xyz(p) for p in additions['search', 1].points] == expected_dots,
                    'Search vertex markers differ from authoritative Graph')
            require(additions['search', 0].type == Marker.LINE_LIST and
                    [xyz(p) for p in additions['search', 0].points] == expected_lines,
                    'Search connection markers differ from authoritative Graph')
            require(all(m.header == graph.header for m in additions.values()), 'Search marker frame/stamp differs')
            status = call(display_status, Trigger.Request(), 'display status')
            require(status.success, status.message)
            report['display'] = json.loads(status.message)
            require(report['display']['geometry_source'] == 'planner_metadata' and
                    report['display']['search_updates'] >= 1, 'Late display did not use planner metadata')
            report['checks']['late_adapter_replays_retained_graph_and_metadata'] = True
            report['checks']['all_search_vertices_and_connections_match'] = True

            report['stage'] = 'late_marker_subscriber'
            late = []
            node.create_subscription(MarkerArray, 'vis/search_graph', late.append, qos)
            until(lambda: bool(late), 'retained markers')
            require(late[-1] == snapshot, 'Late subscriber missed the complete retained graph')
            report['checks']['late_subscriber_gets_identical_snapshot'] = True

            report['stage'] = 'map_reset'
            reset = copy.deepcopy(observed)
            reset.header.stamp = node.get_clock().now().to_msg()
            reset.width, reset.row_step, reset.data = 0, 0, b''
            previous = len(graphs)
            maps.publish(reset)
            until(lambda: len(graphs) > previous and not graphs[-1].vertices and not graphs[-1].edges,
                  'authoritative empty search graph')
            until(lambda: markers and is_empty(markers[-1]), 'search marker clear')
            until(lambda: paths and not paths[-1].poses, 'search path clear')
            cleared = []
            node.create_subscription(MarkerArray, 'vis/search_graph', cleared.append, qos)
            until(lambda: bool(cleared), 'late empty marker snapshot')
            require(is_empty(cleared[-1]), 'Late subscriber resurrected pre-reset search geometry')
            report['checks']['map_reset_clears_search_graph_and_path'] = True
            report['checks']['late_subscriber_after_reset_is_empty'] = True
            report['status'], report['stage'] = 'passed', 'completed'
        except Exception as error:
            report['error'], report['traceback'] = str(error), traceback.format_exc()
        finally:
            forced = False
            for process in processes:
                if process.poll() is None:
                    try:
                        os.killpg(process.pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
            for process in reversed(processes):
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    forced = True
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2.)
            report['child_returncodes'] = [p.returncode for p in processes]
            report['forced_termination'] = forced
            if forced or any(code != 0 for code in report['child_returncodes']):
                report['status'] = 'failed'
                report.setdefault('error', 'Native child failed orderly shutdown')
            if node is not None:
                node.destroy_node()
            if initialized:
                rclpy.shutdown()
    report['elapsed_seconds'] = time.monotonic() - started
    report['source_sha256'] = {str(path.relative_to(workspace)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in [Path(__file__).resolve(), workspace / 'src/gbplanner_local/scripts/visualization_node.py',
                     workspace / 'src/gbplanner_local/config/search.rviz',
                     workspace / 'src/gbplanner_local/launch/search.launch.py']}
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'status': report['status'], 'checks': report['checks'], 'output': str(output),
                      'error': report.get('error'), 'elapsed_seconds': report['elapsed_seconds']}, indent=2))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
