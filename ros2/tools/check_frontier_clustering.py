#!/usr/bin/env python3
"""Check native frontier clustering snapshots against local and PlanAhead graphs.

The planner consumes a partial synthetic occupancy map and fixed odometry. This
checks the actual absorption diagnostics and their RViz geometry without moving
a robot, publishing trajectories, or reimplementing the clustering algorithm.
"""
import argparse
import heapq
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

sys.dont_write_bytecode = True
from check_planning_visualization import close, close_points, require, snapshot_record, stamp_ns, xyz


ADMISSION_FIELDS = ('candidates', 'clusters', 'near_existing', 'near_visited', 'invalid_paths', 'added')


def graph_record(message):
    vertices = []
    for vertex in message.vertices:
        q = vertex.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        vertices.append({'id': vertex.id, 'state': xyz(vertex.pose.position)+[yaw],
                         'frontier': vertex.is_frontier,
                         'counts': [vertex.num_unknown_voxels, vertex.num_free_voxels, vertex.num_occupied_voxels]})
    return {'frame': message.header.frame_id, 'stamp_ns': stamp_ns(message.header),
            'vertices': sorted(vertices, key=lambda entry:entry['id']),
            'edges': sorted([[edge.source_id, edge.target_id, edge.weight] for edge in message.edges])}


def display_markers(message, marker_type):
    require(message.markers and message.markers[0].action == marker_type.DELETEALL,
            'Clustering snapshot does not start with DELETEALL')
    markers = {}
    for index, marker in enumerate(message.markers):
        require(marker.header.frame_id == 'map', 'Clustering marker has wrong coordinate frame')
        require(stamp_ns(marker.header) == stamp_ns(message.markers[0].header),
                'Clustering markers mix different snapshot timestamps')
        if index == 0:
            continue
        require(marker.action == marker_type.ADD, 'Clustering snapshot contains an incremental deletion')
        identity = marker.ns, marker.id
        require(identity not in markers, 'Duplicate clustering marker identity')
        markers[identity] = marker
        values = xyz(marker.pose.position)+xyz(marker.scale)
        values += [value for point in marker.points for value in xyz(point)]
        require(all(math.isfinite(value) for value in values), 'Clustering marker has nonfinite geometry')
        require(marker.color.a > 0., 'Clustering marker is invisible')
        if marker.type in (marker_type.LINE_LIST, marker_type.LINE_STRIP):
            q = marker.pose.orientation
            close(xyz(marker.pose.position), [0., 0., 0.], 'Path marker translation')
            require(abs(q.x)+abs(q.y)+abs(q.z) < 1.e-8 and abs(abs(q.w)-1.) < 1.e-8,
                    'Clustering line marker unexpectedly transforms its graph coordinates')
            require(marker.scale.x > 0., 'Clustering line width is not positive')
        else:
            require(marker.type == marker_type.TEXT_VIEW_FACING, 'Unexpected clustering marker type')
    return markers


def verify_clear(metadata, message, marker_type, revision):
    require(metadata['request'] == 'clear' and metadata['map_revision'] == revision,
            'Clear snapshot has wrong request or map revision')
    require(not metadata['evaluated'] and metadata['candidate_count'] == 0 and
            metadata['representative_ids'] == [] and metadata['candidates'] == [],
            'Clear metadata retains an evaluated clustering result')
    require(all(metadata['admission'][field] == 0 for field in ADMISSION_FIELDS),
            'Clear metadata retains frontier admission counters')
    require(not display_markers(message, marker_type), 'Clear snapshot retains clustering geometry')
    return {'metadata': metadata, 'markers': snapshot_record(message)}


def verify_clustering(metadata, message, graph, marker_type, request, root, revision):
    require(metadata['request'] == request and metadata['map_revision'] == revision,
            'Clustering metadata belongs to another request or map')
    require(metadata['captured'] and metadata['evaluated'], 'Native clustering diagnostics were not captured')
    candidates, representatives = metadata['candidates'], metadata['representative_ids']
    require(metadata['candidate_count'] == len(candidates) > 0 and representatives,
            'Fixture did not exercise nonempty frontier candidates and retained representatives')
    require(len(representatives) == len(set(representatives)), 'Repeated representative ID')
    admission = metadata['admission']
    require(all(isinstance(admission[field], int) and admission[field] >= 0 for field in ADMISSION_FIELDS),
            'Invalid frontier admission counters')
    require(admission['candidates'] == len(candidates) and admission['clusters'] == len(representatives),
            'Admission totals differ from pre-filter clustering diagnostics')
    require(sum(admission[field] for field in ADMISSION_FIELDS[2:]) == admission['clusters'],
            'Admission outcomes do not account for every representative')

    record = graph_record(graph)
    require(record['frame'] == 'map', 'Local graph uses an unexpected frame')
    vertices = {vertex['id']: vertex for vertex in record['vertices']}
    require(len(vertices) == len(record['vertices']) and 0 in vertices, 'Invalid local graph vertex IDs')
    close(vertices[0]['state'][:3], root, 'Current local-graph root')
    adjacency = {identifier: {} for identifier in vertices}
    for a, b, weight in record['edges']:
        require(a in vertices and b in vertices and a != b and math.isfinite(weight) and weight > 0.,
                'Invalid native graph edge')
        distance = math.dist(vertices[a]['state'][:3], vertices[b]['state'][:3])
        require(math.isclose(weight, distance, rel_tol=1.e-6, abs_tol=1.e-8),
                'Aerial graph edge weight differs from its geometry')
        adjacency[a][b] = adjacency[b][a] = weight
    shortest, queue = {0: 0.}, [(0., 0)]
    while queue:
        cost, identifier = heapq.heappop(queue)
        if cost != shortest[identifier]:
            continue
        for neighbor, weight in adjacency[identifier].items():
            updated = cost+weight
            if updated < shortest.get(neighbor, math.inf):
                shortest[neighbor] = updated
                heapq.heappush(queue, (updated, neighbor))

    by_id, paths = {}, []
    for candidate in candidates:
        identifier, states = candidate['vertex_id'], candidate['path']
        require(identifier in vertices and identifier not in by_id and identifier != 0,
                'Candidate has an unknown, repeated, or root vertex ID')
        require(vertices[identifier]['frontier'], 'Candidate is not a frontier in the current local graph')
        require(len(states) >= 2, 'Candidate path is empty or contains only its root')
        identifiers = []
        for state in states:
            require(len(state) == 4 and all(math.isfinite(value) for value in state),
                    'Candidate path does not contain finite [x,y,z,yaw] states')
            matching = [vertex_id for vertex_id, vertex in vertices.items()
                        if math.dist(state[:3], vertex['state'][:3]) < 1.e-8]
            require(len(matching) == 1, 'Candidate path state does not uniquely match a real graph vertex')
            vertex_id = matching[0]
            difference = state[3]-vertices[vertex_id]['state'][3]
            require(abs(math.atan2(math.sin(difference), math.cos(difference))) < 1.e-8,
                    'Candidate path yaw differs from its graph vertex')
            identifiers.append(vertex_id)
        require(identifiers[0] == 0 and identifiers[-1] == identifier,
                'Candidate path is not directed from this graph root to its candidate')
        require(len(set(identifiers)) == len(identifiers), 'Candidate shortest path contains a cycle')
        require(all(b in adjacency[a] for a, b in zip(identifiers, identifiers[1:])),
                'Candidate path contains a segment absent from the real graph')
        weight = sum(adjacency[a][b] for a, b in zip(identifiers, identifiers[1:]))
        require(math.isclose(weight, shortest.get(identifier, math.inf), rel_tol=1.e-6, abs_tol=1.e-8),
                'Candidate is not a shortest path in the current graph')
        by_id[identifier] = candidate
        paths.append({'vertex_id': identifier, 'cluster_id': candidate['cluster_id'],
                      'path_vertex_ids': identifiers, 'path_weight_m': weight,
                      'shortest_weight_m': shortest[identifier]})

    markers = display_markers(message, marker_type)
    line_lists = {identity: marker for identity, marker in markers.items() if marker.type == marker_type.LINE_LIST}
    line_strips = {identity: marker for identity, marker in markers.items() if marker.type == marker_type.LINE_STRIP}
    require(set(line_lists) == {(f'cluster{index}', 0) for index in range(len(representatives))},
            'Cluster line-list identities differ from returned representative order')
    require(set(line_strips) == {('representative', identifier) for identifier in representatives},
            'Representative paths differ from the returned representative IDs')
    cluster_records = []
    for index, representative in enumerate(representatives):
        require(representative in by_id, 'Returned representative is absent from candidate diagnostics')
        members = [candidate for candidate in candidates if candidate['cluster_id'] == representative]
        expected = [state[:3] for candidate in members for pair in
                    zip(candidate['path'], candidate['path'][1:]) for state in pair]
        require(expected, 'Retained representative has no drawable member paths')
        close_points([xyz(point) for point in line_lists[(f'cluster{index}', 0)].points], expected,
                     f'cluster{index}: candidate-order adjacent edge pairs')
        close_points([xyz(point) for point in line_strips[('representative', representative)].points],
                     [state[:3] for state in by_id[representative]['path']],
                     f'Representative {representative}: complete root-to-leaf path')
        cluster_records.append({'index': index, 'representative_id': representative,
                                'member_ids': [candidate['vertex_id'] for candidate in members],
                                'line_segments': len(expected)//2})
    return {'candidate_paths': paths, 'clusters': cluster_records,
            'undrawn_candidate_ids': [candidate['vertex_id'] for candidate in candidates
                                     if candidate['cluster_id'] not in representatives],
            'admission_accounts_for_all_representatives': True}


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/frontier_clustering/summary.json')
    parser.add_argument('--timeout', type=float, default=150.)
    args = parser.parse_args()
    require(math.isfinite(args.timeout) and args.timeout > 0., 'Timeout must be positive')
    output = args.output.resolve()
    log_path = output.with_suffix('.planner.log')
    require(not output.exists() and not log_path.exists(), 'Refusing to overwrite existing results or planner log')
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    started, initialized, probe, process = time.monotonic(), False, None, None
    deadline = started+args.timeout
    summary = {'status': 'failed', 'scope': __doc__, 'domain': os.environ['ROS_DOMAIN_ID'],
               'phases': {}, 'checks': {}, 'cleanup': {}, 'log': str(log_path)}
    try:
        import rclpy
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import Odometry
        from planner_msgs.msg import Graph
        from planner_msgs.srv import PlanAhead, PlannerSrv
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from visualization_msgs.msg import Marker, MarkerArray
        sys.path.insert(0, str(workspace/'src/gbplanner_local/scripts'))
        from local_demo import DIM, ORIGIN, RES, make_map

        rclpy.init(args=[])
        initialized = True
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Match the synthetic local.yaml graph settings explicitly because its
        # unqualified node key would not target this probe's isolated namespace.
        parameters = {
            'map.resolution': RES, 'map.origin': list(ORIGIN), 'map.dimensions': list(DIM),
            'robot.size': [.4, .4, .4], 'global.enable': True, 'global.auto_home': True,
            'global.background_enable': False, 'mission.enabled': False,
            'visualization.planning_diagnostics': True, 'planning.auto_global_planner_enable': False,
            'planning.go_home_if_fully_explored': False, 'planning.path_safety_enhance_enable': False,
            'planning.path_interpolation_distance': .5, 'planning.adaptive_exploration': False,
            'sampling.seed': 42, 'sampling.seed_mode': 'fixed',
            'sampling.min': [-3., -3., -1.], 'sampling.max': [3., 3., 1.],
            'planning.max_vertices': 100, 'planning.max_edges': 2000, 'planning.max_attempts': 4000,
            'planning.empty_graph_cutoff': 1000, 'planning.path_length_penalty': .07,
            'planning.path_direction_penalty': 0., 'sensor.range': 6.,
        }
        summary['parameters'] = parameters

        class Probe(Node):
            def __init__(self):
                super().__init__('probe', namespace=f'/frontier_clustering_{os.getpid()}')
                self.rpc_clients = {}
                self.graphs, self.global_graphs, self.metrics, self.ahead_metrics = [], [], [], []
                self.clustering, self.markers = [], []
                self.maps = self.create_publisher(PointCloud2, 'voxel_map', durable)
                self.odometry = self.create_publisher(Odometry, 'odometry', 10)
                self.create_subscription(Graph, 'local_graph', self.graphs.append, durable)
                self.create_subscription(Graph, 'global_graph', self.global_graphs.append, durable)
                self.create_subscription(String, 'planning_metrics',
                                         lambda message:self.metrics.append(json.loads(message.data)), durable)
                self.create_subscription(String, 'plan_ahead_metrics',
                                         lambda message:self.ahead_metrics.append(json.loads(message.data)), durable)
                self.create_subscription(String, 'frontier_clustering',
                                         lambda message:self.clustering.append(json.loads(message.data)), durable)
                self.create_subscription(MarkerArray, 'vis/shortest_path_clustering', self.markers.append, durable)

            def until(self, predicate, label, seconds=20.):
                limit = min(deadline, time.monotonic()+seconds)
                while not predicate():
                    if process.poll() is not None:
                        raise RuntimeError(f'Planner exited {process.returncode}; see {log_path}')
                    if time.monotonic() >= limit:
                        raise TimeoutError(label)
                    rclpy.spin_once(self, timeout_sec=.02)

            def call(self, name, request, kind=Trigger):
                if name not in self.rpc_clients:
                    self.rpc_clients[name] = self.create_client(kind, name)
                client = self.rpc_clients[name]
                self.until(client.service_is_ready, name+' discovery')
                future = client.call_async(request)
                self.until(future.done, name+' response', 45.)
                result = future.result()
                require(result is not None, name+': empty response')
                return result

            def status(self, name):
                return json.loads(self.call(name, Trigger.Request()).message)

            def publish_map(self, message):
                previous = self.status('map_status')['map_revision']
                self.maps.publish(message)
                self.until(lambda:self.status('map_status')['map_revision'] == previous+1, 'Map accepted')
                return previous+1

            def snapshot(self, label, before, since, request, revision, root=None):
                self.until(lambda:len(self.clustering) > before and self.clustering[-1]['request'] == request and
                           self.clustering[-1]['map_revision'] == revision and self.markers and
                           self.markers[-1].markers and self.clustering[-1]['stamp_ns'] ==
                           stamp_ns(self.markers[-1].markers[0].header) and
                           stamp_ns(self.markers[-1].markers[0].header) >= since and
                           ((len(self.markers[-1].markers) == 1) if request == 'clear' else
                            self.clustering[-1]['evaluated'] and len(self.markers[-1].markers) > 1),
                           label+' fresh clustering snapshot')
                record = {'metadata': self.clustering[-1], 'markers': snapshot_record(self.markers[-1])}
                summary['phases'][label] = record
                if request == 'clear':
                    verify_clear(self.clustering[-1], self.markers[-1], Marker, revision)
                else:
                    self.until(lambda:self.graphs and stamp_ns(self.graphs[-1].header) >= since,
                               label+' fresh local graph')
                    record['graph'] = graph_record(self.graphs[-1])
                    record['verification'] = verify_clustering(self.clustering[-1], self.markers[-1],
                                                              self.graphs[-1], Marker, request, root, revision)
                metadata, markers = [], []
                subscriptions = [self.create_subscription(String, 'frontier_clustering',
                                 lambda message:metadata.append(json.loads(message.data)), durable),
                                 self.create_subscription(MarkerArray, 'vis/shortest_path_clustering',
                                                          markers.append, durable)]
                try:
                    self.until(lambda:metadata and markers, label+' late subscriber')
                    require(metadata[-1] == record['metadata'] and snapshot_record(markers[-1]) == record['markers'],
                            label+': late subscriber did not receive the complete retained snapshot')
                    record['late_subscriber_matches'] = True
                finally:
                    for subscription in subscriptions:
                        self.destroy_subscription(subscription)
                return record

        probe = Probe()
        command = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
                   '-r', '__ns:='+probe.get_namespace()]
        for key, value in parameters.items():
            command += ['-p', key+':='+json.dumps(value)]
        summary['launch'] = command
        with log_path.open('x') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=dict(os.environ))
        probe.until(lambda:probe.maps.get_subscription_count() > 0 and probe.odometry.get_subscription_count() > 0,
                    'Map and odometry discovery')
        base = make_map()
        revision = probe.publish_map(base)
        odometry = Odometry()
        odometry.header.frame_id, odometry.child_frame_id = 'map', 'base_link'
        odometry.header.stamp = probe.get_clock().now().to_msg()
        odometry.pose.pose.position.x, odometry.pose.pose.orientation.w = -2., 1.
        probe.odometry.publish(odometry)
        probe.until(lambda:math.dist(probe.status('sensor_gain').get('root', [1.e6]*4)[:3], [-2., 0., 0.]) < 1.e-8,
                    'Synthetic odometry accepted')
        probe.until(lambda:probe.global_graphs and probe.global_graphs[-1].vertices, 'Global home initialized')

        for label, root in (('local', [-2., 0., 0.]), ('hypothetical', [-1.75, 0., 0.]),
                            ('ahead', [-1.5, 0., 0.])):
            request = PlanAhead.Request() if label == 'ahead' else PlannerSrv.Request()
            request.header.frame_id, request.bound_mode = 'map', 3
            if label == 'local':
                request.root_pose.orientation.w = 0.  # Current-odometry sentinel.
            else:
                request.root_pose = Pose()
                request.root_pose.position.x, request.root_pose.position.y, request.root_pose.position.z = root
                request.root_pose.orientation.w = 1.
            if label == 'ahead':
                request.arrival_time = 1.
                metrics, service, kind = probe.ahead_metrics, 'planner_ahead', PlanAhead
            else:
                metrics, service, kind = probe.metrics, 'planner', PlannerSrv
            before_status = probe.status('global_status')
            before, metric_before, since = len(probe.clustering), len(metrics), probe.get_clock().now().nanoseconds
            response = probe.call(service, request, kind)
            probe.until(lambda:len(metrics) > metric_before, label+' planning metrics')
            if label == 'ahead':
                require(response.success and not response.global_required and not response.homing_required,
                        'PlanAhead fixture failed: '+response.reason)
            else:
                require(metrics[-1]['success'] and response.status == PlannerSrv.Response.K_FORWARD,
                        label+': local fixture failed or changed modes: '+str(metrics[-1]))
            record = probe.snapshot(label, before, since, 'clear' if label == 'hypothetical' else label, revision, root)
            record['planning_metrics'] = metrics[-1]
            record['global_status'] = probe.status('global_status')
            if label == 'hypothetical':
                require(record['global_status'] == before_status, 'Hypothetical local root altered persistent global history')
                summary['checks']['hypothetical_root_clears_without_absorption'] = True

        empty = make_map()
        data = bytearray(empty.data)
        for offset in range(12, len(data), empty.point_step):
            data[offset] = 255
        empty.data = bytes(data)
        before, since = len(probe.clustering), probe.get_clock().now().nanoseconds
        revision = probe.publish_map(empty)
        probe.snapshot('empty_map', before, since, 'clear', revision)
        summary['final_map_status'] = probe.status('map_status')
        require(summary['final_map_status']['known_cells'] == 0, 'Map reset did not clear observed occupancy')
        summary['checks']['current_graph_paths_and_cluster_geometry'] = True
        summary['checks']['late_subscribers_receive_complete_snapshots'] = True
        summary['checks']['empty_map_clears_clustering'] = True
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'], summary['traceback'] = str(error), traceback.format_exc()
        if probe is not None:
            summary['last_clustering'] = probe.clustering[-1] if probe.clustering else None
            summary['last_markers'] = snapshot_record(probe.markers[-1]) if probe.markers else None
            summary['last_local_graph'] = graph_record(probe.graphs[-1]) if probe.graphs else None
    finally:
        cleanup_errors = []
        if process is not None:
            summary['cleanup']['forced'] = False
            try:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=5.)
                    except subprocess.TimeoutExpired:
                        summary['cleanup']['forced'] = True
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=2.)
                summary['cleanup']['exit_code'] = process.returncode
                if summary['cleanup']['forced']:
                    cleanup_errors.append('Planner required forced shutdown')
                if process.returncode != 0:
                    cleanup_errors.append(f'Planner exited with nonzero status {process.returncode}')
            except Exception as error:
                cleanup_errors.append(str(error))
        if probe is not None:
            try:
                probe.destroy_node()
            except Exception as error:
                cleanup_errors.append(str(error))
        if initialized:
            try:
                rclpy.shutdown()
            except Exception as error:
                cleanup_errors.append(str(error))
        if cleanup_errors:
            summary['cleanup']['errors'], summary['status'] = cleanup_errors, 'failed'
        summary['wall_time_s'] = time.monotonic()-started
        temporary = output.with_suffix('.tmp')
        temporary.write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        temporary.replace(output)
        print(json.dumps({'status': summary['status'], 'output': str(output),
                          'error': summary.get('error'), 'wall_time_s': summary['wall_time_s']}, ensure_ascii=False))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
