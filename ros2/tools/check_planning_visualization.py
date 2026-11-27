#!/usr/bin/env python3
"""Check native planning markers against real local and PlanAhead service results.

One isolated planner consumes a known synthetic occupancy map and odometry. A
unique sampled chain distinguishes the full best candidate from its executable
budget prefix. No trajectory consumer or physical robot is started.
"""
import argparse
import heapq
import json
import math
import os
from pathlib import Path
import re
import signal
import shutil
import subprocess
import sys
import tempfile
import time
import traceback


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def xyz(point):
    return [point.x, point.y, point.z]


def close(actual, expected, label, tolerance=1.e-8):
    require(len(actual) == len(expected) and
            all(math.isfinite(a) and abs(a-b) <= tolerance for a, b in zip(actual, expected)),
            f'{label}: {actual} != {expected}')


def close_points(actual, expected, label):
    require(len(actual) == len(expected), f'{label}: point count {len(actual)} != {len(expected)}')
    for index, (a, b) in enumerate(zip(actual, expected)):
        close(a, b, f'{label}[{index}]')


def length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def stamp_ns(header):
    return header.stamp.sec*1000000000+header.stamp.nanosec


def snapshot_record(message):
    records = []
    for marker in message.markers:
        q = marker.pose.orientation
        records.append({'namespace': marker.ns, 'id': marker.id, 'type': marker.type,
                        'action': marker.action, 'frame': marker.header.frame_id,
                        'stamp_ns': stamp_ns(marker.header), 'position': xyz(marker.pose.position),
                        'orientation_xyzw': [q.x, q.y, q.z, q.w], 'scale': xyz(marker.scale),
                        'rgba': [marker.color.r, marker.color.g, marker.color.b, marker.color.a],
                        'points': [xyz(point) for point in marker.points], 'text': marker.text})
    return records


def added_markers(message, marker_type, label):
    require(message.markers and message.markers[0].action == marker_type.DELETEALL,
            label+': snapshot must start with DELETEALL')
    require(all(marker.header.frame_id == 'map' for marker in message.markers),
            label+': unexpected coordinate frame')
    identities, added = set(), {}
    for marker in message.markers[1:]:
        require(marker.action == marker_type.ADD, label+': unexpected incremental action')
        identity = (marker.ns, marker.id)
        require(identity not in identities, label+': duplicate marker identity')
        identities.add(identity)
        require(marker.ns not in added, label+': fixture expects one marker per namespace')
        added[marker.ns] = marker
        q, color = marker.pose.orientation, marker.color
        values = xyz(marker.pose.position)+[q.x, q.y, q.z, q.w]+xyz(marker.scale)
        values += [color.r, color.g, color.b, color.a]
        values += [value for point in marker.points for value in xyz(point)]
        require(all(math.isfinite(value) for value in values), label+': nonfinite display geometry')
        require(marker.color.a > 0., label+': invisible marker')
    return added


def shortest_tree(message):
    """Independent Dijkstra oracle; this fixture has no equal-cost alternatives."""
    vertices = {vertex.id: xyz(vertex.pose.position) for vertex in message.vertices}
    require(len(vertices) == len(message.vertices) == 6, 'Fixture did not build six unique vertices')
    require(0 in vertices, 'Native graph has no root ID 0')
    adjacency = {identifier: {} for identifier in vertices}
    edges = []
    for edge in message.edges:
        a, b, weight = edge.source_id, edge.target_id, edge.weight
        require(a in vertices and b in vertices and a != b, 'Native graph edge has invalid endpoints')
        require(math.isfinite(weight) and weight > 0., 'Native graph has invalid edge weight')
        require(math.isclose(weight, math.dist(vertices[a], vertices[b]), rel_tol=1.e-6, abs_tol=1.e-8),
                'Aerial edge weight differs from its geometric length')
        adjacency[a][b] = adjacency[b][a] = weight
        edges.append([a, b, weight])
    require(sum(len(neighbors) for neighbors in adjacency.values()) == 2*(len(vertices)-1),
            'Fixture is not an unambiguous tree')
    distance, parent, queue = {0: 0.}, {0: 0}, [(0., 0)]
    while queue:
        cost, current = heapq.heappop(queue)
        if cost != distance[current]:
            continue
        for neighbor, weight in adjacency[current].items():
            candidate = cost+weight
            if candidate < distance.get(neighbor, math.inf):
                distance[neighbor], parent[neighbor] = candidate, current
                heapq.heappush(queue, (candidate, neighbor))
    require(len(distance) == len(vertices), 'Fixture tree contains disconnected vertices')
    leaves = sorted(set(vertices)-{value for key, value in parent.items() if key != value}-{0})
    require(len(leaves) == 1, 'Fixture must have exactly one best-candidate leaf')
    return vertices, edges, distance, parent, leaves


def verify_plan(graph, response_path, path_message, metric, shortest, best, marker_type, expected_root):
    require(metric['success'] and metric['gain'] > 0., 'Native plan did not succeed with positive gain')
    vertices, edges, distances, parents, leaves = shortest_tree(graph)
    close(vertices[0], expected_root, 'Explicit planning root')
    displays = added_markers(shortest, marker_type, 'shortest_paths')
    require(set(displays) == {'shortest_edges', 'vertices', 'leaves'}, 'Missing or extra shortest-tree markers')
    for name, kind in (('shortest_edges', marker_type.LINE_LIST),
                       ('vertices', marker_type.SPHERE_LIST), ('leaves', marker_type.SPHERE_LIST)):
        require(displays[name].type == kind and displays[name].id == 0, name+': wrong marker type or ID')
    ordered = sorted(distances)
    close_points([xyz(point) for point in displays['vertices'].points],
                 [vertices[identifier] for identifier in ordered], 'Reachable vertices in native ID order')
    expected_edges = [vertices[endpoint] for identifier in ordered if identifier != parents[identifier]
                      for endpoint in (identifier, parents[identifier])]
    close_points([xyz(point) for point in displays['shortest_edges'].points], expected_edges,
                 'Shortest edges in child-parent order')
    close_points([xyz(point) for point in displays['leaves'].points],
                 [vertices[identifier] for identifier in leaves], 'Shortest-tree leaves')

    selected = int(metric['best_id'])
    require(selected == leaves[0], 'Selected target is not the unique scored leaf')
    candidate_ids = [selected]
    while candidate_ids[-1] != 0:
        candidate_ids.append(parents[candidate_ids[-1]])
    candidate_ids.reverse()
    candidate = [vertices[identifier] for identifier in candidate_ids]
    candidate_length = length(candidate)
    require(abs(candidate_length-metric['candidate_path_length_m']) < 1.e-8,
            'Candidate metric differs from the full shortest path')
    displays = added_markers(best, marker_type, 'best_planning_paths')
    require(set(displays) == {'best_path', 'best_target'}, 'Missing or extra best-path markers')
    require(displays['best_path'].type == marker_type.LINE_STRIP and displays['best_path'].id == 0,
            'Best path has wrong type or ID')
    close_points([xyz(point) for point in displays['best_path'].points], candidate, 'Full best candidate')
    target = displays['best_target']
    require(target.type == marker_type.TEXT_VIEW_FACING and target.id == 0, 'Best target has wrong type or ID')
    label = re.fullmatch(r'id=(\d+)\s+gain=([-+\deE.]+)', target.text)
    require(label is not None and int(label[1]) == selected, 'Best-target label does not identify selected graph vertex')
    require(math.isclose(float(label[2]), metric['gain'], rel_tol=1.e-6, abs_tol=1.e-8),
            'Best-target label does not display the selected gain')
    close(xyz(target.pose.position), [candidate[-1][0], candidate[-1][1], candidate[-1][2]+.2],
          'Best-target label position')

    executed = [xyz(pose.position) for pose in response_path]
    require(len(executed) >= 2, 'Native response has no executable prefix')
    # Upstream retains the first COMPLETE edge that crosses a traversal limit.
    # With 0.8 m edges and a 1 m budget the correct prefix is 1.6 m, not 1 m.
    prefix = [candidate[0]]
    for point in candidate[1:]:
        prefix.append(point)
        if length(prefix) > 1.:
            break
    close_points(executed, prefix, 'Budget prefix without interpolation')
    close_points([xyz(point.pose.position) for point in path_message.poses], executed,
                 'local_path versus service response')
    require(path_message.header.frame_id == 'map', 'Execution path has unexpected frame')
    executed_length = length(executed)
    require(candidate_length > executed_length+1. and executed_length > 1.,
            'Fixture did not distinguish complete candidate from crossing-edge budget prefix')
    require(abs(executed_length-metric['path_length_m']) < 1.e-8, 'Execution path length metric differs')
    require(not metric['refinement']['attempted'], 'Fixture unexpectedly refined its execution path')
    if 'truncated_for_budget' in metric:
        require(metric['truncated_for_budget'] and metric['executed_target_id'] == candidate_ids[len(prefix)-1],
                'Native budget accounting differs from visible execution prefix')
    return {'vertices': [{'id': identifier, 'xyz': vertices[identifier]} for identifier in sorted(vertices)],
            'edges': edges, 'shortest_parents': parents, 'leaf_ids': leaves, 'best_id': selected,
            'candidate_ids': candidate_ids, 'candidate_xyz': candidate, 'candidate_length_m': candidate_length,
            'executed_xyz': executed, 'executed_length_m': executed_length, 'metrics': metric,
            'shortest_snapshot': snapshot_record(shortest), 'best_snapshot': snapshot_record(best)}


def verify_zones(message, marker_type, parameters):
    markers = added_markers(message, marker_type, 'no_gain_zone')
    require(len(markers) == 2, 'Expected both configured no-gain zones')
    cubes = [marker for marker in markers.values() if marker.type == marker_type.CUBE]
    spheres = [marker for marker in markers.values() if marker.type == marker_type.SPHERE]
    require(len(cubes) == len(spheres) == 1, 'No-gain zones have wrong geometric types')
    box, sphere = cubes[0], spheres[0]
    yaw, pitch, roll = parameters['gain.rotated_box.rotations']
    cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    rotation = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    minimum, maximum = parameters['gain.rotated_box.minimum'], parameters['gain.rotated_box.maximum']
    midpoint = [(a+b)/2 for a, b in zip(minimum, maximum)]
    center = [value+sum(rotation[i][j]*midpoint[j] for j in range(3))
              for i, value in enumerate(parameters['gain.rotated_box.center'])]
    close(xyz(box.pose.position), center, 'Rotated asymmetric box center')
    close(xyz(box.scale), [b-a for a, b in zip(minimum, maximum)], 'Box dimensions')
    cy, sy, cp, sp, cr, sr = (math.cos(yaw/2), math.sin(yaw/2), math.cos(pitch/2),
                            math.sin(pitch/2), math.cos(roll/2), math.sin(roll/2))
    expected_q = [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]
    q = box.pose.orientation
    actual_q = [q.x, q.y, q.z, q.w]
    # q and -q encode the same rotation.
    if sum(a*b for a, b in zip(actual_q, expected_q)) < 0.:
        expected_q = [-value for value in expected_q]
    close(actual_q, expected_q, 'Cuboid orientation Rz(yaw) Ry(pitch) Rx(roll)')
    close(xyz(sphere.pose.position), parameters['gain.sphere.center'], 'Sphere center')
    close(xyz(sphere.scale), [2*parameters['gain.sphere.radius']]*3, 'Sphere diameter')
    return snapshot_record(message)


def rviz_configuration(source, namespace):
    """Keep the shipped display configuration; only scope topics and disable its interactive editor."""
    lines, remaps, disabled = [], {}, 0
    interactive = False
    for line in source.read_text().splitlines():
        if re.match(r'\s*-\s+Class:', line):
            interactive = line.strip() == '- Class: rviz_default_plugins/InteractiveMarkers'
        if interactive and re.match(r'\s*Enabled:\s*true\s*$', line):
            line = re.sub(r'(Enabled:\s*)true', r'\1false', line)
            disabled += 1
        topic = re.fullmatch(r'(\s*(?:Value|Update Topic):\s*)(/[^\s#]+)(\s*)', line)
        if topic:
            remaps[topic[2]] = namespace+topic[2]
            line = topic[1]+remaps[topic[2]]+topic[3]
        lines.append(line)
    require(disabled > 0, 'RViz source configuration has no enabled interactive marker to disable')
    for topic in ('/vis/shortest_paths', '/vis/best_planning_paths', '/vis/no_gain_zone', '/local_path'):
        require(topic in remaps, 'RViz source configuration does not contain '+topic)
    return '\n'.join(lines)+'\n', remaps


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/planning_visualization/summary.json')
    parser.add_argument('--timeout', type=float, default=90.)
    parser.add_argument('--rviz-screenshot', type=Path,
                        help='Optional PNG of the real RViz window on an existing DISPLAY')
    parser.add_argument('--screen-capture-helper', type=Path,
                        help='Compiled tools/capture_rviz_screen.cpp; required only with --rviz-screenshot')
    args = parser.parse_args()
    require(math.isfinite(args.timeout) and args.timeout > 0., 'Timeout must be positive')
    output = args.output.resolve()
    require(not output.exists(), 'Refusing to overwrite existing results: '+str(output))
    log_path = output.with_suffix('.planner.log')
    require(not log_path.exists(), 'Refusing to overwrite an existing planner log: '+str(log_path))
    screenshot = args.rviz_screenshot.resolve() if args.rviz_screenshot else None
    if screenshot is not None:
        require(args.screen_capture_helper is not None,
                '--rviz-screenshot requires --screen-capture-helper')
        require(not screenshot.exists() and screenshot not in (output, log_path),
                'Screenshot must have an unused path separate from results and logs')
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    sys.dont_write_bytecode = True
    started, initialized, probe, process = time.monotonic(), False, None, None
    rviz_process, rviz_temporary = None, None
    deadline = started+args.timeout
    summary = {'status': 'failed', 'scope': __doc__, 'domain': os.environ['ROS_DOMAIN_ID'],
               'plans': {}, 'checks': {}, 'cleanup': {}}
    try:
        import rclpy
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import Odometry, Path as RosPath
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
        parameters = {
            'map.resolution': RES, 'map.origin': list(ORIGIN), 'map.dimensions': list(DIM),
            'global.enable': False, 'global.background_enable': False, 'mission.enabled': False,
            'visualization.planning_diagnostics': True,
            'planning.auto_global_planner_enable': False, 'planning.go_home_if_fully_explored': False,
            'planning.path_safety_enhance_enable': False, 'planning.adaptive_exploration': False,
            'planning.path_interpolation_distance': 100.,
            'sampling.seed': 42, 'sampling.min': [-.5, -.5, -.1], 'sampling.max': [4.5, .5, .1],
            'planning.max_vertices': 6, 'planning.max_attempts': 20, 'planning.graph_mode': False,
            'planning.edge_min': .05, 'planning.edge_max': .8, 'planning.overshoot': 0.,
            'planning.v_max': .3, 'planning.traverse_length_max': 1., 'planning.traverse_time_max': 100.,
            'planning.path_length_penalty': 0., 'planning.path_direction_penalty': 0.,
            'planning.cluster_vertices_for_gain': False, 'robot.size': [.1, .1, .1],
            'gain.free': 1., 'gain.unknown': 1., 'gain.frontier_threshold': .05,
            'sensor.range': .75, 'sensor.horizontal_fov': 1., 'sensor.horizontal_resolution': 1.,
            'sensor.vertical_fov': .2, 'sensor.vertical_resolution': .2,
            'gain.no_gain_zones': ['rotated_box', 'sphere'],
            'gain.rotated_box.type': 'kCuboid', 'gain.rotated_box.center': [1., 3., 0.],
            'gain.rotated_box.minimum': [-.5, -.2, -.3], 'gain.rotated_box.maximum': [.7, .4, .5],
            'gain.rotated_box.rotations': [.4, .2, -.1],
            'gain.sphere.type': 'kSphere', 'gain.sphere.center': [-2., -3., .25], 'gain.sphere.radius': .45,
        }
        for axis in ('x', 'y', 'z', 'heading'):
            parameters[f'sampling.{axis}.distribution'] = 'constant'
            parameters[f'sampling.{axis}.constant'] = 4. if axis == 'x' else 0.
        summary['parameters'] = parameters

        class Probe(Node):
            def __init__(self):
                super().__init__('probe', namespace=f'/planning_visualization_{os.getpid()}')
                self.graphs, self.paths, self.metrics, self.ahead_metrics = [], [], [], []
                self.markers = {name: [] for name in ('shortest_paths', 'best_planning_paths', 'no_gain_zone')}
                self.rpc_clients = {}
                self.maps = self.create_publisher(PointCloud2, 'voxel_map', durable)
                self.odometry = self.create_publisher(Odometry, 'odometry', 10)
                self.create_subscription(Graph, 'local_graph', self.graphs.append, durable)
                self.create_subscription(RosPath, 'local_path', self.paths.append, durable)
                self.create_subscription(String, 'planning_metrics',
                                         lambda message:self.metrics.append(json.loads(message.data)), durable)
                self.create_subscription(String, 'plan_ahead_metrics',
                                         lambda message:self.ahead_metrics.append(json.loads(message.data)), durable)
                for name, received in self.markers.items():
                    self.create_subscription(MarkerArray, 'vis/'+name, received.append, durable)

            def until(self, predicate, label, seconds=15.):
                limit = min(deadline, time.monotonic()+seconds)
                while not predicate():
                    if process.poll() is not None:
                        raise RuntimeError(f'Planner exited {process.returncode}; see {summary["log"]}')
                    if rviz_process is not None and rviz_process.poll() is not None:
                        raise RuntimeError(f'RViz exited {rviz_process.returncode}; see {summary["rviz"]["log"]}')
                    if time.monotonic() >= limit:
                        raise TimeoutError(label)
                    rclpy.spin_once(self, timeout_sec=.02)

            def call(self, name, request, kind=Trigger):
                if name not in self.rpc_clients:
                    self.rpc_clients[name] = self.create_client(kind, name)
                client = self.rpc_clients[name]
                self.until(client.service_is_ready, name+' discovery')
                future = client.call_async(request)
                self.until(future.done, name+' response')
                response = future.result()
                require(response is not None, name+': empty response')
                return response

            def status(self, name):
                response = self.call(name, Trigger.Request())
                # sensor_gain reports success=false until queued odometry arrives.
                # Its JSON reason lets the bounded readiness predicate retry.
                return json.loads(response.message)

            def publish_map(self, message):
                before = self.status('map_status')['map_revision']
                self.maps.publish(message)
                self.until(lambda:self.status('map_status')['map_revision'] == before+1, 'Map accepted')

            def wait_plan(self, since):
                self.until(lambda:self.graphs and stamp_ns(self.graphs[-1].header) >= since and
                           self.paths and stamp_ns(self.paths[-1].header) >= since and
                           all(self.markers[name] and len(self.markers[name][-1].markers) > 1 and
                               stamp_ns(self.markers[name][-1].markers[0].header) >= since
                               for name in ('shortest_paths', 'best_planning_paths')),
                           'Complete fresh planning publications')

            def check_late(self, expected):
                received = {name: [] for name in self.markers}
                subscriptions = [self.create_subscription(MarkerArray, 'vis/'+name, values.append, durable)
                                 for name, values in received.items()]
                try:
                    self.until(lambda:all(received.values()), 'Late subscriber receives all durable snapshots')
                    records = {name:snapshot_record(values[-1]) for name, values in received.items()}
                    require(records == expected, 'Late subscriber received a stale or partial marker snapshot')
                    return {'same_retained_snapshots': True,
                            'marker_counts': {name:len(values[-1].markers) for name, values in received.items()}}
                finally:
                    for subscription in subscriptions:
                        self.destroy_subscription(subscription)

        probe = Probe()
        command = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
                   '-r', '__ns:='+probe.get_namespace()]
        for key, value in parameters.items():
            command += ['-p', key+':='+json.dumps(value)]
        summary['launch'], summary['log'] = command, str(log_path)
        with log_path.open('x') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=dict(os.environ))
        probe.until(lambda:probe.maps.get_subscription_count() > 0 and
                    probe.odometry.get_subscription_count() > 0, 'Map and odometry discovery')
        probe.publish_map(make_map(known=True, wall=False))
        odometry = Odometry()
        odometry.header.frame_id, odometry.child_frame_id = 'map', 'base_link'
        odometry.header.stamp = probe.get_clock().now().to_msg()
        odometry.pose.pose.position.x, odometry.pose.pose.orientation.w = -2., 1.
        probe.odometry.publish(odometry)
        probe.until(lambda:math.dist(probe.status('sensor_gain').get('root', [1.e6]*4)[:3], [-2., 0., 0.]) < 1.e-9,
                    'Synthetic odometry accepted')
        probe.until(lambda:probe.markers['no_gain_zone'], 'Initial no-gain-zone snapshot')
        summary['no_gain_zones'] = verify_zones(probe.markers['no_gain_zone'][-1], Marker, parameters)

        for label, root in (('local', [-2., 0., 0.]), ('ahead', [-1.5, .25, 0.])):
            request = PlannerSrv.Request() if label == 'local' else PlanAhead.Request()
            request.header.frame_id, request.bound_mode = 'map', 3
            if label == 'local':
                # ROS 2 defaults Quaternion.w to 1; explicitly select current odometry.
                request.root_pose.orientation.w = 0.
                metrics, service, service_type = probe.metrics, 'planner', PlannerSrv
            else:
                request.root_pose = Pose()
                request.root_pose.position.x, request.root_pose.position.y, request.root_pose.position.z = root
                request.root_pose.orientation.w, request.arrival_time = 1., 1.
                metrics, service, service_type = probe.ahead_metrics, 'planner_ahead', PlanAhead
            before, since = len(metrics), probe.get_clock().now().nanoseconds
            response = probe.call(service, request, service_type)
            probe.until(lambda:len(metrics) > before, label+' metrics')
            if label == 'local':
                require(response.status == PlannerSrv.Response.K_FORWARD, 'Local request changed planning modes')
                metric = metrics[-1]
            else:
                require(response.success and not response.homing_required and not response.global_required,
                        'PlanAhead rejected the fixture: '+response.reason)
                require(metrics[-1]['success'] and metrics[-1]['plan'] is not None, 'Missing PlanAhead phase metrics')
                summary['ahead_service_metrics'] = metrics[-1]
                metric = metrics[-1]['plan']
            probe.wait_plan(since)
            summary['plans'][label] = verify_plan(probe.graphs[-1], response.path, probe.paths[-1], metric,
                probe.markers['shortest_paths'][-1], probe.markers['best_planning_paths'][-1], Marker, root)
            snapshots = {name:snapshot_record(values[-1]) for name, values in probe.markers.items()}
            summary['checks'][label+'_late_subscriber'] = probe.check_late(snapshots)
            if label == 'local' and screenshot is not None:
                require(os.environ.get('DISPLAY'), '--rviz-screenshot needs an existing X11 DISPLAY')
                require(shutil.which('rviz2'), 'RViz2 is unavailable in the current environment')
                capture_helper = args.screen_capture_helper.resolve()
                require(capture_helper.is_file() and os.access(capture_helper, os.X_OK),
                        'Screen capture helper is missing or not executable: '+str(capture_helper))
                source = workspace/'src/gbplanner_local/config/gbplanner.rviz'
                configuration, remaps = rviz_configuration(source, probe.get_namespace())
                rviz_temporary = tempfile.TemporaryDirectory(prefix='gbplanner_rviz_')
                config_path = Path(rviz_temporary.name)/'planning.rviz'
                config_path.write_text(configuration)
                rviz_log = output.with_suffix('.rviz.log')
                require(not rviz_log.exists(), 'Refusing to overwrite existing RViz log: '+str(rviz_log))
                require(screenshot != rviz_log, 'Screenshot and RViz log need different paths')
                rviz_name = 'planning_visualization_rviz'
                command = ['rviz2', '-d', str(config_path), '--ros-args', '-r', '__node:='+rviz_name]
                summary['rviz'] = {'source_configuration': str(source), 'topic_remaps': remaps,
                                   'launch': command, 'log': str(rviz_log), 'display': os.environ['DISPLAY'],
                                   'interactive_markers_enabled': False, 'captured_stage': 'local',
                                   'screenshot': str(screenshot), 'capture_helper': str(capture_helper)}
                display_environment = dict(os.environ, QT_QPA_PLATFORM='xcb')
                with rviz_log.open('x') as log:
                    rviz_process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                                    start_new_session=True, env=display_environment)
                # Read actual DDS discovery: the rendered window must subscribe
                # to all three native marker snapshots before capture begins.
                def rviz_subscribers():
                    # RViz creates its own internal ROS node; Humble does not
                    # apply the executable's __node remap to every internal node.
                    # This isolated domain contains only our probe and planner
                    # before starting RViz, neither subscribes as an external
                    # consumer to these marker topics.
                    subscribers = {}
                    for name in probe.markers:
                        subscribers[name] = [info.node_namespace+'/'+info.node_name for info in
                            probe.get_subscriptions_info_by_topic(probe.get_namespace()+'/vis/'+name)
                            if info.topic_type == 'visualization_msgs/msg/MarkerArray' and
                            (info.node_name, info.node_namespace) != (probe.get_name(), probe.get_namespace())]
                    summary['rviz']['marker_subscribers'] = subscribers
                    return all(subscribers.values())
                probe.until(rviz_subscribers, 'RViz subscribes to native marker topics', 20.)
                render_until = time.monotonic()+2.
                probe.until(lambda:time.monotonic() >= render_until, 'RViz render interval', 3.)
                require(rviz_process.poll() is None, 'RViz exited before screenshot capture')
                screenshot.parent.mkdir(parents=True, exist_ok=True)
                captured = subprocess.run([str(capture_helper), str(screenshot)],
                                          capture_output=True, text=True, timeout=10., check=False,
                                          env=display_environment)
                summary['rviz']['capture_stdout'] = captured.stdout
                summary['rviz']['capture_stderr'] = captured.stderr
                require(captured.returncode == 0, 'Screen capture helper failed: '+captured.stderr.strip())
                require(screenshot.is_file() and screenshot.stat().st_size > 0, 'Screen capture produced no PNG')
                summary['rviz']['capture'] = json.loads(captured.stdout)
                summary['checks']['real_rviz_window_captured'] = True

        # A replacement snapshot with no observed voxels clears dynamic geometry,
        # while configured no-gain zones remain available to new subscribers.
        empty = make_map(known=True, wall=False)
        data = bytearray(empty.data)
        for offset in range(12, len(data), empty.point_step):
            data[offset] = 255  # signed occupancy -1: Unknown.
        empty.data = bytes(data)
        since = probe.get_clock().now().nanoseconds
        probe.publish_map(empty)
        probe.until(lambda:all(probe.markers[name] and len(probe.markers[name][-1].markers) == 1 and
                              stamp_ns(probe.markers[name][-1].markers[0].header) >= since
                              for name in ('shortest_paths', 'best_planning_paths')),
                    'Empty map clears both planning displays')
        for name in ('shortest_paths', 'best_planning_paths'):
            require(not added_markers(probe.markers[name][-1], Marker, 'reset '+name),
                    'Empty map retained dynamic planning geometry')
        require(snapshot_record(probe.markers['no_gain_zone'][-1]) == summary['no_gain_zones'],
                'Map reset changed configured no-gain zones')
        snapshots = {name:snapshot_record(values[-1]) for name, values in probe.markers.items()}
        summary['checks']['empty_map_late_subscriber'] = probe.check_late(snapshots)
        summary['checks']['empty_map_clears_paths_and_preserves_zones'] = True
        summary['final_map_status'] = probe.status('map_status')
        require(summary['final_map_status']['known_cells'] == 0, 'Reset map still has observed voxels')
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'], summary['traceback'] = str(error), traceback.format_exc()
    finally:
        cleanup_errors = []
        if rviz_process is not None:
            summary['cleanup']['rviz_forced'] = False
            try:
                if rviz_process.poll() is None:
                    os.killpg(rviz_process.pid, signal.SIGINT)
                    try:
                        rviz_process.wait(timeout=5.)
                    except subprocess.TimeoutExpired:
                        summary['cleanup']['rviz_forced'] = True
                        os.killpg(rviz_process.pid, signal.SIGKILL)
                        rviz_process.wait(timeout=2.)
                summary['cleanup']['rviz_exit_code'] = rviz_process.returncode
                if summary['cleanup']['rviz_forced']:
                    cleanup_errors.append('RViz required forced shutdown')
                if rviz_process.returncode != 0:
                    cleanup_errors.append('RViz exited with code '+str(rviz_process.returncode))
            except Exception as error:
                cleanup_errors.append(str(error))
        if rviz_temporary is not None:
            try:
                rviz_temporary.cleanup()
            except Exception as error:
                cleanup_errors.append(str(error))
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
