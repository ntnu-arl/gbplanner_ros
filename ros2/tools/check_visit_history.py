#!/usr/bin/env python3
"""Check algorithm visit history through one real ROS 2 planner node.

A small known-Free map and explicit odometry distinguish the 0.5 m graph
spacing from 1 m E1 history events. Check persistence, independent snapshot
revisions, native markers and late subscribers. No RViz or motion is started.
Legacy snapshot contents and the 3 m / 5 m exclusion algorithms are covered
by the separate core check, not reconstructed or claimed by this ROS fixture.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import traceback


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def xyz(point):
    return [point.x, point.y, point.z]


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=80 + os.getpid() % 120)
    parser.add_argument('--timeout', type=float, default=45.)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('Require domain 0..232 and a finite positive timeout')
    output = args.output.resolve()
    log_path, parameters = output.with_suffix('.log'), output.with_suffix('.yaml')
    stored_graph = output.with_suffix('.graph.bin')
    if any(path.exists() for path in (output, log_path, parameters, stored_graph)):
        parser.error('Use a new output name; previous evidence is retained')
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    namespace = f'/visit_history_{os.getpid()}'
    started = time.monotonic()
    deadline = started + args.timeout
    report = {'status': 'failed', 'scope': __doc__, 'stage': 'startup', 'checks': {},
              'domain_id': args.domain, 'namespace': namespace, 'snapshots': {}, 'odometry': []}
    process = node = None
    initialized = False
    with log_path.open('w') as log:
        try:
            import rclpy
            from geometry_msgs.msg import Pose
            from nav_msgs.msg import Odometry
            from planner_msgs.msg import Graph
            from planner_msgs.srv import PlannerSearch, PlannerSetHomingPos, PlannerStringTrigger
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import PointCloud2, PointField
            from std_msgs.msg import String
            from std_srvs.srv import Trigger
            from visualization_msgs.msg import Marker, MarkerArray

            resolution, origin, dimensions = .25, (-1., -1., -1.), (24, 8, 8)
            options = {
                'map.resolution': resolution, 'map.origin': list(origin), 'map.dimensions': list(dimensions),
                'robot.type': 'kAerial', 'robot.size': [.1, .1, .1],
                'global.enable': True, 'global.auto_home': False, 'global.background_enable': False,
                'mission.enabled': False, 'planning.auto_global_planner_enable': False,
                'planning.path_safety_enhance_enable': False, 'planning.overshoot': 0.,
                'sensor.range': .25, 'sensor.min_range': 0.,
                'sensor.horizontal_fov': .1, 'sensor.vertical_fov': .1,
                'sensor.horizontal_resolution': .1, 'sensor.vertical_resolution': .1,
            }
            parameters.write_text(json.dumps({'/**': {'ros__parameters': options}}, indent=2) + '\n')
            rclpy.init(args=[])
            initialized = True
            node = Node('probe', namespace=namespace)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            histories, markers, graphs, accepted = [], [], [], []
            maps = node.create_publisher(PointCloud2, 'voxel_map', qos)
            odometry = node.create_publisher(Odometry, 'odometry', 10)
            node.create_subscription(String, 'state_history', lambda msg: histories.append(json.loads(msg.data)), qos)
            node.create_subscription(MarkerArray, 'vis/state_history', markers.append, qos)
            node.create_subscription(Graph, 'global_graph', graphs.append, qos)
            node.create_subscription(PointCloud2, 'accepted_voxel_map', accepted.append, qos)
            sensor_status = node.create_client(Trigger, 'sensor_gain')
            home = node.create_client(PlannerSetHomingPos, 'gbplanner/set_homing_pos')
            save = node.create_client(PlannerStringTrigger, 'gbplanner/save_graph')
            load = node.create_client(PlannerStringTrigger, 'gbplanner/load_graph')
            search = node.create_client(PlannerSearch, 'planner_search')

            def until(condition, label):
                while not condition():
                    require(process is not None and process.poll() is None, 'Planner exited before ' + label)
                    if time.monotonic() >= deadline:
                        raise TimeoutError(report['stage'] + ': ' + label)
                    rclpy.spin_once(node, timeout_sec=.03)

            def call(client, request, label):
                until(client.service_is_ready, label + ' discovery')
                future = client.call_async(request)
                until(future.done, label + ' response')
                return future.result()

            def pose(x):
                value = Pose()
                value.position.x, value.orientation.w = x, 1.
                return value

            def sample(x):
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                message.header.stamp = node.get_clock().now().to_msg()
                message.pose.pose = pose(x)
                odometry.publish(message)
                # The source-stamped status is an application-level barrier:
                # it proves this node processed this exact odometry callback.
                # The one short gain ray is unrelated to the visited history.
                while True:
                    response = call(sensor_status, Trigger.Request(), 'odometry processing barrier')
                    if response.success:
                        observed = json.loads(response.message)
                        if observed['odometry_stamp_ns'] == stamp_ns(message.header):
                            require(observed['root'] == [x, 0., 0., 0.], 'Planner acknowledged a different root')
                            report['odometry'].append({'x': x, 'stamp_ns': stamp_ns(message.header)})
                            break
                    if time.monotonic() >= deadline:
                        raise TimeoutError('Odometry was not consumed')
                    rclpy.spin_once(node, timeout_sec=.03)

            def late_snapshot(label):
                late_history, late_markers, late_graph = [], [], []
                subscriptions = [
                    node.create_subscription(String, 'state_history', lambda msg: late_history.append(json.loads(msg.data)), qos),
                    node.create_subscription(MarkerArray, 'vis/state_history', late_markers.append, qos),
                    node.create_subscription(Graph, 'global_graph', late_graph.append, qos),
                ]
                try:
                    until(lambda: bool(late_history and late_markers and late_graph) and
                          stamp_ns(late_markers[-1].markers[0].header) == late_history[-1]['stamp_ns'],
                          label + ' durable snapshot')
                    return copy.deepcopy(late_history[-1]), copy.deepcopy(late_markers[-1]), copy.deepcopy(late_graph[-1])
                finally:
                    for subscription in subscriptions:
                        node.destroy_subscription(subscription)

            def check_snapshot(label, expected_states, expected_marker, restored):
                history, display, graph = late_snapshot(label)
                require(history['scope'] == 'algorithm_visit_history' and
                        history['coordinate_convention'] == 'planning_height', 'Wrong history meaning')
                require(history['new_event_spacing_m'] == 1 and history['odometry_graph_spacing_m'] == .5 and
                        history['frontier_exclusion_radius_m'] == 3 and history['background_exclusion_radius_m'] == 5,
                        'Wrong algorithm/display constants')
                require(history['states'] == expected_states and history['total_states'] == len(expected_states),
                        label + ': unexpected accepted history samples')
                require(history['marker'] == expected_marker and history['restored_prefix_count'] == restored,
                        label + ': marker or restored prefix differs')
                require(history['first_index'] == 0 and not history['truncated'] and
                        history['range_spheres_first_index'] == 0 and history['max_displayed_states'] == 8192 and
                        history['max_displayed_ranges'] == 256, 'Small fixture must not be display-truncated')
                require(isinstance(history['revision'], int) and history['stamp_ns'] > 0, 'Missing revision or timestamp')
                require(display.markers and display.markers[0].action == Marker.DELETEALL, 'Missing snapshot clear')
                require(all(m.header.frame_id == 'map' and stamp_ns(m.header) == history['stamp_ns']
                            for m in display.markers), 'JSON and Marker snapshot headers differ')
                additions = {m.ns: m for m in display.markers[1:]}
                expected_names = {'visit_samples', 'frontier_exclusion_3m'} if expected_states else set()
                require(set(additions) == expected_names and len(display.markers) == 1 + len(expected_names),
                        'Unexpected history markers')
                for name, scale, alpha in [('visit_samples', .2, 1.), ('frontier_exclusion_3m', 6., .06)]:
                    if name not in additions:
                        continue
                    marker = additions[name]
                    require(marker.id == 0 and marker.action == Marker.ADD and marker.type == Marker.SPHERE_LIST,
                            'Wrong marker identity/type')
                    require([xyz(point) for point in marker.points] == [state[:3] for state in history['states']],
                            'Marker points differ from actual algorithm samples')
                    require(xyz(marker.scale) == [scale] * 3, 'Wrong sphere dimensions; exclusion is a 6 m diameter')
                    require(xyz(marker.pose.position) == [0., 0., 0.] and
                            [marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z,
                             marker.pose.orientation.w] == [0., 0., 0., 1.], 'Unexpected marker transform')
                    require(all(abs(a-b) < 1e-6 for a, b in zip(
                        [marker.color.r, marker.color.g, marker.color.b, marker.color.a], [.1, .9, .2, alpha])),
                        'Wrong marker color/opacity')
                    require(not marker.frame_locked and marker.lifetime.sec == marker.lifetime.nanosec == 0,
                            'History snapshot must persist until replaced')
                report['snapshots'][label] = dict(history, graph_vertices=len(graph.vertices), graph_edges=len(graph.edges))
                return history, display, graph

            scene = PointCloud2()
            scene.header.frame_id, scene.height, scene.width = 'map', 1, math.prod(dimensions)
            scene.point_step, scene.row_step, scene.is_dense = 16, scene.width * 16, True
            scene.fields = [PointField(name=name, offset=axis * 4, datatype=PointField.FLOAT32, count=1)
                            for axis, name in enumerate(('x', 'y', 'z'))]
            scene.fields.append(PointField(name='occupancy', offset=12, datatype=PointField.INT8, count=1))
            data = bytearray(scene.row_step)
            index = 0
            for z in range(dimensions[2]):
                for y in range(dimensions[1]):
                    for x in range(dimensions[0]):
                        point = [base + (cell + .5) * resolution for base, cell in zip(origin, (x, y, z))]
                        struct.pack_into('<fffbxxx', data, index * 16, *point, 0)
                        index += 1
            scene.data = bytes(data)
            report['scene_sha256'] = hashlib.sha256(scene.data).hexdigest()
            command = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
                       '-r', '__ns:=' + namespace, '--params-file', str(parameters)]
            report['command'] = command
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            until(lambda: maps.get_subscription_count() > 0 and odometry.get_subscription_count() > 0 and
                  home.service_is_ready(), 'planner discovery')
            scene.header.stamp = node.get_clock().now().to_msg()
            maps.publish(scene)
            until(lambda: bool(accepted), 'accepted map')
            require(bytes(accepted[-1].data) == bytes(scene.data), 'Fixture map was changed')
            sample(0.)
            require(call(home, PlannerSetHomingPos.Request(), 'explicit home').success, 'Explicit home failed')
            report['stage'] = 'spacing'
            initial, _, home_graph = check_snapshot('home', [], [0., 0., 0., 0.], 0)
            require(len(home_graph.vertices) == 1, 'Home must create exactly one graph anchor')
            sample(.75)
            before, _, graph075 = check_snapshot('odom_075', [], [0., 0., 0., 0.], 0)
            require(len(graph075.vertices) == 2 and any(v.pose.position.x == .75 for v in graph075.vertices),
                    '0.75 m odometry must advance the graph independently of history')
            sample(.999)
            below, _, graph0999 = check_snapshot('odom_0999', [], [0., 0., 0., 0.], 0)
            require(initial['revision'] == before['revision'] == below['revision'], 'Sub-metre motion changed history revision')
            require(graph0999.vertices == graph075.vertices and graph0999.edges == graph075.edges,
                    'Sub-0.5 m graph displacement unexpectedly inserted a node')
            sample(1.)
            first, _, graph1 = check_snapshot('odom_1', [[1., 0., 0., 0.]], [1., 0., 0., 0.], 0)
            require(first['revision'] > below['revision'], 'First E1 event did not advance its own revision')
            require(graph1.vertices == graph0999.vertices and graph1.edges == graph0999.edges and
                    graph1.header == graph0999.header, 'E1-only event changed the public graph snapshot')
            report['checks']['home_is_not_a_visit'] = True
            report['checks']['half_metre_graph_is_separate_from_one_metre_history'] = True
            report['checks']['e1_publishes_when_graph_snapshot_is_unchanged'] = True

            sample(1.75)
            middle, _, _ = check_snapshot('odom_175', [[1., 0., 0., 0.]], [1., 0., 0., 0.], 0)
            require(middle == first, 'A later sub-metre move republished or altered history')
            report['stage'] = 'persistence'
            storage = PlannerStringTrigger.Request()
            storage.message = str(stored_graph)
            require(call(save, storage, 'save graph').success and stored_graph.exists(), 'Native graph save failed')
            require(call(load, storage, 'load graph').success, 'Native graph load failed')
            restored, _, _ = check_snapshot('loaded', [[1., 0., 0., 0.]], [1., 0., 0., 0.], 1)
            require(restored['revision'] > middle['revision'], 'Load did not advance history snapshot revision')
            sample(2.)
            after, _, _ = check_snapshot('odom_2', [[1., 0., 0., 0.], [2., 0., 0., 0.]], [2., 0., 0., 0.], 1)
            require(after['revision'] > restored['revision'], 'Restored marker did not gate the next event')
            report['checks']['load_preserves_samples_marker_and_restored_prefix'] = True
            report['checks']['new_event_after_load_keeps_prefix_count'] = True

            report['stage'] = 'hypothetical_search'
            request = PlannerSearch.Request()
            request.header.frame_id, request.header.stamp = 'map', node.get_clock().now().to_msg()
            request.use_current_state, request.bound_mode = False, request.K_EXACT_BOUND
            request.source, request.target = pose(3.), pose(3.25)
            reply = call(search, request, 'hypothetical direct search')
            require(reply.success and len(reply.path) >= 2, 'Cheap hypothetical search did not succeed')
            unchanged, _, _ = check_snapshot('after_search', after['states'], after['marker'], 1)
            require(unchanged == after, 'Hypothetical search changed actual visit history')
            report['checks']['hypothetical_search_does_not_record_visit'] = True

            report['stage'] = 'reset'
            empty = copy.deepcopy(scene)
            empty.header.stamp = node.get_clock().now().to_msg()
            empty.width, empty.row_step, empty.data = 0, 0, b''
            accepted_count = len(accepted)
            maps.publish(empty)
            until(lambda: len(accepted) > accepted_count and accepted[-1].width == 0, 'accepted empty map')
            cleared, display, cleared_graph = check_snapshot('reset', [], [0., 0., 0., 0.], 0)
            require(cleared['revision'] > after['revision'] and len(display.markers) == 1 and
                    not cleared_graph.vertices and not cleared_graph.edges, 'Map reset retained old history or graph')
            report['checks']['reset_and_late_subscribers_do_not_resurrect_history'] = True
            report['checks']['all_json_marker_points_scales_and_timestamps_match'] = True
            report['status'], report['stage'] = 'passed', 'completed'
        except Exception as error:
            report['error'], report['traceback'] = str(error), traceback.format_exc()
        finally:
            forced = False
            if process is not None:
                if process.poll() is None:
                    try:
                        os.killpg(process.pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    forced = True
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2.)
                report['child_returncode'] = process.returncode
                if forced or process.returncode != 0:
                    report['status'] = 'failed'
                    report.setdefault('error', 'Planner failed orderly shutdown')
            report['forced_termination'] = forced
            if node is not None:
                node.destroy_node()
            if initialized:
                rclpy.shutdown()
    report['elapsed_seconds'] = time.monotonic() - started
    source_paths = [Path(__file__).resolve(), workspace / 'src/gbplanner_local/src/local_planner_node.cpp',
                    workspace / 'src/gbplanner_local/src/global_memory.cpp',
                    workspace / 'src/gbplanner_local/src/global_persistence.cpp',
                    workspace / 'src/gbplanner_local/include/gbplanner_local/state_history_markers.hpp']
    report['source_sha256'] = {str(path.relative_to(workspace)): hashlib.sha256(path.read_bytes()).hexdigest()
                               for path in source_paths}
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'status': report['status'], 'checks': report['checks'], 'output': str(output),
                      'error': report.get('error'), 'elapsed_seconds': report['elapsed_seconds']}, indent=2))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
