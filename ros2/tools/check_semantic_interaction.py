#!/usr/bin/env python3
"""Exercise the native marker protocol through the real semantic backend.

This checks ROS messages and services; it does not check RViz rendering or
robot motion. The complete workflow has one shared 54 s deadline, followed by
bounded process-group cleanup, rather than a new timeout for each operation.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry, Path as RosPath
from planner_msgs.srv import PlannerGlobal, ValidatePath
from planner_semantic_msgs.msg import SemanticClass, SemanticPoint
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate, Marker, MarkerArray
from visualization_msgs.srv import GetInteractiveMarkers

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/gbplanner_local/scripts'))
from local_demo import xyz
from search_demo import SOURCE, TARGET, pose, scene


def position(value):
    return [value.position.x, value.position.y, value.position.z]


def stamp_ns(value):
    return value.sec * 1_000_000_000 + value.nanosec


def main():
    started = time.monotonic()
    deadline = started + 54.0
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
                        default=ROOT / 'results/semantic_interaction/native.json')
    parser.add_argument('--domain', type=int, default=80 + os.getpid() % 120)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error('--domain must be between 0 and 232')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    namespace = '/semantic_interaction_' + str(os.getpid())
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    report = {
        'status': 'failed', 'scope': 'native interactive-marker protocol and real backend; '
                                   'no GUI rendering or robot motion',
        'namespace': namespace, 'domain_id': args.domain,
        'workflow_deadline_seconds': 54.0, 'total_budget_seconds': 60.0,
        'checks': {}, 'steps': [], 'stage': 'startup', 'observed_states': {},
    }
    processes, logs, events, updates, paths, semantic_markers = [], [], [], [], [], []
    node = timer = None
    try:
        config = args.output.parent / 'parameters.yaml'
        config.write_text(json.dumps({'/**': {'ros__parameters': {
            'global.enable': True, 'search.sampling.min': [-5., -5., -1.],
            'search.sampling.max': [5., 5., 1.], 'search.target_radius': 0.8,
        }}}, indent=2) + '\n')
        commands = [
            ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
             '--params-file', str(config), '-r', '__ns:=' + namespace],
            ['ros2', 'run', 'gbplanner_local', 'semantic_waypoint_node.py',
             '--ros-args', '-r', '__ns:=' + namespace],
        ]
        report['commands'] = commands
        for label, command in zip(('backend', 'marker'), commands):
            log = (args.output.parent / (label + '.log')).open('w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        rclpy.init(args=[])
        node = Node('semantic_interaction_probe', namespace=namespace)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        maps = node.create_publisher(PointCloud2, 'voxel_map', durable)
        odometry = node.create_publisher(Odometry, 'odometry', 10)
        feedback = node.create_publisher(InteractiveMarkerFeedback, 'semantics/feedback', 10)
        node.create_subscription(SemanticPoint, 'semantic_location', events.append, 10)
        node.create_subscription(InteractiveMarkerUpdate, 'semantics/update', updates.append, 10)
        node.create_subscription(RosPath, 'semantic_path', paths.append, durable)
        node.create_subscription(MarkerArray, 'vis/semantic_targets', semantic_markers.append, durable)
        clients = {name: node.create_client(kind, name) for name, kind in (
            ('semantics/status', Trigger), ('semantics/get_interactive_markers', GetInteractiveMarkers),
            ('semantic_state', Trigger), ('map_status', Trigger), ('global_status', Trigger),
            ('gbplanner/global', PlannerGlobal), ('validate_path', ValidatePath),
        )}

        active_waits = []

        def check_deadline():
            if time.monotonic() >= deadline:
                report['timeout_waits'] = list(active_waits)
                raise TimeoutError('Shared semantic workflow deadline exceeded at '
                                   + report['stage'] + ': ' + ' -> '.join(active_waits))
            for process, label in zip(processes, ('backend', 'marker')):
                if process.poll() is not None:
                    raise RuntimeError(f'{label} exited with status {process.returncode}; see {label}.log')

        def until(predicate, label='event'):
            active_waits.append(label)
            try:
                while True:
                    check_deadline()
                    value = predicate()
                    if value:
                        return value
                    rclpy.spin_once(node, timeout_sec=min(.05, max(0., deadline - time.monotonic())))
            finally:
                active_waits.pop()

        def call(name, request):
            check_deadline()
            future = clients[name].call_async(request)
            until(future.done, 'service_response:' + name)
            return future.result()

        def state(name='semantics/status'):
            value = json.loads(call(name, Trigger.Request()).message)
            report['observed_states'][name] = value
            return value

        def wait_state(predicate, name='semantics/status'):
            def poll():
                value = state(name)
                return value if predicate(value) else None
            return until(poll, 'state_predicate:' + name)

        def server_marker():
            reply = call('semantics/get_interactive_markers', GetInteractiveMarkers.Request())
            found = [item for item in reply.markers if item.name == 'semantic']
            assert len(found) == 1, [item.name for item in reply.markers]
            return found[0], reply.sequence_number

        def quiet(expected_received, expected_events):
            end = time.monotonic() + .15
            until(lambda: time.monotonic() >= end)
            backend = state('semantic_state')
            assert len(events) == expected_events and backend['received'] == expected_received, backend
            return backend

        def target_snapshot(category):
            def matches():
                if not semantic_markers:
                    return False
                added = {marker.ns: marker for marker in semantic_markers[-1].markers
                         if marker.action == Marker.ADD}
                return category in added and len(added[category].points) == 1
            until(matches)
            snapshot = semantic_markers[-1]
            assert len(snapshot.markers) == 3 and snapshot.markers[0].action == Marker.DELETEALL
            values = {}
            for item, rgba in zip(snapshot.markers[1:], ((1., 1., 0., 1.), (1., 0., 1., 1.))):
                assert item.ns in ('stairs', 'door') and item.id == 0
                assert item.type == Marker.CUBE_LIST and item.action == Marker.ADD
                assert item.header.frame_id == 'map'
                assert all(abs(v - .7) < 1e-6 for v in (item.scale.x, item.scale.y, item.scale.z))
                assert (item.color.r, item.color.g, item.color.b, item.color.a) == rgba
                points = [[point.x, point.y, point.z] for point in item.points]
                assert points == ([list(TARGET)] if item.ns == category else []), points
                values[item.ns] = {'points': points, 'rgba': list(rgba), 'size': item.scale.x}
            assert set(values) == {'stairs', 'door'}
            return {'clear_previous': True, 'classes': values}

        def send(event, value, menu_id=0, frame='map'):
            message = InteractiveMarkerFeedback()
            message.header.frame_id = frame
            message.header.stamp = node.get_clock().now().to_msg()
            message.client_id = 'semantic_interaction_probe'
            message.marker_name = 'semantic'
            message.control_name = 'button_control' if menu_id else 'x_control'
            message.event_type = event
            message.menu_entry_id = menu_id
            message.pose = copy.deepcopy(value)
            feedback.publish(message)

        current_odom = list(SOURCE)

        def publish_odometry():
            message = Odometry()
            message.header.frame_id, message.child_frame_id = 'map', 'base_link'
            message.header.stamp = node.get_clock().now().to_msg()
            message.pose.pose = pose(current_odom)
            # A quarter-turn makes world-X initialization distinguishable
            # from an offset along the robot's heading.
            message.pose.pose.orientation.z = message.pose.pose.orientation.w = math.sqrt(.5)
            odometry.publish(message)

        timer = node.create_timer(.05, publish_odometry)
        report['stage'] = 'service_discovery_and_initialization'
        until(lambda: all(client.service_is_ready() for client in clients.values())
              and feedback.get_subscription_count() > 0 and odometry.get_subscription_count() >= 2,
              'services_and_input_subscribers')
        initialized = wait_state(lambda value: value['initialized'])
        marker, sequence = server_marker()
        initial_position = [SOURCE[0] + 1.5, SOURCE[1], SOURCE[2]]
        assert position(marker.pose) == initial_position, position(marker.pose)
        assert marker.header.frame_id == 'map'
        assert [marker.pose.orientation.x, marker.pose.orientation.y,
                marker.pose.orientation.z, marker.pose.orientation.w] == [0., 0., 0., 1.]
        menu_entries = {entry.id: entry for entry in marker.menu_entries}
        menu_ids = initialized['menu_ids']
        for key, title in (('class', 'Class'), ('door', 'Door'),
                           ('staircase', 'Staircase'), ('accept', 'Accept')):
            assert menu_entries[menu_ids[key]].title.endswith(title), menu_entries
        assert menu_entries[menu_ids['door']].parent_id == menu_ids['class']
        assert menu_entries[menu_ids['staircase']].parent_id == menu_ids['class']
        report['initialization'] = {'marker_position': position(marker.pose),
                                    'robot_position': list(SOURCE), 'robot_yaw': math.pi / 2,
                                    'server_sequence': sequence, 'menu_ids': menu_ids}
        report['checks']['world_x_initialization_and_native_menu'] = True
        # Humble initializes last_feedback to now and last_client_id to an
        # empty string. Its native 1 s client-ownership guard therefore drops
        # a nonempty client's first feedback immediately after creation. Let
        # that initial ownership expire once, as a newly attached UI would.
        client_ready_after = time.monotonic() + 1.05

        current_odom[0] += .25
        quiet(0, 0)
        assert position(server_marker()[0].pose) == initial_position
        current_odom[:] = SOURCE
        # Restore the backend's measured root before the first map seeds its
        # global home. Otherwise map arrival can race the next odometry tick
        # and retain the small-movement probe as the graph's home position.
        publish_odometry()
        quiet(0, 0)
        report['checks']['small_odometry_move_preserves_marker'] = True
        report['stage'] = 'map_and_global_home'
        maps.publish(scene('doorway'))
        wait_state(lambda value: value['map_revision'] > 0, 'map_status')
        wait_state(lambda value: value['vertices'] == 1, 'global_status')
        report['stage'] = 'semantic_subscriber_discovery'

        def semantic_subscribers_ready():
            count = node.count_subscribers(namespace + '/semantic_location')
            report['semantic_subscribers'] = count
            return count >= 2

        until(semantic_subscribers_ready, 'semantic_location_subscribers>=2')
        until(lambda: time.monotonic() >= client_ready_after, 'native_initial_client_ownership_expiry')

        report['stage'] = 'accept_without_class'
        send(InteractiveMarkerFeedback.MENU_SELECT, marker.pose, menu_ids['accept'])
        rejected = wait_state(lambda value: value['rejected_updates'] == 1)
        assert rejected['reason'] == 'semantic_class_not_selected' and rejected['accept_count'] == 0
        quiet(0, 0)
        report['checks']['accept_without_class_rejected'] = True

        report['stage'] = 'drag_and_select_door'
        target_pose = pose(TARGET)
        send(InteractiveMarkerFeedback.POSE_UPDATE, target_pose)
        dragged = wait_state(lambda value: value['pose_updates'] == 1)
        assert dragged['pose']['position'] == list(TARGET)
        send(InteractiveMarkerFeedback.MENU_SELECT, target_pose, menu_ids['door'])
        door = wait_state(lambda value: value['selected_class'] == SemanticClass.K_DOOR)
        assert position(server_marker()[0].pose) == list(TARGET)
        quiet(0, 0)
        report['checks']['drag_and_class_selection_do_not_publish'] = True

        report['stage'] = 'invalid_drag_restore'
        send(InteractiveMarkerFeedback.POSE_UPDATE, pose((2., 0., 0.)), frame='odom')
        invalid = wait_state(lambda value: value['rejected_updates'] == 2)
        assert invalid['reason'] == 'wrong_feedback_frame'
        assert position(server_marker()[0].pose) == list(TARGET)
        quiet(0, 0)
        report['checks']['invalid_drag_restores_server_pose'] = True

        report['stage'] = 'door_accept_and_global_route'
        before_accept_ns = node.get_clock().now().nanoseconds
        send(InteractiveMarkerFeedback.MENU_SELECT, target_pose, menu_ids['accept'])
        accepted = wait_state(lambda value: value['accept_count'] == 1)
        until(lambda: len(events) == 1)
        after_accept_ns = node.get_clock().now().nanoseconds
        event = events[0]
        assert event.header.frame_id == 'map' and event.type.value == SemanticClass.K_DOOR
        assert [event.point.x, event.point.y, event.point.z] == list(TARGET)
        assert before_accept_ns <= stamp_ns(event.header.stamp) <= after_accept_ns
        assert accepted['last_accepted']['position'] == list(TARGET)
        connected = wait_state(lambda value: value['received'] == 1, 'semantic_state')
        assert connected['success'] and connected['target_id'] > 0, connected
        until(lambda: bool(paths))
        connection = xyz([item.pose for item in paths[-1].poses])
        assert len(connection) > 2 and connection[-1] == list(TARGET), connection
        assert connection[0] == list(SOURCE), connection
        assert any(point[1] > 1. for point in connection), connection
        request = PlannerGlobal.Request()
        request.id = connected['target_id']
        request.ignore_time = request.not_check_frontier = True
        route = call('gbplanner/global', request)
        assert len(route.path) > 2 and xyz(route.path)[-1] == list(TARGET), xyz(route.path)
        validation = ValidatePath.Request()
        validation.header.frame_id, validation.path = 'map', route.path
        validated = call('validate_path', validation)
        assert validated.valid, validated.reason
        report['steps'].append({'name': 'door_accept', 'marker': accepted, 'backend': connected,
                                'connection': connection, 'global_route': xyz(route.path),
                                'route_validation': validated.reason,
                                'semantic_markers': target_snapshot('door'),
                                'stamp_interval_ns': [before_accept_ns, after_accept_ns]})
        report['checks']['menu_accept_publishes_exact_stamped_semantic_point'] = True
        report['checks']['backend_search_inserts_semantic_global_route'] = True

        report['stage'] = 'same_point_staircase_accept'
        send(InteractiveMarkerFeedback.MENU_SELECT, target_pose, menu_ids['staircase'])
        wait_state(lambda value: value['selected_class'] == SemanticClass.K_STAIRCASE)
        quiet(1, 1)
        send(InteractiveMarkerFeedback.MENU_SELECT, target_pose, menu_ids['accept'])
        relabeled = wait_state(lambda value: value['accept_count'] == 2)
        until(lambda: len(events) == 2)
        staircase = wait_state(lambda value: value['received'] == 2, 'semantic_state')
        assert staircase['success'] and staircase['target_id'] == connected['target_id'], staircase
        assert events[1].type.value == SemanticClass.K_STAIRCASE
        assert [events[1].point.x, events[1].point.y, events[1].point.z] == list(TARGET)
        targets = staircase['targets']
        assert len(targets) == 1 and targets[0]['class'] == SemanticClass.K_STAIRCASE, targets
        assert targets[0]['position'] == list(TARGET)
        report['steps'].append({'name': 'same_point_staircase_accept',
                                'marker': relabeled, 'backend': staircase,
                                'semantic_markers': target_snapshot('stairs')})
        report['checks']['same_point_accept_replaces_semantic_class'] = True
        report['checks']['semantic_marker_snapshots_replace_old_class_with_exact_geometry_and_color'] = True

        report['stage'] = 'strict_5m_odometry_recenter'
        current_odom[:] = [SOURCE[0], SOURCE[1] + 5., SOURCE[2]]
        quiet(2, 2)
        boundary = state()
        assert boundary['recenter_count'] == 0 and boundary['odometry_anchor'] == list(SOURCE)
        assert position(server_marker()[0].pose) == list(TARGET)
        current_odom[1] += .1
        recentered = wait_state(lambda value: value['recenter_count'] == 1)
        marker, final_sequence = server_marker()
        assert position(marker.pose) == current_odom
        assert [marker.pose.orientation.x, marker.pose.orientation.y,
                marker.pose.orientation.z, marker.pose.orientation.w] == [0., 0., 0., 1.]
        assert recentered['selected_class'] == SemanticClass.K_STAIRCASE
        assert recentered['accept_count'] == 2 and recentered['last_accepted'] == relabeled['last_accepted']
        assert recentered['odometry_anchor'] == current_odom
        final_backend = quiet(2, 2)
        assert final_backend['targets'] == staircase['targets']
        assert final_sequence > sequence and any(value.type == value.UPDATE for value in updates)
        report['steps'].append({'name': 'odometry_recenter', 'at_5m': boundary,
                                'over_5m': recentered, 'server_sequence': final_sequence})
        report['checks']['strict_5m_recenter_preserves_class_and_does_not_publish'] = True
        report['checks']['native_server_update_stream'] = True
        report['status'] = 'passed'
        report['stage'] = 'completed'
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        if timer is not None:
            timer.cancel()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        # Signal entire groups, including ros2-run descendants. Both receive
        # SIGINT together; cleanup cannot accumulate a timeout per process.
        for process in processes:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        grace = min(started + 58., time.monotonic() + 1.)
        for process in processes:
            try:
                process.wait(timeout=max(0., grace - time.monotonic()))
            except subprocess.TimeoutExpired:
                pass
        for process in processes:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        for process in processes:
            try:
                process.wait(timeout=max(0., started + 59. - time.monotonic()))
            except subprocess.TimeoutExpired:
                pass
        for log in logs:
            log.close()
        report['semantic_events'] = [
            {'class': event.type.value, 'position': [event.point.x, event.point.y, event.point.z],
             'frame_id': event.header.frame_id, 'stamp_ns': stamp_ns(event.header.stamp)}
            for event in events]
        report['elapsed_seconds'] = time.monotonic() - started
        temporary = args.output.with_suffix('.tmp')
        temporary.write_text(json.dumps(report, indent=2) + '\n')
        temporary.replace(args.output)
    print(json.dumps({'status': report['status'], 'checks': report['checks'],
                      'error': report.get('error'), 'output': str(args.output)}, indent=2))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
