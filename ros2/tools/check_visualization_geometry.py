#!/usr/bin/env python3
"""One native ROS run for geometry, late display, frontier retirement and reset recovery.

Uses a synthetic, world-zero sparse voxel snapshot and observed poses. It does
not start a simulator, command robot motion, or claim an RViz render check.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

RESOLUTION = .1
ROBOT_SIZE = [1.17, 1.17, .25]
CENTER_OFFSET = [.2, -.15, .015]


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    log_path = output.with_suffix('.log')
    if output.exists() or log_path.exists():
        parser.error('Use a new output name to retain previous evidence')
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from planner_msgs.msg import Graph
    from planner_msgs.srv import PlannerStringTrigger
    from planner_semantic_msgs.msg import SemanticPoint
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from visualization_msgs.msg import Marker, MarkerArray

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    started = time.monotonic()
    deadline = started + 65.
    report = {'status': 'failed', 'scope': __doc__, 'checks': {}}
    processes = []
    probe = None

    class Probe(Node):
        def __init__(self):
            super().__init__('visualization_geometry_probe')
            self.maps = self.create_publisher(PointCloud2, 'voxel_map', qos)
            self.odom = self.create_publisher(Odometry, 'odometry', 10)
            self.semantic = self.create_publisher(SemanticPoint, 'semantic_location', 10)
            self.geometry, self.accepted, self.graphs = [], [], []
            self.markers = {kind: [] for kind in ('map', 'robot', 'global', 'local')}
            self.create_subscription(String, 'visualization_geometry',
                                     lambda m: self.geometry.append(json.loads(m.data)), qos)
            self.create_subscription(PointCloud2, 'accepted_voxel_map', self.accepted.append, qos)
            self.create_subscription(Graph, 'global_graph', self.graphs.append, qos)
            for kind, topic in [('map', 'vis/voxel_map'), ('robot', 'vis/robot_state'),
                                ('global', 'vis/planning_global_graph'), ('local', 'vis/planning_graph')]:
                self.create_subscription(MarkerArray, topic,
                    lambda message, name=kind: self.markers[name].append(message), qos)
            self.rpc_clients = {}
            self.x = 0.
            self.odom_timer = self.create_timer(.05, self.publish_odometry)

        def spin_until(self, condition, timeout=12.):
            end = min(deadline, time.monotonic() + timeout)
            while not condition():
                for process in processes:
                    require(process.poll() is None, 'Native child exited before checks completed')
                if time.monotonic() >= end:
                    raise TimeoutError('Expected native ROS event did not arrive')
                rclpy.spin_once(self, timeout_sec=.05)

        def status(self, name):
            if name not in self.rpc_clients:
                self.rpc_clients[name] = self.create_client(Trigger, name)
            client = self.rpc_clients[name]
            self.spin_until(client.service_is_ready)
            future = client.call_async(Trigger.Request())
            self.spin_until(future.done)
            reply = future.result()
            require(reply.success, f'{name}: {reply.message}')
            return json.loads(reply.message)

        def storage(self, load, path):
            name = 'gbplanner/' + ('load_graph' if load else 'save_graph')
            if name not in self.rpc_clients:
                self.rpc_clients[name] = self.create_client(PlannerStringTrigger, name)
            client = self.rpc_clients[name]
            self.spin_until(client.service_is_ready)
            request = PlannerStringTrigger.Request()
            request.message = str(path)
            future = client.call_async(request)
            self.spin_until(future.done)
            require(future.result().success, f'{name} failed')

        def publish_odometry(self):
            message = Odometry()
            message.header.frame_id = 'map'
            message.header.stamp = self.get_clock().now().to_msg()
            message.child_frame_id = 'base_link'
            message.pose.pose.position.x = self.x
            message.pose.pose.orientation.z = math.sin(math.pi / 4)
            message.pose.pose.orientation.w = math.cos(math.pi / 4)
            self.odom.publish(message)

    def fixture(empty=False):
        message = PointCloud2()
        message.header.frame_id = 'map'
        message.header.stamp = probe.get_clock().now().to_msg()
        message.height = 1
        message.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                          for i, name in enumerate(('x', 'y', 'z'))]
        message.fields.append(PointField(name='occupancy', offset=12, datatype=PointField.INT8, count=1))
        message.point_step = 16
        data = bytearray()
        if not empty:
            for x in range(-10, 60):
                for y in range(-15, 15):
                    for z in range(-6, 6):
                        point = [(i + .5) * RESOLUTION for i in (x, y, z)]
                        occupied = x == 58 and y == 13 and z == 4
                        data.extend(struct.pack('<fffbxxx', *point, 100 if occupied else 0))
        message.data = bytes(data)
        message.width = len(data) // 16
        message.row_step = len(data)
        message.is_dense = True
        return message

    def robot_marker():
        if not probe.markers['robot']:
            return None
        return next((m for m in probe.markers['robot'][-1].markers
                     if m.action == Marker.ADD and m.type == Marker.CUBE), None)

    def shown_frontiers():
        if not probe.markers['global']:
            return None
        return sum(len(m.points) for m in probe.markers['global'][-1].markers
                   if m.action == Marker.ADD and m.id == 2)

    def shown_vertices(snapshots):
        if not snapshots:
            return None
        return sum(len(marker.points) for marker in snapshots[-1].markers
                   if marker.action == Marker.ADD and marker.id == 1)

    with tempfile.TemporaryDirectory(prefix='gbplanner-vis-geometry-') as directory:
        parameters = Path(directory) / 'parameters.json'
        parameters.write_text(json.dumps({'gbplanner_local': {'ros__parameters': {
            'map.storage': 'sparse', 'map.resolution': RESOLUTION,
            'robot.size': ROBOT_SIZE, 'robot.center_offset': CENTER_OFFSET,
            'global.enable': True, 'global.background_enable': False,
            'planning.global_min': [-1., -1.5, -.6], 'planning.global_max': [6., 1.5, .6],
        }}}))
        with log_path.open('w') as log:
            try:
                rclpy.init(args=[])
                probe = Probe()
                processes.append(subprocess.Popen(['ros2', 'run', 'gbplanner_local', 'local_planner_node',
                    '--ros-args', '--params-file', str(parameters)], stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True))
                probe.spin_until(lambda: bool(probe.geometry))
                metadata = probe.geometry[-1]
                require(metadata == {'frame_id': 'map', 'map_resolution': RESOLUTION,
                    'robot_size': ROBOT_SIZE, 'robot_center_offset': CENTER_OFFSET}, metadata)
                probe.spin_until(lambda: probe.maps.get_subscription_count() > 0)
                observed = fixture()
                probe.maps.publish(observed)
                probe.spin_until(lambda: bool(probe.accepted))
                require(probe.status('map_status')['known_cells'] == observed.width,
                        'Sparse fixture was not fully accepted by the real planner')
                probe.spin_until(lambda: probe.status('global_status')['vertices'] == 1)
                probe.spin_until(lambda: probe.semantic.get_subscription_count() > 0)
                semantic = SemanticPoint()
                semantic.header.frame_id = 'map'
                semantic.header.stamp = probe.get_clock().now().to_msg()
                semantic.type.value = semantic.type.K_DOOR
                semantic.point.x = 4.
                probe.semantic.publish(semantic)
                probe.spin_until(lambda: probe.status('semantic_state')['received'] == 1)
                require(probe.status('semantic_state')['success'], 'Native semantic route was rejected')
                probe.spin_until(lambda: probe.graphs and sum(v.is_frontier for v in probe.graphs[-1].vertices) == 1)

                # The consumer joins after geometry, map and graph publication.
                # Default fallback dimensions are deliberately left unconfigured.
                processes.append(subprocess.Popen(['ros2', 'run', 'gbplanner_local', 'visualization_node.py'],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
                probe.spin_until(lambda: probe.markers['map'] and
                    any(m.action == Marker.ADD for m in probe.markers['map'][-1].markers))
                probe.spin_until(lambda: robot_marker() is not None and shown_frontiers() == 1)
                display = probe.status('visualization_status')
                require(display['geometry_source'] == 'planner_metadata' and
                        display['map_resolution'] == RESOLUTION and display['robot_size'] == ROBOT_SIZE,
                        display)
                for snapshot in probe.markers['map']:
                    for marker in snapshot.markers:
                        if marker.action == Marker.ADD:
                            require([marker.scale.x, marker.scale.y, marker.scale.z] == [RESOLUTION]*3,
                                    'Map was displayed with fallback-sized cubes')
                robot = robot_marker()
                require([robot.scale.x, robot.scale.y, robot.scale.z] == ROBOT_SIZE,
                        'Physical robot box has incorrect axis sizes')
                require([robot.pose.position.x, robot.pose.position.y, robot.pose.position.z] == CENTER_OFFSET,
                        'World center offset was missing, rotated or applied more than once')
                require(abs(robot.pose.orientation.z - math.sin(math.pi/4)) < 1e-12 and
                        abs(robot.pose.orientation.w - math.cos(math.pi/4)) < 1e-12,
                        'Measured robot orientation was changed')
                report['checks']['late_metadata_and_map_rebuild'] = True
                report['checks']['native_map_and_physical_robot_dimensions'] = True
                report['checks']['world_offset_and_measured_orientation'] = True

                probe.x = .999
                probe.spin_until(lambda: robot_marker() is not None and
                    abs(robot_marker().pose.position.x - (.999 + CENTER_OFFSET[0])) < 1e-10)
                require(probe.status('global_status')['frontiers'] == 1, 'Frontier retired below E1 threshold')
                probe.x = 1.
                probe.spin_until(lambda: probe.status('global_status')['frontiers'] == 0)
                probe.spin_until(lambda: probe.graphs and not any(v.is_frontier for v in probe.graphs[-1].vertices)
                                 and shown_frontiers() == 0)
                report['checks']['visited_frontier_removed_from_native_display'] = True

                probe.odom_timer.cancel()
                prior = len(probe.accepted)
                probe.maps.publish(fixture(empty=True))
                probe.spin_until(lambda: len(probe.accepted) > prior)
                probe.spin_until(lambda: probe.status('visualization_status')['known_cells'] == 0)
                probe.spin_until(lambda: all(snapshots and len(snapshots[-1].markers) == 1 and
                    snapshots[-1].markers[0].action == Marker.DELETEALL
                    for snapshots in probe.markers.values()))
                late = []
                probe.create_subscription(MarkerArray, 'vis/robot_state', late.append, qos)
                probe.spin_until(lambda: bool(late))
                require(len(late[-1].markers) == 1 and late[-1].markers[0].action == Marker.DELETEALL,
                        'Late subscriber retained a robot from before reset')
                report['checks']['reset_and_late_subscriber_clear'] = True

                # Resume observations after the reset. The planner publishes
                # graph and accepted-map snapshots on independent topics, so
                # either can reach the display adapter first.
                prior = len(probe.accepted)
                probe.maps.publish(fixture())
                probe.odom_timer.reset()
                probe.spin_until(lambda: len(probe.accepted) > prior and probe.accepted[-1].width == observed.width)
                probe.spin_until(lambda: probe.graphs and len(probe.graphs[-1].vertices) == 1)
                probe.spin_until(lambda: shown_vertices(probe.markers['global']) == 1)
                probe.spin_until(lambda: robot_marker() is not None and
                    probe.status('visualization_status')['known_cells'] == observed.width)
                report['checks']['reset_known_map_and_home_graph_recovery'] = True

                # A graph restored while the map is unavailable is retained
                # by the planner. Further empty maps must not erase its display.
                saved_graph = Path(directory) / 'home.graph'
                probe.storage(False, saved_graph)
                probe.odom_timer.cancel()
                prior = len(probe.accepted)
                probe.maps.publish(fixture(empty=True))
                probe.spin_until(lambda: len(probe.accepted) > prior and probe.accepted[-1].width == 0)
                probe.spin_until(lambda: probe.status('visualization_status')['known_cells'] == 0 and
                    probe.graphs and not probe.graphs[-1].vertices and
                    shown_vertices(probe.markers['global']) == 0)
                probe.storage(True, saved_graph)
                probe.spin_until(lambda: probe.graphs and len(probe.graphs[-1].vertices) == 1 and
                    shown_vertices(probe.markers['global']) == 1)
                require(probe.status('global_graph_storage')['awaiting_map'],
                        'Loaded graph did not enter the awaiting-map state')
                prior = len(probe.accepted)
                prior_updates = probe.status('visualization_status')['map_updates']
                probe.maps.publish(fixture(empty=True))
                probe.spin_until(lambda: len(probe.accepted) > prior and probe.accepted[-1].width == 0)
                probe.spin_until(lambda: probe.status('visualization_status')['map_updates'] > prior_updates)
                require(len(probe.graphs[-1].vertices) == 1 and shown_vertices(probe.markers['global']) == 1,
                        'An empty map erased the loaded home graph display')
                report['checks']['loaded_home_survives_empty_map'] = True

                # A new consumer receives durable geometry, an empty accepted
                # map and the loaded graph. Their inter-topic arrival order is
                # unspecified, but the retained graph must still be displayed.
                late_graphs = []
                probe.create_subscription(MarkerArray, 'late_vis/planning_global_graph', late_graphs.append, qos)
                remaps = ['__node:=gbplanner_visualization_late',
                          'visualization_status:=late_visualization_status']
                remaps += [f'vis/{name}:=late_vis/{name}' for name in
                           ('planning_graph', 'planning_global_graph', 'voxel_map', 'robot_state')]
                command = ['ros2', 'run', 'gbplanner_local', 'visualization_node.py', '--ros-args']
                for remap in remaps:
                    command.extend(['-r', remap])
                processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True))
                probe.spin_until(lambda: probe.status('late_visualization_status')['map_updates'] > 0 and
                    shown_vertices(late_graphs) == 1)
                late_display = probe.status('late_visualization_status')
                require(late_display['geometry_source'] == 'planner_metadata' and late_display['known_cells'] == 0,
                        'Late consumer did not restore the empty-map scene from planner metadata')
                report['checks']['late_loaded_home_with_empty_map'] = True
                report.update(status='passed', metadata=metadata, accepted_cells=observed.width,
                              final_display=probe.status('visualization_status'), late_display=late_display)
            except Exception as error:
                report['error'] = f'{type(error).__name__}: {error}'
            finally:
                exits, forced = [], False
                for process in reversed(processes):
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        try:
                            process.wait(timeout=8)
                        except subprocess.TimeoutExpired:
                            forced = True
                            os.killpg(process.pid, signal.SIGTERM)
                            try:
                                process.wait(timeout=4)
                            except subprocess.TimeoutExpired:
                                os.killpg(process.pid, signal.SIGKILL)
                                process.wait(timeout=4)
                    exits.append(process.returncode)
                if probe:
                    probe.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                report.update(child_returncodes=exits, forced_termination=forced,
                              elapsed_s=time.monotonic()-started, log=str(log_path))
                if forced or any(code != 0 for code in exits):
                    report['status'] = 'failed'
                    report.setdefault('error', 'Native child failed orderly shutdown')
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
