#!/usr/bin/env python3
"""ROS 2 display adapter for the migrated graph/map/path interfaces.

This is a new adapter, not a port of every upstream Visualization method.
Only planner-accepted maps are displayed; display sampling never changes maps
used for collision checking. Each topic contains a complete marker snapshot.
"""
from collections import deque
import copy
import json
import math
import signal
import struct

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from planner_msgs.msg import Graph
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


def finite_point(p):
    return all(math.isfinite(v) for v in (p.x, p.y, p.z))


class PlannerVisualization(Node):
    def __init__(self):
        super().__init__('gbplanner_visualization')
        self.frame = self.declare_parameter('frame_id', 'map').value
        self.resolution = float(self.declare_parameter('map_resolution', .25).value)
        fallback_robot_size = float(self.declare_parameter('robot_size', .4).value)
        self.robot_size = [fallback_robot_size] * 3
        self.robot_center_offset = [0., 0., 0.]
        self.use_geometry_metadata = self.declare_parameter('use_geometry_metadata', True).value
        # Surface-cloud RViz views do not need an additional scan through the
        # full occupancy map just to draw disabled voxel markers.
        self.display_voxel_map = self.declare_parameter('display_voxel_map', True).value
        self.display_robot_box = self.declare_parameter('display_robot_box', True).value
        self.geometry_source = 'waiting_for_planner' if self.use_geometry_metadata else 'parameters'
        self.geometry_ready = not self.use_geometry_metadata
        self.latest_map = self.latest_odometry = None
        self.latest_graphs = {}
        self.max_points = int(self.declare_parameter('max_map_points', 40000).value)
        self.max_snapshot_voxels = int(self.declare_parameter('map.max_snapshot_voxels', 4000000).value)
        self.history_limit = int(self.declare_parameter('history_limit', 5000).value)
        if (not self.frame or any(not math.isfinite(x) or x <= 0 for x in
                (self.resolution, fallback_robot_size)) or self.max_points < 1 or
                not 1 <= self.max_snapshot_voxels <= 100000000 or self.history_limit < 2):
            raise ValueError('Invalid visualization parameters')
        self.history = deque(maxlen=self.history_limit)
        self.known = False
        self.stats = {'map_updates': 0, 'local_updates': 0, 'global_updates': 0, 'search_updates': 0,
                      'odom_updates': 0, 'rejected': 0, 'known_cells': 0,
                      'displayed_cells': 0, 'map_decimated': False, 'geometry_updates': 0}
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publishers_by_kind = {kind: self.create_publisher(MarkerArray, topic, durable)
            for kind, topic in [('local', 'vis/planning_graph'),
                                ('global', 'vis/planning_global_graph'),
                                ('search', 'vis/search_graph'),
                                ('map', 'vis/voxel_map'), ('robot', 'vis/robot_state')]}
        if self.use_geometry_metadata:
            self.create_subscription(String, 'visualization_geometry', self.geometry, durable)
        self.create_subscription(Graph, 'local_graph', lambda m: self.graph(m, 'local'), durable)
        self.create_subscription(Graph, 'global_graph', lambda m: self.graph(m, 'global'), durable)
        self.create_subscription(Graph, 'search_graph', lambda m: self.graph(m, 'search'), durable)
        if self.display_voxel_map:
            self.create_subscription(PointCloud2, 'accepted_voxel_map', self.map, durable)
        else:
            # Keep map readiness and reset lifecycle without deserializing the
            # full cloud. The planner emits these counters from its accepted map.
            self.create_subscription(String, 'map_updates', self.map_observation, durable)
        self.create_subscription(Odometry, 'odometry', self.odometry, 10)
        self.create_service(Trigger, 'visualization_status', self.status)
        for kind in self.publishers_by_kind:
            self.publish(kind, [])

    def marker(self, namespace, identity, kind, size, rgba, stamp):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = stamp
        marker.ns, marker.id, marker.type = namespace, identity, kind
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = float(size)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        return marker

    def publish(self, kind, markers):
        # DELETEALL and all current markers are in the SAME transient-local
        # message. Late subscribers receive a complete scene, never a delta.
        clear = Marker()
        clear.header.frame_id = self.frame
        clear.action = Marker.DELETEALL
        self.publishers_by_kind[kind].publish(MarkerArray(markers=[clear] + markers))

    def reject(self, error):
        self.stats['rejected'] += 1
        self.get_logger().warning(str(error))

    def geometry(self, message):
        try:
            data = json.loads(message.data)
            frame = data['frame_id']
            resolution = float(data['map_resolution'])
            size = [float(value) for value in data['robot_size']]
            offset = [float(value) for value in data['robot_center_offset']]
            if (not isinstance(frame, str) or not frame or len(size) != 3 or len(offset) != 3
                    or not math.isfinite(resolution) or resolution <= 0
                    or any(not math.isfinite(value) or value <= 0 for value in size)
                    or any(not math.isfinite(value) for value in offset)):
                raise ValueError('Planner visualization geometry is invalid')
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            self.reject(error)
            return
        unchanged = (self.geometry_ready and self.frame == frame and self.resolution == resolution
                     and self.robot_size == size and self.robot_center_offset == offset)
        if unchanged:
            return
        if frame != self.frame:
            self.history.clear()
            self.known = False
            for kind in self.publishers_by_kind:
                self.publish(kind, [])
        self.frame, self.resolution = frame, resolution
        self.robot_size, self.robot_center_offset = size, offset
        self.geometry_ready = True
        self.geometry_source = 'planner_metadata'
        self.stats['geometry_updates'] += 1
        # The planner may have published its durable map before this node
        # joined. Replay the retained inputs only after their dimensions are
        # known; an initial fallback-sized scene must never flash on screen.
        if self.latest_map is not None:
            self.map(self.latest_map)
        # Loaded or suspended graphs remain useful even without a known map.
        # Geometry metadata is the only prerequisite for drawing a graph.
        for kind, graph in list(self.latest_graphs.items()):
            self.graph(graph, kind)

    def graph(self, message, kind):
        if not self.geometry_ready:
            self.latest_graphs[kind] = message
            return
        try:
            if message.header.frame_id != self.frame or len(message.vertices) > 20000 or len(message.edges) > 100000:
                raise ValueError('Graph frame or size invalid')
            vertices = {}
            for v in message.vertices:
                if v.id in vertices or not finite_point(v.pose.position):
                    raise ValueError('Graph has duplicate IDs or invalid positions')
                vertices[v.id] = v
            for edge in message.edges:
                if (edge.source_id not in vertices or edge.target_id not in vertices
                        or not math.isfinite(edge.weight) or edge.weight < 0):
                    raise ValueError('Graph edge is invalid')
            color = {'local': (.15, .48, .78, .75),
                     'global': (.65, .38, .16, .75),
                     'search': (.48, .28, .72, .75)}[kind]
            stamp = message.header.stamp
            edges = self.marker(kind, 0, Marker.LINE_LIST, .025, color, stamp)
            dots = self.marker(kind, 1, Marker.SPHERE_LIST, .075, color, stamp)
            frontier = self.marker(kind, 2, Marker.SPHERE_LIST, .17, (.1, .8, .3, 1.), stamp)
            for v in vertices.values():
                dots.points.append(v.pose.position)
                if v.is_frontier:
                    frontier.points.append(v.pose.position)
            for edge in message.edges:
                edges.points.extend([vertices[edge.source_id].pose.position,
                                     vertices[edge.target_id].pose.position])
            self.publish(kind, [m for m in (edges, dots, frontier) if m.points])
            self.latest_graphs[kind] = message
            self.stats[kind + '_updates'] += 1
        except (ValueError, TypeError) as error:
            # Invalid snapshots never replace the last valid display.
            self.reject(error)

    def map(self, message):
        if not self.geometry_ready:
            self.latest_map = message
            return
        try:
            count = message.width * message.height
            if (message.header.frame_id != self.frame or message.is_bigendian or
                    message.height < 1 or message.point_step < 1 or count > self.max_snapshot_voxels or
                    message.row_step < message.width * message.point_step or
                    len(message.data) != message.row_step * message.height):
                raise ValueError('Map frame, shape or byte count invalid')
            offsets, occupied_bytes = {}, set()
            for name, datatype, length in [('x', PointField.FLOAT32, 4), ('y', PointField.FLOAT32, 4),
                                           ('z', PointField.FLOAT32, 4), ('occupancy', PointField.INT8, 1)]:
                fields = [f for f in message.fields if f.name == name]
                if len(fields) != 1:
                    raise ValueError('Map field missing or duplicated')
                field = fields[0]
                span = set(range(field.offset, field.offset + length))
                if (field.datatype != datatype or field.count != 1 or
                        field.offset + length > message.point_step or span & occupied_bytes):
                    raise ValueError('Map field layout invalid')
                occupied_bytes.update(span)
                offsets[name] = field.offset
            counts = {0: 0, 100: 0}
            for row in range(message.height):
                for col in range(message.width):
                    if col % 1024 == 0 and not rclpy.ok():
                        return
                    base = row * message.row_step + col * message.point_step
                    xyz = [struct.unpack_from('<f', message.data, base + offsets[k])[0] for k in ('x', 'y', 'z')]
                    state = struct.unpack_from('<b', message.data, base + offsets['occupancy'])[0]
                    if not all(math.isfinite(v) for v in xyz) or state not in (-1, 0, 100):
                        raise ValueError('Map has invalid position or occupancy')
                    if state != -1:
                        counts[state] += 1
            known = counts[0] + counts[100]
            sample_counts = {100: min(self.max_points, counts[100]),
                             0: min(max(0, self.max_points-counts[100]), counts[0])}
            markers = []
            for state, rgba in [(100, (.22, .27, .33, .95)), (0, (.3, .65, .85, .07))]:
                # Occupied cells take priority. Deterministic even sampling is
                # for display only; collision checks retain the full map.
                n = sample_counts[state]
                if n:
                    marker = self.marker('map', state, Marker.CUBE_LIST, self.resolution,
                                         rgba, message.header.stamp)
                    marker.points = []
                    markers.append(marker)
            targets = {state: 0 for state in (0, 100)}
            selected = {state: 0 for state in (0, 100)}
            marker_by_state = {marker.id: marker for marker in markers}
            for row in range(message.height):
                for col in range(message.width):
                    if col % 1024 == 0 and not rclpy.ok():
                        return
                    base = row * message.row_step + col * message.point_step
                    state = struct.unpack_from('<b', message.data, base + offsets['occupancy'])[0]
                    n = sample_counts.get(state, 0)
                    if not n:
                        continue
                    index = selected[state]
                    target = targets[state]
                    if target < n and index == target * counts[state] // n:
                        xyz = [struct.unpack_from('<f', message.data, base + offsets[k])[0]
                               for k in ('x', 'y', 'z')]
                        marker_by_state[state].points.append(Point(x=xyz[0], y=xyz[1], z=xyz[2]))
                        targets[state] += 1
                    selected[state] += 1
            if not rclpy.ok():
                return
            self.publish('map', markers)
            self.latest_map = message
            self.known = known > 0
            if not self.known:
                self.history.clear()
                self.latest_odometry = None
                self.publish('robot', [])
                # Graph topics own their lifecycles. A normal reset publishes
                # empty Graph snapshots; an imported graph awaiting its first
                # map remains valid and must stay visible across empty maps.
            self.stats.update(map_updates=self.stats['map_updates']+1, known_cells=known,
                              displayed_cells=sum(sample_counts.values()),
                              map_decimated=known > self.max_points)
            if self.known and self.latest_odometry is not None:
                self.odometry(self.latest_odometry)
        except (ValueError, TypeError, struct.error) as error:
            self.reject(error)

    def map_observation(self, message):
        try:
            update = json.loads(message.data)
            known = update['known_cells']
            if type(known) is not int or known < 0:
                raise ValueError('Invalid map observation count')
            self.known = known > 0
            self.stats.update(map_updates=self.stats['map_updates']+1, known_cells=known,
                              displayed_cells=0, map_decimated=False)
            if not self.known:
                self.history.clear()
                self.latest_odometry = None
                self.publish('robot', [])
            elif self.latest_odometry is not None:
                self.odometry(self.latest_odometry)
        except (ValueError, TypeError, KeyError) as error:
            self.reject(error)

    def odometry(self, message):
        if not self.geometry_ready:
            self.latest_odometry = message
            return
        pose, stamp = message.pose.pose, message.header.stamp
        q = pose.orientation
        if (message.header.frame_id != self.frame or not finite_point(pose.position) or
                not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)) or
                q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w < 1e-12):
            self.reject('Odometry frame or pose invalid')
            return
        self.latest_odometry = message
        if not self.known:
            return
        p = pose.position
        if not self.history or math.dist((p.x, p.y, p.z), (self.history[-1].x, self.history[-1].y, self.history[-1].z)) >= .05:
            self.history.append(p)
        robot = self.marker('robot', 0, Marker.CUBE, 1., (.8, .18, .16, .9), stamp)
        robot.scale.x, robot.scale.y, robot.scale.z = self.robot_size
        robot.pose = copy.deepcopy(pose)
        # Match the planner's world-axis center offset. The physical box keeps
        # the measured orientation; history remains at the unshifted odometry.
        robot.pose.position.x += self.robot_center_offset[0]
        robot.pose.position.y += self.robot_center_offset[1]
        robot.pose.position.z += self.robot_center_offset[2]
        trace = self.marker('robot', 1, Marker.LINE_STRIP, .045, (.75, .18, .65, 1.), stamp)
        trace.points = list(self.history)
        self.publish('robot', ([robot] if self.display_robot_box else []) +
                     ([trace] if len(trace.points) > 1 else []))
        self.stats['odom_updates'] += 1

    def status(self, request, response):
        response.success = True
        response.message = json.dumps(dict(self.stats, history_points=len(self.history),
            geometry_source=self.geometry_source, frame_id=self.frame,
            display_voxel_map=self.display_voxel_map,
            display_robot_box=self.display_robot_box,
            map_resolution=self.resolution, max_snapshot_voxels=self.max_snapshot_voxels,
            robot_size=self.robot_size,
            robot_center_offset=self.robot_center_offset))
        return response


def main():
    rclpy.init()
    node = PlannerVisualization()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
