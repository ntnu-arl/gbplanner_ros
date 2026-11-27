#!/usr/bin/env python3
"""ROS 2 interactive source/target markers for the native search planner.

The two initial waypoints and marker colors follow PlannerControlInterface's
initIMarker. ROS 2's InteractiveMarkerServer owns the RViz update protocol.
"""
import copy
import json
import math

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker


MARKERS = {'source': 'wp_source', 'target': 'wp_target'}


def normalized_pose(pose):
    result = copy.deepcopy(pose)
    p, q = result.position, result.orientation
    if not all(math.isfinite(value) for value in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
        raise ValueError('nonfinite_pose')
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1e-8:
        raise ValueError('invalid_orientation')
    q.x, q.y, q.z, q.w = (value / norm for value in (q.x, q.y, q.z, q.w))
    return result


def pose_record(pose):
    p, q = pose.position, pose.orientation
    return {'position': [p.x, p.y, p.z], 'orientation': [q.x, q.y, q.z, q.w]}


class SearchWaypoints(Node):
    def __init__(self):
        super().__init__('gbplanner_search_waypoints')
        self.frame = self.declare_parameter('frame_id', 'map').value
        self.namespace = self.declare_parameter('marker_namespace', 'search_waypoints').value
        self.odom_timeout = float(self.declare_parameter('odom_timeout', 2.0).value)
        if not self.frame or not self.namespace or not math.isfinite(self.odom_timeout) or self.odom_timeout <= 0:
            raise ValueError('Invalid marker configuration')
        self.server = InteractiveMarkerServer(self, self.namespace)
        self.poses = {}
        self.version = 0
        self.versions = {'source': 0, 'target': 0}
        self.updates = {'initialization': 0, 'feedback': 0, 'setter': 0}
        self.rejected = 0
        self.last_reason = 'waiting_for_odometry'
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.waypoint_publishers = {
            key: self.create_publisher(PoseStamped, 'search_' + key, durable)
            for key in MARKERS}
        self.create_subscription(Odometry, 'odometry', self.receive_odometry, 10)
        for key in MARKERS:
            self.create_subscription(PoseStamped, 'search/' + key + '_pose',
                                     lambda message, role=key: self.set_waypoint(role, message), 10)
        self.create_service(Trigger, self.namespace + '/status', self.status)

    def header(self):
        result = Header()
        result.frame_id = self.frame
        result.stamp = self.get_clock().now().to_msg()
        return result

    def marker(self, role, pose):
        marker = InteractiveMarker()
        marker.header = self.header()
        marker.name = MARKERS[role]
        marker.description = role.capitalize() + ' Pose'
        marker.pose = copy.deepcopy(pose)
        marker.scale = 1.0
        box = Marker()
        box.type = Marker.CUBE
        box.pose.orientation.w = 1.0
        box.scale.x = box.scale.y = box.scale.z = .45
        box.color.r = .8 if role == 'source' else .1
        box.color.g = .8 if role == 'target' else .1
        box.color.b = .1
        box.color.a = 1.0
        visible = InteractiveMarkerControl()
        visible.name = 'waypoint'
        visible.always_visible = True
        visible.markers.append(box)
        marker.controls.append(visible)

        # Keep the original XY plane and Z drag controls. Additional X/Y axes
        # and rotation rings make all six pose degrees of freedom editable.
        plane = InteractiveMarkerControl()
        plane.name = 'move_xy'
        plane.orientation.y = plane.orientation.w = math.sqrt(.5)
        plane.orientation_mode = InteractiveMarkerControl.FIXED
        plane.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        marker.controls.append(plane)
        for axis, quaternion_axis in [('x', 'x'), ('y', 'z'), ('z', 'y')]:
            for operation, mode in [('move', InteractiveMarkerControl.MOVE_AXIS),
                                    ('rotate', InteractiveMarkerControl.ROTATE_AXIS)]:
                control = InteractiveMarkerControl()
                control.name = operation + '_' + axis
                control.orientation.w = math.sqrt(.5)
                setattr(control.orientation, quaternion_axis, math.sqrt(.5))
                control.orientation_mode = InteractiveMarkerControl.FIXED
                control.interaction_mode = mode
                marker.controls.append(control)
        return marker

    def publish(self, role):
        message = PoseStamped()
        message.header = self.header()
        message.pose = copy.deepcopy(self.poses[role])
        self.waypoint_publishers[role].publish(message)

    def receive_odometry(self, message):
        if self.poses:
            return  # User selections remain fixed when the robot moves.
        try:
            if message.header.frame_id != self.frame:
                raise ValueError('wrong_odometry_frame')
            current = normalized_pose(message.pose.pose)
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            now = self.get_clock().now().nanoseconds * 1e-9
            if stamp <= 0 or now - stamp > self.odom_timeout or stamp - now > .1:
                raise ValueError('stale_odometry')
        except ValueError as error:
            self.last_reason = str(error)
            return
        for role in MARKERS:
            pose = Pose()
            pose.position.x = current.position.x + (1.0 if role == 'target' else 0.0)
            pose.position.y = current.position.y
            # Preserve upstream's absolute 1.5 m initial waypoint height.
            pose.position.z = 1.5
            pose.orientation.w = 1.0
            self.poses[role] = pose
            self.server.insert(self.marker(role, pose), feedback_callback=self.feedback)
            self.versions[role] = 1
        self.server.applyChanges()
        self.version = 1
        self.updates['initialization'] = 1
        self.last_reason = 'initialized'
        for role in MARKERS:
            self.publish(role)

    def update_pose(self, role, pose, origin):
        changed = pose_record(self.poses[role]) != pose_record(pose)
        # Programmatic setters use the same server pose as RViz feedback.
        # There is no independent feedback subscriber or shadow GUI state.
        if not self.server.setPose(MARKERS[role], copy.deepcopy(pose), self.header()):
            raise RuntimeError('Interactive marker disappeared')
        self.server.applyChanges()
        self.poses[role] = copy.deepcopy(pose)
        if changed:
            self.version += 1
            self.versions[role] += 1
            self.updates[origin] += 1
        self.last_reason = origin + '_updated'
        self.publish(role)

    def reject(self, reason):
        self.rejected += 1
        self.last_reason = reason

    def set_waypoint(self, role, message):
        try:
            if not self.poses:
                raise ValueError('odometry_not_initialized')
            if message.header.frame_id != self.frame:
                raise ValueError('wrong_setter_frame')
            pose = normalized_pose(message.pose)
        except ValueError as error:
            self.reject(str(error))
            return
        self.update_pose(role, pose, 'setter')

    def feedback(self, message):
        if message.event_type not in (InteractiveMarkerFeedback.POSE_UPDATE,
                                      InteractiveMarkerFeedback.MOUSE_UP):
            return
        role = next((key for key, name in MARKERS.items() if name == message.marker_name), None)
        if role is None:
            return
        try:
            if message.header.frame_id != self.frame:
                raise ValueError('wrong_feedback_frame')
            pose = normalized_pose(message.pose)
        except ValueError as error:
            self.reject(str(error))
            # The server stages POSE_UPDATE before invoking this callback.
            # Replace that pending invalid update with the last accepted pose.
            self.server.setPose(MARKERS[role], copy.deepcopy(self.poses[role]), self.header())
            self.server.applyChanges()
            return
        self.update_pose(role, pose, 'feedback')

    def status(self, request, response):
        response.success = bool(self.poses)
        response.message = json.dumps({
            'initialized': bool(self.poses), 'frame_id': self.frame,
            'marker_namespace': self.namespace, 'marker_names': list(MARKERS.values()),
            'version': self.version, 'versions': self.versions,
            'server_sequence': self.server.seq_num, 'updates': self.updates,
            'rejected_updates': self.rejected, 'reason': self.last_reason,
            **{role: pose_record(pose) for role, pose in self.poses.items()}})
        return response

    def destroy_node(self):
        self.server.shutdown()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = SearchWaypoints()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
