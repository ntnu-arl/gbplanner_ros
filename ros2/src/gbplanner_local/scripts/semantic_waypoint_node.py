#!/usr/bin/env python3
"""Interactive semantic point selection for the existing planning backend.

The marker, class menu and explicit Accept action follow the original PCI.
Only Accept publishes: moving the marker or choosing its class is a local edit.
"""
import copy
import json
import math

import rclpy
from geometry_msgs.msg import Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from nav_msgs.msg import Odometry
from planner_semantic_msgs.msg import SemanticClass, SemanticPoint
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

from search_waypoints_node import normalized_pose, pose_record


class SemanticWaypoint(Node):
    MARKER_NAME = 'semantic'

    def __init__(self):
        super().__init__('gbplanner_semantic_waypoint')
        self.frame = self.declare_parameter('frame_id', 'map').value
        self.namespace = self.declare_parameter('marker_namespace', 'semantics').value
        self.odom_timeout = float(self.declare_parameter('odom_timeout', 2.0).value)
        if not self.frame or not self.namespace or not math.isfinite(self.odom_timeout) or self.odom_timeout <= 0:
            raise ValueError('Invalid semantic marker configuration')
        self.server = InteractiveMarkerServer(self, self.namespace)
        self.menu = MenuHandler()
        self.menu_ids = {'class': self.menu.insert('Class')}
        self.menu_ids['staircase'] = self.menu.insert(
            'Staircase', parent=self.menu_ids['class'], callback=self.select_class)
        self.menu_ids['door'] = self.menu.insert(
            'Door', parent=self.menu_ids['class'], callback=self.select_class)
        self.menu_ids['accept'] = self.menu.insert('Accept', callback=self.accept)
        self.classes = {
            self.menu_ids['staircase']: (SemanticClass.K_STAIRCASE, 'Staircase'),
            self.menu_ids['door']: (SemanticClass.K_DOOR, 'Door'),
        }
        for entry in self.classes:
            self.menu.setCheckState(entry, MenuHandler.UNCHECKED)
        self.selected_class = SemanticClass.K_NONE
        self.selected_label = 'None'
        self.pose = None
        self.odometry_anchor = None
        self.recenter_count = 0
        self.pose_updates = 0
        self.accept_count = 0
        self.rejected_updates = 0
        self.last_reason = 'waiting_for_odometry'
        self.last_accepted = None
        # Semantic submissions are events. Do not replay the last accepted
        # object when a planner reconnects or a new subscriber appears.
        self.publisher = self.create_publisher(SemanticPoint, 'semantic_location', 10)
        self.create_subscription(Odometry, 'odometry', self.receive_odometry, 10)
        self.create_service(Trigger, self.namespace + '/status', self.status)

    def header(self):
        header = Header()
        header.frame_id = self.frame
        header.stamp = self.get_clock().now().to_msg()
        return header

    def marker(self):
        marker = InteractiveMarker()
        marker.header = self.header()
        marker.name = self.MARKER_NAME
        marker.pose = copy.deepcopy(self.pose)
        marker.scale = 1.0
        for axis in ('x', 'y', 'z'):
            control = InteractiveMarkerControl()
            control.name = axis + '_control'
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            control.orientation.w = 1.0
            if axis != 'x':
                control.orientation.w = math.sqrt(.5)
                setattr(control.orientation, 'z' if axis == 'y' else 'y', math.sqrt(.5))
            marker.controls.append(control)
        cube = Marker()
        cube.type = Marker.CUBE
        cube.pose.orientation.w = 1.0
        cube.scale.x = cube.scale.y = cube.scale.z = 1.0
        cube.color.r = cube.color.g = cube.color.b = .5
        cube.color.a = .65
        button = InteractiveMarkerControl()
        button.name = 'button_control'
        button.description = 'menu_button'
        button.interaction_mode = InteractiveMarkerControl.BUTTON
        button.always_visible = True
        button.markers.append(cube)
        marker.controls.append(button)
        return marker

    def receive_odometry(self, message):
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
        position = (current.position.x, current.position.y, current.position.z)
        if self.pose is not None:
            # Original odometryCallback/processPose moves the marker to the
            # robot only after travelling strictly more than 5 m in 3D since
            # the previous recenter. Smaller moves preserve the user's edit.
            if math.dist(position, self.odometry_anchor) <= 5.0:
                return
            self.pose = Pose()
            self.pose.position = copy.deepcopy(current.position)
            self.pose.orientation.w = 1.0
            self.odometry_anchor = position
            self.recenter_count += 1
            self.server.setPose(self.MARKER_NAME, copy.deepcopy(self.pose), self.header())
            self.server.applyChanges()
            self.last_reason = 'recentered_on_odometry'
            return
        # Original initSemanticIMarker adds 1.5 m along world X, not body X;
        # unlike search waypoints, it retains the robot's current height.
        self.pose = Pose()
        self.pose.position.x = current.position.x + 1.5
        self.pose.position.y = current.position.y
        self.pose.position.z = current.position.z
        self.pose.orientation.w = 1.0
        self.odometry_anchor = position
        self.server.insert(self.marker(), feedback_callback=self.feedback)
        self.menu.apply(self.server, self.MARKER_NAME)
        self.server.applyChanges()
        self.last_reason = 'initialized'

    def reject(self, reason):
        self.rejected_updates += 1
        self.last_reason = reason

    def checked_feedback(self, message):
        if self.pose is None:
            raise ValueError('odometry_not_initialized')
        if message.marker_name != self.MARKER_NAME:
            raise ValueError('wrong_marker_name')
        if message.header.frame_id != self.frame:
            raise ValueError('wrong_feedback_frame')
        return normalized_pose(message.pose)

    def feedback(self, message):
        if message.event_type not in (InteractiveMarkerFeedback.POSE_UPDATE,
                                      InteractiveMarkerFeedback.MOUSE_UP):
            return
        try:
            pose = self.checked_feedback(message)
        except ValueError as error:
            self.reject(str(error))
            # The server stages POSE_UPDATE before this callback. Restore its
            # pending pose too, so an invalid drag cannot enter a later Accept.
            if self.pose is not None:
                self.server.setPose(self.MARKER_NAME, copy.deepcopy(self.pose), self.header())
                self.server.applyChanges()
            return
        if pose_record(pose) != pose_record(self.pose):
            self.pose_updates += 1
        self.pose = pose
        self.server.setPose(self.MARKER_NAME, copy.deepcopy(pose), self.header())
        self.server.applyChanges()
        self.last_reason = 'pose_updated'

    def select_class(self, message):
        try:
            self.checked_feedback(message)
        except ValueError as error:
            self.reject(str(error))
            return
        entry = message.menu_entry_id
        if entry not in self.classes:
            return
        self.selected_class, self.selected_label = self.classes[entry]
        for candidate in self.classes:
            self.menu.setCheckState(candidate, MenuHandler.CHECKED if candidate == entry else MenuHandler.UNCHECKED)
        self.menu.reApply(self.server)
        self.server.applyChanges()
        self.last_reason = 'class_selected'

    def accept(self, feedback):
        try:
            self.checked_feedback(feedback)
            # ROS 2 adaptation: the backend accepts Door/Staircase only. Keep
            # the original None default, but require a class before submission.
            if self.selected_class == SemanticClass.K_NONE:
                raise ValueError('semantic_class_not_selected')
            marker = self.server.get(self.MARKER_NAME)
            if marker is None or marker.header.frame_id != self.frame:
                raise ValueError('semantic_marker_unavailable')
            pose = normalized_pose(marker.pose)
        except ValueError as error:
            self.reject(str(error))
            return
        message = SemanticPoint()
        message.header = self.header()
        message.type.value = self.selected_class
        message.point.x = pose.position.x
        message.point.y = pose.position.y
        message.point.z = pose.position.z
        self.publisher.publish(message)
        self.accept_count += 1
        self.last_reason = 'semantic_point_published'
        self.last_accepted = {
            'class': self.selected_class,
            'position': [message.point.x, message.point.y, message.point.z],
            'frame_id': message.header.frame_id,
            'stamp': {'sec': message.header.stamp.sec, 'nanosec': message.header.stamp.nanosec},
        }

    def status(self, request, response):
        response.success = self.pose is not None
        response.message = json.dumps({
            'initialized': self.pose is not None,
            'frame_id': self.frame,
            'marker_namespace': self.namespace,
            'marker_name': self.MARKER_NAME,
            'menu_ids': self.menu_ids,
            'selected_class': self.selected_class,
            'selected_label': self.selected_label,
            'pose': pose_record(self.pose) if self.pose is not None else None,
            'pose_updates': self.pose_updates,
            'odometry_anchor': self.odometry_anchor,
            'recenter_count': self.recenter_count,
            'accept_count': self.accept_count,
            'rejected_updates': self.rejected_updates,
            'reason': self.last_reason,
            'last_accepted': self.last_accepted,
            'server_sequence': self.server.seq_num,
        })
        return response

    def destroy_node(self):
        self.server.shutdown()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = SemanticWaypoint()
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
