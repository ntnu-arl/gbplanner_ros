#!/usr/bin/env python3
"""Expose native Gazebo observations through the planner's frame contract.

No vehicle integration or lidar synthesis occurs here. Source simulation
timestamps are retained. The robot description publishes fixed/joint TF; only
the physical base's map transform is published here.
"""
import copy
import json
import math
import signal
import struct

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ros_gz_interfaces.msg import Contacts
from tf2_ros import TransformBroadcaster


class FortressState(Node):
    def __init__(self):
        super().__init__('fortress_state')
        self.frame = self.declare_parameter('frame_id', 'map').value
        self.base = self.declare_parameter('robot_frame', 'base_link').value
        self.sensor = self.declare_parameter('sensor_frame', 'smb_lidar').value
        qos_descriptor = ParameterDescriptor(read_only=True)
        self.pointcloud_reliability = self.declare_parameter(
            'pointcloud_reliability', 'best_effort', qos_descriptor).value
        self.pointcloud_queue_size = self.declare_parameter(
            'pointcloud_queue_size', qos_profile_sensor_data.depth, qos_descriptor).value
        if self.pointcloud_reliability not in ('best_effort', 'reliable'):
            raise ValueError('pointcloud_reliability must be best_effort or reliable')
        if not 1 <= self.pointcloud_queue_size <= 1000:
            raise ValueError('pointcloud_queue_size must be in [1, 1000]')
        # Retain the sensor-data default for external best-effort publishers;
        # native simulation explicitly enables reliability across both hops.
        pointcloud_qos = QoSProfile(
            history=qos_profile_sensor_data.history, depth=self.pointcloud_queue_size,
            durability=qos_profile_sensor_data.durability,
            reliability=(ReliabilityPolicy.RELIABLE if self.pointcloud_reliability == 'reliable'
                         else ReliabilityPolicy.BEST_EFFORT))
        # Each native laser retains its own frame, topic and acquisition stamp.
        # An empty list preserves the existing single-sensor SMB/RMF contract.
        names = self.declare_parameter('sensor_names', ['legacy']).value
        if not names or len(set(names)) != len(names):
            raise ValueError('Expected unique, nonempty native sensor names')
        self.scan_sources = {}
        self.points_publishers = {}
        for name in names:
            frame = self.declare_parameter('sensor.' + name + '.frame', self.sensor).value
            raw = self.declare_parameter('sensor.' + name + '.input', 'gazebo/points_raw').value
            topic = self.declare_parameter('sensor.' + name + '.output', 'points').value
            minimum = self.declare_parameter('sensor.' + name + '.output_min_range', 0.).value
            maximum = self.declare_parameter('sensor.' + name + '.output_max_range', 0.).value
            if not frame or not raw or not topic:
                raise ValueError('Each native sensor needs a frame and two topics')
            if not math.isfinite(minimum + maximum) or minimum < 0 or maximum < 0 or (maximum and maximum <= minimum):
                raise ValueError('Invalid native lidar output range')
            self.scan_sources[name] = {'frame': frame, 'input': raw, 'output': topic,
                                       'output_min_range': minimum, 'output_max_range': maximum,
                                       'messages': 0, 'rejected': 0, 'range_filtered_points': 0,
                                       'last_stamp_ns': None}
            self.points_publishers[name] = self.create_publisher(PointCloud2, topic, pointcloud_qos)
            self.create_subscription(PointCloud2, raw,
                lambda message, source=name: self.points(message, source), pointcloud_qos)
        self.odom_count = self.scan_count = self.rejected = 0
        self.last_odom = self.last_scan_stamp = None
        self.last_status_at = -math.inf
        self.contacts_enabled = self.declare_parameter('contact_diagnostics', False).value
        self.model_kind = self.declare_parameter('model_kind', 'smb').value
        contact_prefix = self.declare_parameter('contact_prefix', '/smb/contact').value
        contact_parts = self.declare_parameter('contact_parts', ['base', 'lf', 'rf', 'lh', 'rh']).value
        self.lateral_slip_compliance = self.declare_parameter('lateral_slip_compliance', 0.).value
        self.contacts = {}
        if self.contacts_enabled:
            for part in contact_parts:
                self.contacts[part] = {'messages': 0, 'contact_pairs': 0, 'contact_points': 0,
                                       'max_depth_m': None, 'depth_samples': 0,
                                       'normal_samples': 0, 'wrench_samples': 0,
                                       'last_stamp_ns': None, 'last_pairs': []}
                self.create_subscription(Contacts, contact_prefix.rstrip('/') + '/' + part,
                    lambda message, name=part: self.contact(message, name), qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)
        self.status_pub = self.create_publisher(String, 'gazebo_state', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, 'gazebo/odometry_raw', self.odometry, 10)
        self.create_service(Trigger, 'gazebo_state', self.state)

    @staticmethod
    def stamp_ns(header):
        return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

    def payload(self):
        value = {'physics': 'Gazebo Fortress', 'odometry_source': 'native OdometryPublisher',
                 'lidar_source': 'native gpu_lidar', 'odometry_messages': self.odom_count,
                 'pointcloud_messages': self.scan_count, 'rejected': self.rejected,
                 'pointcloud_reliability': self.pointcloud_reliability,
                 'pointcloud_queue_size': self.pointcloud_queue_size,
                 'last_scan_stamp_ns': self.last_scan_stamp,
                 'sensor_sources': self.scan_sources,
                 'pose_semantics': 'world model pose; zero plugin offset; base_link is model origin',
                 'linear_twist_semantics': 'native body-frame velocity',
                 'angular_twist_semantics': 'native filtered Euler rates converted to body angular velocity'}
        value['model_kind'] = self.model_kind
        if self.model_kind == 'smb':
            value['wheel_contact_model'] = {
                'lateral_slip_compliance': self.lateral_slip_compliance,
                'longitudinal_slip_compliance': 0.,
                'profile': 'native_finite_slip' if self.lateral_slip_compliance > 0 else 'upstream_rigid',
                'calibration': 'Nominal simulation model; no real-tire calibration'}
        value['contact_diagnostics'] = {
            'enabled': self.contacts_enabled, 'parts': self.contacts,
            'semantics': 'Observed nonempty contact messages; silence alone is not proof of no contact'}
        if self.last_odom:
            m = self.last_odom
            p, q = m.pose.pose.position, m.pose.pose.orientation
            value.update(position=[p.x, p.y, p.z], quaternion=[q.x, q.y, q.z, q.w],
                         source_stamp_ns=self.stamp_ns(m.header))
        return value

    def state(self, request, response):
        response.success = True
        response.message = json.dumps(self.payload())
        return response

    def contact(self, message, part):
        item = self.contacts[part]
        item['messages'] += 1
        item['last_stamp_ns'] = self.stamp_ns(message.header)
        item['contact_pairs'] += len(message.contacts)
        item['contact_points'] += sum(len(c.positions) for c in message.contacts)
        item['last_pairs'] = [[c.collision1.name, c.collision2.name] for c in message.contacts]
        depths = [depth for contact in message.contacts for depth in contact.depths if math.isfinite(depth)]
        item['normal_samples'] += sum(len(c.normals) for c in message.contacts)
        item['wrench_samples'] += sum(len(c.wrenches) for c in message.contacts)
        item['depth_samples'] += len(depths)
        if depths:
            item['max_depth_m'] = max(max(depths), item['max_depth_m']) if item['max_depth_m'] is not None else max(depths)

    def odometry(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        if (self.stamp_ns(message.header) <= 0 or
                message.header.frame_id != self.frame or message.child_frame_id != self.base or
                not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))):
            self.rejected += 1
            return
        if self.last_odom and self.stamp_ns(message.header) <= self.stamp_ns(self.last_odom.header):
            return
        output = copy.deepcopy(message)
        output.header.frame_id, output.child_frame_id = self.frame, self.base
        # Fortress 6.18.0 OdometryPublisher uses worldPose(model) and rotates
        # linear differences into the body frame. Its angular vector contains
        # filtered RPY derivatives, however, not body angular velocity. Apply
        # the standard ZYX rate transform before exposing nav_msgs/Odometry.
        # This retains the native finite-difference/filter approximation.
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
        rates = message.twist.twist.angular
        output.twist.twist.angular.x = rates.x - rates.z*math.sin(pitch)
        output.twist.twist.angular.y = rates.y*math.cos(roll) + rates.z*math.sin(roll)*math.cos(pitch)
        output.twist.twist.angular.z = -rates.y*math.sin(roll) + rates.z*math.cos(roll)*math.cos(pitch)
        self.odom_pub.publish(output)
        transform = TransformStamped()
        transform.header = output.header
        transform.child_frame_id = self.base
        transform.transform.translation.x = p.x
        transform.transform.translation.y = p.y
        transform.transform.translation.z = p.z
        transform.transform.rotation = q
        self.tf.sendTransform(transform)
        self.last_odom = output
        self.odom_count += 1
        now = self.stamp_ns(output.header) * 1e-9
        if now - self.last_status_at >= .2:
            self.status_pub.publish(String(data=json.dumps(self.payload())))
            self.last_status_at = now

    def points(self, message, source='legacy'):
        item = self.scan_sources[source]
        if self.stamp_ns(message.header) <= 0 or not message.width or not message.data:
            self.rejected += 1
            item['rejected'] += 1
            return
        # Ignition may scope sensor frame names by model/link. The model's
        # fixed sensor transform defines this same optical frame in ROS.
        message.header.frame_id = item['frame']
        minimum, maximum = item['output_min_range'], item['output_max_range']
        if maximum:
            # Gazebo Classic's lidar plugin accepted an open range interval.
            # Keep the native organized layout and all non-XYZ fields; an
            # invalid return becomes NaN, never a fictitious far obstacle.
            # Classic also gated before adding plugin noise. Fortress noise
            # already happened in the sensor, so only its final gate is shared.
            fields = {field.name: field for field in message.fields}
            xyz = [fields[name] for name in ('x', 'y', 'z')]
            if any(field.datatype != 7 or field.count != 1 for field in xyz):
                raise ValueError('Native lidar must publish scalar FLOAT32 XYZ')
            reader = struct.Struct('>f' if message.is_bigendian else '<f')
            data = None
            filtered = 0
            for row in range(message.height):
                for column in range(message.width):
                    start = row * message.row_step + column * message.point_step
                    values = [reader.unpack_from(message.data, start + field.offset)[0] for field in xyz]
                    if not all(math.isfinite(value) for value in values):
                        continue
                    distance = math.sqrt(sum(value * value for value in values))
                    if minimum < distance < maximum:
                        continue
                    if data is None:
                        data = bytearray(message.data)
                    for field in xyz:
                        reader.pack_into(data, start + field.offset, math.nan)
                    filtered += 1
            if data is not None:
                message.data = data
                message.is_dense = False
            item['range_filtered_points'] += filtered
        self.points_publishers[source].publish(message)
        self.scan_count += 1
        self.last_scan_stamp = self.stamp_ns(message.header)
        item['messages'] += 1
        item['last_stamp_ns'] = self.last_scan_stamp


def main():
    # Stop dispatch before destroying the context. rclpy's default asynchronous
    # SIGINT handler can shut it down while take_message converts a high-rate
    # native Contacts message, causing a pybind RuntimeError during teardown.
    # The handler below only requests stop; active callbacks finish normally.
    # Runtime conversion errors remain visible and are never broadly ignored.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = FortressState()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while not stop_requested and rclpy.ok():
            executor.spin_once(timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
