#!/usr/bin/env python3
"""Adapt the upstream pose topics into the planner's existing odometry topic.

Only real, time-stamped input samples are published. Frame conversion uses TF
at each sample's acquisition time; this node never substitutes the current
time or synthesizes odometry while the source is silent.
"""
import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion(value):
    result = (float(value.x), float(value.y), float(value.z), float(value.w))
    norm = math.hypot(*result)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('invalid_quaternion')
    return tuple(component / norm for component in result)


def multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rotate(q, point):
    vector = multiply(multiply(q, (point[0], point[1], point[2], 0.)),
                      (-q[0], -q[1], -q[2], q[3]))
    return vector[:3]


def rotation_matrix(q):
    x, y, z, w = q
    return ((1. - 2. * (y*y + z*z), 2. * (x*y - z*w), 2. * (x*z + y*w)),
            (2. * (x*y + z*w), 1. - 2. * (x*x + z*z), 2. * (y*z - x*w)),
            (2. * (x*z - y*w), 2. * (y*z + x*w), 1. - 2. * (x*x + y*y)))


def transform_covariance(covariance, rotation):
    """Rotate both pose error blocks and their cross covariance into the target frame."""
    if len(covariance) != 36 or not all(math.isfinite(value) for value in covariance):
        raise ValueError('invalid_pose_covariance')
    jacobian = [[0.] * 6 for _ in range(6)]
    for block in (0, 3):
        for row in range(3):
            for column in range(3):
                jacobian[block + row][block + column] = rotation[row][column]
    matrix = [covariance[row*6:(row+1)*6] for row in range(6)]
    left = [[sum(jacobian[i][k] * matrix[k][j] for k in range(6))
             for j in range(6)] for i in range(6)]
    return [sum(left[i][k] * jacobian[j][k] for k in range(6))
            for i in range(6) for j in range(6)]


class StateInput(Node):
    def __init__(self):
        super().__init__('gbplanner_state_input')
        self.source = str(self.declare_parameter('mode', 'pose').value)
        self.input_topic = str(self.declare_parameter('input_topic', '').value)
        self.output_topic = str(self.declare_parameter('output_topic', 'odometry').value)
        self.frame_id = str(self.declare_parameter('frame_id', 'map').value)
        self.body_frame = str(self.declare_parameter('body_frame', 'base_link').value)
        self.transform_enabled = bool(self.declare_parameter('transform_to_frame', True).value)
        self.tf_timeout = float(self.declare_parameter('tf_timeout_sec', 0.2).value)
        if self.source not in ('pose', 'pose_stamped'):
            raise ValueError('mode must be pose or pose_stamped; odom mode needs no adapter')
        if not self.input_topic:
            self.input_topic = self.source
        if not self.input_topic or not self.output_topic or not self.frame_id or not self.body_frame:
            raise ValueError('input_topic, output_topic, frame_id and body_frame must be nonempty')
        if not math.isfinite(self.tf_timeout) or self.tf_timeout <= 0.:
            raise ValueError('tf_timeout_sec must be positive and finite')

        self.received = 0
        self.published = 0
        self.rejected = 0
        self.transformed = 0
        self.last_error = ''
        self.last_input_stamp = None
        self.last_input_frame = ''
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        message_type = PoseWithCovarianceStamped if self.source == 'pose' else PoseStamped
        self.subscription = self.create_subscription(message_type, self.input_topic, self.receive,
                                                     qos_profile_sensor_data)
        self.status_service = self.create_service(Trigger, 'state_input_status', self.status)
        self.get_logger().info(
            f'{self.source} {self.input_topic} -> {self.output_topic}; '
            f'frame={self.frame_id}, body_frame={self.body_frame}, '
            f'TF conversion={self.transform_enabled}')

    @staticmethod
    def stamp_seconds(stamp):
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000 or not math.isfinite(value) or value <= 0.:
            raise ValueError('invalid_or_zero_sample_stamp')
        return value

    @staticmethod
    def set_quaternion(destination, value):
        destination.x, destination.y, destination.z, destination.w = value

    def receive(self, message):
        self.received += 1
        try:
            source_pose = message.pose.pose if self.source == 'pose' else message.pose
            # PoseStamped has no uncertainty or velocity observation. Large
            # diagonal variances mark those unmeasured values; zero twist is
            # only the message default, not an assertion that the robot rests.
            unmeasured = [1e6 if i % 7 == 0 else 0. for i in range(36)]
            covariance = list(message.pose.covariance) if self.source == 'pose' else unmeasured[:]
            source_frame = message.header.frame_id
            stamp = message.header.stamp
            self.stamp_seconds(stamp)
            if not source_frame:
                raise ValueError('empty_source_frame')
            position = (float(source_pose.position.x), float(source_pose.position.y),
                        float(source_pose.position.z))
            if not all(math.isfinite(value) for value in position):
                raise ValueError('invalid_position')
            orientation = quaternion(source_pose.orientation)
            if len(covariance) != 36 or not all(math.isfinite(value) for value in covariance):
                raise ValueError('invalid_pose_covariance')

            transform = None
            if source_frame != self.frame_id:
                if not self.transform_enabled:
                    raise ValueError('source_frame_differs_from_configured_frame')
                transform = self.buffer.lookup_transform(
                    self.frame_id, source_frame, Time.from_msg(stamp),
                    timeout=Duration(seconds=self.tf_timeout))

            if transform is not None:
                translation = transform.transform.translation
                tf_position = (float(translation.x), float(translation.y), float(translation.z))
                tf_orientation = quaternion(transform.transform.rotation)
                if not all(math.isfinite(value) for value in tf_position):
                    raise ValueError('invalid_transform_translation')
                rotated = rotate(tf_orientation, position)
                position = tuple(rotated[i] + tf_position[i] for i in range(3))
                orientation = quaternion_value(multiply(tf_orientation, orientation))
                covariance = transform_covariance(covariance, rotation_matrix(tf_orientation))

            if not all(math.isfinite(value) for value in (*position, *covariance)):
                raise ValueError('nonfinite_transformed_pose')

            odometry = Odometry()
            odometry.header.frame_id = self.frame_id
            odometry.header.stamp = stamp
            odometry.child_frame_id = self.body_frame
            odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z = position
            self.set_quaternion(odometry.pose.pose.orientation, orientation)
            odometry.pose.covariance = covariance
            odometry.twist.covariance = unmeasured
            self.publisher.publish(odometry)
            self.published += 1
            self.transformed += int(transform is not None)
            self.last_input_stamp = {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)}
            self.last_input_frame = source_frame
            self.last_error = ''
        except (ValueError, TransformException, OverflowError) as error:
            self.rejected += 1
            self.last_error = str(error) or type(error).__name__
            self.get_logger().warning(f'Rejected {self.source} sample: {self.last_error}')

    def status(self, _request, response):
        response.success = True
        response.message = json.dumps({
            'mode': self.source, 'input_topic': self.input_topic,
            'output_topic': self.output_topic, 'frame_id': self.frame_id,
            'body_frame': self.body_frame, 'transform_to_frame': self.transform_enabled,
            'received': self.received, 'published': self.published,
            'rejected': self.rejected, 'transformed': self.transformed,
            'last_input_frame': self.last_input_frame,
            'last_input_stamp': self.last_input_stamp, 'last_error': self.last_error,
        }, separators=(',', ':'))
        return response


def quaternion_value(value):
    norm = math.hypot(*value)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('invalid_transformed_quaternion')
    return tuple(component / norm for component in value)


def main():
    rclpy.init()
    node, executor = None, None
    try:
        node = StateInput()
        # The TF listener has its own callback group. A second thread lets it
        # receive a just-arriving transform while a sample waits for that TF.
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
