#!/usr/bin/env python3
"""ROS 2 port of SMB Joy mapping; provenance: teleop_SOURCE.json.

Corrections to upstream: check message indices, toggle only on button edges,
stop refreshing stale Joy commands, and release manual ownership at neutral.
"""
import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Empty


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('gbplanner_joy_teleop')
        self.axis_linear = int(self.declare_parameter('axis_linear', 3).value)
        self.axis_angular = int(self.declare_parameter('axis_angular', 2).value)
        self.button_stop = int(self.declare_parameter('button_stop', 5).value)
        self.button_lock = int(self.declare_parameter('button_lock', 4).value)
        self.max_v = float(self.declare_parameter('max_velocity_linear', 1.0).value)
        self.max_w = float(self.declare_parameter('max_velocity_angular', 1.5).value)
        self.timeout = float(self.declare_parameter('joy_timeout', 0.5).value)
        period = float(self.declare_parameter('publish_period', 0.1).value)
        self.deadzone = float(self.declare_parameter('deadzone', 0.05).value)
        if min(self.axis_linear, self.axis_angular, self.button_stop, self.button_lock) < 0:
            raise ValueError('Joy indices must be nonnegative')
        if any(not math.isfinite(value) or value <= 0 for value in (self.max_v, self.max_w, self.timeout, period)):
            raise ValueError('Joy scales and timing must be finite and positive')
        if not math.isfinite(self.deadzone) or not 0 <= self.deadzone < 1:
            raise ValueError('Joy deadzone must be in [0,1)')
        self.command = (0.0, 0.0)
        self.active = False
        self.locked = False
        self.last_received = 0.0
        self.previous_stop = self.previous_lock = False
        self.commands = self.create_publisher(Twist, 'joy_teleop/cmd_vel', 1)
        self.locks = self.create_publisher(Bool, 'cmd_vel_e_stop', 10)
        self.soft_stops = self.create_publisher(Empty, 'soft_emergency_stop', 10)
        self.create_subscription(Joy, 'joy', self.receive, 1)
        self.create_subscription(Bool, 'cmd_vel_e_stop', self.receive_lock, 10)
        self.create_subscription(Empty, 'soft_emergency_stop', self.receive_soft_stop, 10)
        self.create_timer(period, self.tick)

    def publish(self):
        message = Twist()
        message.linear.x, message.angular.z = self.command
        self.commands.publish(message)

    def stop_output(self):
        was_active = self.active
        self.command, self.active = (0.0, 0.0), False
        if was_active:
            self.publish()

    def receive_lock(self, message):
        self.locked = bool(message.data)
        if self.locked:
            self.stop_output()

    def receive_soft_stop(self, _message):
        self.locked = True
        self.stop_output()

    def set_lock(self, locked):
        self.locked = locked
        self.stop_output()
        message = Bool()
        message.data = locked
        self.locks.publish(message)

    def receive(self, message):
        if (len(message.axes) <= max(self.axis_linear, self.axis_angular) or
                len(message.buttons) <= max(self.button_stop, self.button_lock)):
            self.stop_output()
            return
        axes = (float(message.axes[self.axis_linear]), float(message.axes[self.axis_angular]))
        if not all(math.isfinite(value) for value in axes):
            self.stop_output()
            return
        # A zero stamp remains compatible with ordinary Joy publishers. A
        # provided source timestamp must describe a current input sample.
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if stamp > 0 and (age < -0.1 or age > self.timeout):
            self.stop_output()
            return
        stop, toggle = bool(message.buttons[self.button_stop]), bool(message.buttons[self.button_lock])
        stop_edge, toggle_edge = stop and not self.previous_stop, toggle and not self.previous_lock
        self.previous_stop, self.previous_lock = stop, toggle
        self.last_received = time.monotonic()
        if stop:
            if stop_edge:
                self.soft_stops.publish(Empty())
                self.set_lock(True)
            self.stop_output()
            return
        if toggle:
            if toggle_edge:
                self.set_lock(not self.locked)
            return
        if self.locked:
            self.stop_output()
            return
        linear, angular = (0.0 if abs(value) <= self.deadzone else max(-1.0, min(1.0, value)) for value in axes)
        command = (linear * self.max_v, angular * self.max_w)
        if command == (0.0, 0.0):
            # Continually publishing neutral Joy would permanently outrank
            # autonomous control. Send one stop, then let the mux TTL expire.
            self.stop_output()
            return
        self.command, self.active = command, True
        self.publish()

    def tick(self):
        if self.active:
            if self.locked or time.monotonic() - self.last_received > self.timeout:
                self.stop_output()
            else:
                self.publish()


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A launch process may forward SIGINT after the process group already
        # received it. Finish destruction without a second interruption.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if rclpy.ok():
            node.stop_output()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
