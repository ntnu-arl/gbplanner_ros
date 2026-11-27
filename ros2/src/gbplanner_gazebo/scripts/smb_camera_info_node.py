#!/usr/bin/env python3
"""Remove Fortress's inferred stereo baseline from three independent cameras."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class SmbCameraInfo(Node):
    def __init__(self):
        super().__init__('smb_camera_info')
        self.outputs = {}
        for camera in ('rgb_cam_front', 'rgb_cam_left', 'rgb_cam_right'):
            self.outputs[camera] = self.create_publisher(CameraInfo, '/' + camera + '/camera_info', 10)
            self.create_subscription(CameraInfo, 'gazebo/' + camera + '_camera_info_raw',
                lambda message, name=camera: self.convert(message, name), 10)

    def convert(self, message, camera):
        # Fortress Sensors::CreateSensor assigns a stereo baseline to cameras
        # sharing a parent after URDF fixed-joint reduction. These front/side
        # cameras have different viewing axes and are independent monocular
        # streams. Keep their timestamp, optical frame and native calibration;
        # remove only the inferred stereo X translation. Raw Info is retained.
        message.p[3] = 0.
        self.outputs[camera].publish(message)


def main():
    rclpy.init()
    node = SmbCameraInfo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
