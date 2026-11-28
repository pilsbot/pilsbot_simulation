#!/usr/bin/env python3
"""
Sim only: the OAK's camera_info with the stereo baseline that ROS expects.

Gazebo Fortress puts P[3] = -fx * 0.0375 (half the OAK-D baseline) into the
left, the right and the depth camera's camera_info
(pilsbot/pilsbot_simulation#4). stereo_image_proc takes the baseline from
the right camera's P[3] and computes half the depth.

config/bridge.yaml bridges those three to /gz/camera/...; this node
publishes them under the robot's names with
- left and depth (aligned to the left camera): P[3] = 0,
- right: P[3] = -fx * B, B the x offset of the right optical frame in the
  left one, from tf (the URDF: 0.075 m).
Everything else is unchanged, the stamps too, so exact-time sync of the
images still matches. The right camera_info is held back until tf has the
baseline.
"""
import copy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
import tf2_ros

# Gazebo's camera_info (bridge.yaml) -> the robot's name.
RELAYED = {
    '/gz/camera/left/camera_info': '/camera/left/camera_info',
    '/gz/camera/right/camera_info': '/camera/right/camera_info',
    '/gz/camera/stereo/camera_info': '/camera/stereo/camera_info',
}
RIGHT = '/gz/camera/right/camera_info'


def with_baseline(info, baseline):
    """Return a copy of info with P[3] = -fx * baseline [m]."""
    fixed = copy.deepcopy(info)
    fixed.p[3] = -fixed.p[0] * baseline
    return fixed


class CameraInfoRelay(Node):

    def __init__(self):
        super().__init__('camera_info_relay')
        self.left_frame = self.declare_parameter(
            'left_frame', 'oak_left_camera_optical_frame').value
        self.right_frame = self.declare_parameter(
            'right_frame', 'oak_right_camera_optical_frame').value
        self.baseline = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        for source, target in RELAYED.items():
            # Reliable, as parameter_bridge publishes; sensor-data subscribers
            # match it too.
            publisher = self.create_publisher(CameraInfo, target, 10)
            self.create_subscription(
                CameraInfo, source,
                lambda info, source=source, publisher=publisher:
                    self.relay(source, info, publisher),
                qos_profile_sensor_data)

    def relay(self, source, info, publisher):
        if source != RIGHT:
            publisher.publish(with_baseline(info, 0.0))
            return
        if self.baseline is None:
            self.baseline = self.lookup_baseline()
            if self.baseline is None:
                return
        publisher.publish(with_baseline(info, self.baseline))

    def lookup_baseline(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.left_frame, self.right_frame, Time())
        except tf2_ros.TransformException as error:
            self.get_logger().warn(
                f'no stereo baseline yet, holding back the right camera_info: {error}',
                throttle_duration_sec=10.0)
            return None
        baseline = transform.transform.translation.x
        self.get_logger().info(
            f'stereo baseline from tf: {baseline:.4f} m ({self.right_frame} in '
            f'{self.left_frame})')
        return baseline


def main():
    rclpy.init()
    node = CameraInfoRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
