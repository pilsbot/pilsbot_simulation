#!/usr/bin/env python3
"""SPIKE (pilsbot_simulation#4): camera_info relay for stereo_image_proc.

Fortress (gz-sensors 6) publishes P[3] = -fx * 0.0375 on BOTH mono cameras
(half the OAK-D baseline, and also on the left one). stereo_image_proc reads
the baseline from the right camera's P[3] = -fx * B and expects the left's
P[3] = 0, so its depth comes out at half the distance.

This relays /camera/{left,right}/camera_info to
/camera/stereo_fixed/{left,right}/camera_info with left P[3] = 0 and right
P[3] = -fx * baseline. baseline: parameter 'baseline' [m]; if it is <= 0, the
distance between the two optical frames from tf (the URDF: 0.075 m).
Header stamps are kept, so exact-time sync still matches the images.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
import tf2_ros


class Relay(Node):
    def __init__(self):
        super().__init__('spike_stereo_info_relay')
        self.baseline = self.declare_parameter('baseline', 0.0).value
        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf, self)
        self.pubs = {s: self.create_publisher(CameraInfo, f'/camera/stereo_fixed/{s}/camera_info', 10)
                     for s in ('left', 'right')}
        for s in ('left', 'right'):
            self.create_subscription(CameraInfo, f'/camera/{s}/camera_info',
                                     lambda m, s=s: self.cb(s, m), qos_profile_sensor_data)
        self.logged = False

    def cb(self, side, m):
        if self.baseline <= 0.0:
            try:
                t = self.buf.lookup_transform('oak_left_camera_optical_frame',
                                              'oak_right_camera_optical_frame', rclpy.time.Time())
            except Exception:  # noqa: BLE001
                return
            v = t.transform.translation
            self.baseline = (v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5
            self.get_logger().info(f'baseline from tf: {self.baseline:.4f} m '
                                   f'(right in left optical: {v.x:+.4f} {v.y:+.4f} {v.z:+.4f})')
        p = list(m.p)
        old = p[3]
        p[3] = 0.0 if side == 'left' else -p[0] * self.baseline
        m.p = p
        if side == 'right' and not self.logged:
            self.get_logger().info(f'right P[3]: {old:.4f} -> {p[3]:.4f} (fx {p[0]:.3f})')
            self.logged = True
        self.pubs[side].publish(m)


def main():
    rclpy.init()
    rclpy.spin(Relay())


if __name__ == '__main__':
    main()
