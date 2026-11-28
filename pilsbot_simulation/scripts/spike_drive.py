#!/usr/bin/env python3
"""SPIKE (pilsbot_simulation#4): scripted drive. Publishes AckermannDriveStamped
on /pilsbot_velocity_controller/cmd_vel for <seconds> of sim time with <speed>
m/s and <steering> rad, then zero; prints /odom at start and end.

Usage: spike_drive.py <seconds> <speed> <steering> --ros-args -p use_sim_time:=true
"""
import sys

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def main():
    secs, speed, steer = (float(v) for v in sys.argv[1:4])
    rclpy.init(args=sys.argv)
    n = Node('spike_drive')
    pub = n.create_publisher(AckermannDriveStamped, '/pilsbot_velocity_controller/cmd_vel', 10)
    odom = []
    n.create_subscription(Odometry, '/odom', lambda m: odom.append(m), 10)
    while rclpy.ok() and (not odom or n.get_clock().now().nanoseconds == 0):
        rclpy.spin_once(n, timeout_sec=0.1)
    t0 = n.get_clock().now()
    p0 = odom[-1].pose.pose.position
    print(f'start sim t={t0.nanoseconds / 1e9:.2f} odom=({p0.x:.3f}, {p0.y:.3f})', flush=True)
    m = AckermannDriveStamped()
    m.drive.speed = speed
    m.drive.steering_angle = steer
    m.drive.steering_angle_velocity = 1.0
    last = None
    while rclpy.ok() and (n.get_clock().now() - t0).nanoseconds / 1e9 < secs:
        now = n.get_clock().now()
        if last is None or (now - last).nanoseconds > 1e8:
            m.header.stamp = now.to_msg()
            pub.publish(m)
            last = now
        rclpy.spin_once(n, timeout_sec=0.02)
    m.drive.speed = 0.0
    for _ in range(20):
        m.header.stamp = n.get_clock().now().to_msg()
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.05)
    p1 = odom[-1].pose.pose.position
    t1 = n.get_clock().now()
    print(f'end sim t={t1.nanoseconds / 1e9:.2f} odom=({p1.x:.3f}, {p1.y:.3f}) '
          f'moved {((p1.x - p0.x) ** 2 + (p1.y - p0.y) ** 2) ** 0.5:.3f} m', flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
