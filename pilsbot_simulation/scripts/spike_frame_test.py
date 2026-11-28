#!/usr/bin/env python3
"""SPIKE (pilsbot_simulation#4): frame test of a point cloud against worlds/frame_test.sdf.

Usage: spike_frame_test.py <cloud topic> [n_clouds] [frame_override] --ros-args -p use_sim_time:=true

Takes n clouds, transforms every point into base_footprint with tf2 (the
transform at the cloud's stamp), and compares with the box of
worlds/frame_test.sdf: front face x = 3.0, y in [-0.5, 1.5], z in [0, 1]
(the robot is spawned at the origin and does not move, so base_footprint
coincides with the world frame).

Prints: frame_id; the transform; for points above the ground (z > 0.05)
and nearer than 8 m: count, percentiles of x, y, z; the share within 5 cm
of the face plane inside the face's y/z extent; a mirror/rotation verdict;
and the ground points' z (pitch/roll check).
"""
import sys

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros

FACE_X, Y0, Y1, Z0, Z1 = 3.0, -0.5, 1.5, 0.0, 1.0


def quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class FrameTest(Node):
    def __init__(self, topic, n):
        super().__init__('spike_frame_test')
        self.n = n
        self.clouds = []
        self.buf = tf2_ros.Buffer(cache_time=Duration(seconds=30))
        self.tfl = tf2_ros.TransformListener(self.buf, self)
        self.sub = self.create_subscription(PointCloud2, topic, self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        if len(self.clouds) < self.n:
            self.clouds.append(msg)


def main():
    topic = sys.argv[1]
    n = int(sys.argv[2]) if len(sys.argv) > 2 and not sys.argv[2].startswith('--') else 3
    override = sys.argv[3] if len(sys.argv) > 3 and not sys.argv[3].startswith('--') else None
    rclpy.init(args=sys.argv)
    node = FrameTest(topic, n)
    deadline = node.get_clock().now() + Duration(seconds=600)
    while rclpy.ok() and len(node.clouds) < n:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.get_clock().now() > deadline:
            print('timeout waiting for clouds')
            return 1
    # let tf catch up
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
    allp = []
    for msg in node.clouds:
        frame = override or msg.header.frame_id
        if override:
            print(f'header frame_id {msg.header.frame_id} IGNORED, using {override}')
        try:
            t = node.buf.lookup_transform('base_footprint', frame, Time.from_msg(msg.header.stamp),
                                          timeout=Duration(seconds=5))
        except Exception as e:  # noqa: BLE001
            print('tf failed:', e)
            continue
        R = quat_to_mat(t.transform.rotation)
        tr = t.transform.translation
        T = np.array([tr.x, tr.y, tr.z])
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = pts[np.isfinite(pts).all(axis=1)]
        allp.append(pts @ R.T + T)
        print(f'cloud {msg.width}x{msg.height} frame_id={frame} stamp={msg.header.stamp.sec}.'
              f'{msg.header.stamp.nanosec:09d} valid={len(pts)}')
    print('base_footprint <- %s: t = (%.3f %.3f %.3f)' % (frame, *T))
    print('R =\n', np.round(R, 3))
    p = np.vstack(allp)
    above = p[(p[:, 2] > 0.05) & (np.linalg.norm(p[:, :2], axis=1) < 8.0)]
    ground = p[(np.abs(p[:, 2]) < 0.3) & (p[:, 0] < FACE_X - 0.3)]
    print(f'points: {len(p)}, above ground (z>0.05, <8 m): {len(above)}')
    if len(above):
        for i, ax in enumerate('xyz'):
            q = np.percentile(above[:, i], [1, 5, 50, 95, 99])
            print(f'  {ax}: p1 {q[0]:+.3f} p5 {q[1]:+.3f} median {q[2]:+.3f} p95 {q[3]:+.3f} p99 {q[4]:+.3f}')
        face = above[(above[:, 1] > Y0 - 0.05) & (above[:, 1] < Y1 + 0.05) &
                     (above[:, 2] < Z1 + 0.05)]
        on = np.abs(face[:, 0] - FACE_X) < 0.05
        print(f'  in face y/z extent: {len(face)}; within 5 cm of x = {FACE_X}: '
              f'{on.sum()} ({100.0 * on.mean() if len(face) else 0:.1f} %)')
        if len(face):
            err = face[:, 0] - FACE_X
            print(f'  x error on face: mean {err.mean():+.4f} m, median {np.median(err):+.4f}, '
                  f'std {err.std():.4f}, |err| p95 {np.percentile(np.abs(err), 95):.4f}')
            f = face[on]
            if len(f):
                print(f'  face points extent: y [{f[:, 1].min():+.3f}, {f[:, 1].max():+.3f}] '
                      f'(box {Y0:+.1f}..{Y1:+.1f}), z [{f[:, 2].min():+.3f}, {f[:, 2].max():+.3f}] '
                      f'(box {Z0:.1f}..{Z1:.1f})')
        mirrored = np.median(above[:, 1]) < 0
        print(f'  verdict: median y {np.median(above[:, 1]):+.3f} -> '
              f'{"MIRRORED (box is at +y)" if mirrored else "box on the +y side as in the world"}')
    if len(ground):
        print(f'ground points (x < {FACE_X - 0.3}): {len(ground)}, z median {np.median(ground[:, 2]):+.4f} '
              f'p5 {np.percentile(ground[:, 2], 5):+.4f} p95 {np.percentile(ground[:, 2], 95):+.4f}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
