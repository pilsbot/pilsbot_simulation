#!/usr/bin/env python3
"""SPIKE: save one image of each given sensor_msgs/Image topic as PNG.

Usage: spike_save_image.py <out_dir> <prefix> <topic>...
Depth (32FC1) is scaled 0..20 m to 0..255.
"""
import os
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def to_cv(m):
    a = np.frombuffer(bytes(m.data), dtype=np.float32 if m.encoding == '32FC1' else np.uint8)
    if m.encoding == '32FC1':
        a = a.reshape(m.height, m.width)
        a = np.nan_to_num(a, nan=0.0, posinf=20.0)
        return (np.clip(a / 20.0, 0, 1) * 255).astype(np.uint8)
    ch = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}[m.encoding]
    a = a.reshape(m.height, m.step)[:, :m.width * ch].reshape(m.height, m.width, ch)
    return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a


def main():
    out, prefix, topics = sys.argv[1], sys.argv[2], [t for t in sys.argv[3:] if not t.startswith('--')]
    rclpy.init()
    n = Node('spike_save_image')
    got = {}
    for t in topics:
        n.create_subscription(Image, t, lambda m, t=t: got.setdefault(t, m), 2)
    while rclpy.ok() and len(got) < len(topics):
        rclpy.spin_once(n, timeout_sec=0.5)
    for t, m in got.items():
        f = os.path.join(out, prefix + t.strip('/').replace('/', '_') + '.png')
        cv2.imwrite(f, to_cv(m))
        print(f, m.encoding, m.width, m.height, m.header.frame_id, m.header.stamp.sec)


if __name__ == '__main__':
    main()
