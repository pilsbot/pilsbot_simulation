#!/usr/bin/env python3
"""SPIKE pilsbot/pilsbot_simulation#1: drive scenarios, steering and odom vs ground truth.

Publishes AckermannDriveStamped on /pilsbot_velocity_controller/cmd_vel (the
gamepad's authority 1.0), records steering_axle_joint from /joint_states, /odom
and the model pose from /ground_truth/poses, and prints per phase:
commanded vs measured steering angle (mean of the last 2 s) and odom pose vs
ground truth pose (ground truth expressed relative to its pose at the start).
"""

import math
import sys

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Eval(Node):
    def __init__(self, phases, authority):
        super().__init__('spike_drive_eval')
        self.phases = phases
        self.authority = authority
        self.pub = self.create_publisher(
            AckermannDriveStamped, '/pilsbot_velocity_controller/cmd_vel', 10)
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(TFMessage, '/ground_truth/poses', self.on_gt, 10)
        self.steer = None
        self.odom = None
        self.gt = None
        self.gt0 = None
        self.odom0 = None
        self.samples = []
        self.last_gt = None
        self.path = 0.0
        self.turned = 0.0
        self.phase = -1
        self.phase_start = None
        self.timer = self.create_timer(0.05, self.tick)
        self.done = False

    def on_js(self, m):
        if 'steering_axle_joint' in m.name:
            self.steer = m.position[m.name.index('steering_axle_joint')]
        self.wheels = {n: (m.velocity[i] if i < len(m.velocity) else float('nan'))
                       for i, n in enumerate(m.name) if 'wheel' in n}

    def on_odom(self, m):
        p = m.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_gt(self, m):
        for t in m.transforms:
            if t.child_frame_id == 'pilsbot':
                tr = t.transform
                self.gt = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))

    def gt_rel(self):
        x0, y0, th0 = self.gt0
        dx, dy = self.gt[0] - x0, self.gt[1] - y0
        c, s = math.cos(-th0), math.sin(-th0)
        return (c * dx - s * dy, s * dx + c * dy, wrap(self.gt[2] - th0))

    def odom_rel(self):
        x0, y0, th0 = self.odom0
        dx, dy = self.odom[0] - x0, self.odom[1] - y0
        c, s = math.cos(-th0), math.sin(-th0)
        return (c * dx - s * dy, s * dx + c * dy, wrap(self.odom[2] - th0))

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.steer is None or self.odom is None or self.gt is None:
            return
        if self.phase < 0:
            self.phase, self.phase_start = 0, now
            self.gt0, self.odom0 = self.gt, self.odom
            print(f'start: odom={fmt(self.odom)} gt={fmt(self.gt)}', flush=True)
        name, speed, angle, duration = self.phases[self.phase]
        t = now - self.phase_start
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        msg.drive.steering_angle_velocity = self.authority
        self.pub.publish(msg)
        self.samples.append((t, self.steer))
        if self.last_gt is not None:
            self.path += math.hypot(self.gt[0] - self.last_gt[0], self.gt[1] - self.last_gt[1])
            self.turned += wrap(self.gt[2] - self.last_gt[2])
        self.last_gt = self.gt
        if VERBOSE and int(t * 20) % 10 == 0:
            w = self.wheels
            print(f'  t={t:5.2f} steer={self.steer:+.3f} '
                  f'FL={w.get("front_wheel_left_joint", 0):+.2f} FR={w.get("front_wheel_right_joint", 0):+.2f} '
                  f'RL={w.get("rear_wheel_left_joint", 0):+.2f} RR={w.get("rear_wheel_right_joint", 0):+.2f} '
                  f'gt={fmt(self.gt_rel())} odom={fmt(self.odom_rel())}', flush=True)
        if t >= duration:
            tail = [s for (ts, s) in self.samples if ts >= duration - 2.0]
            mean = sum(tail) / len(tail)
            o, g = self.odom_rel(), self.gt_rel()
            err = math.hypot(o[0] - g[0], o[1] - g[1])
            dist = math.hypot(g[0], g[1])
            print(f'{name:14s} v={speed:+.2f} cmd_angle={angle:+.3f} '
                  f'steer(last2s)={mean:+.3f} [{min(tail):+.3f},{max(tail):+.3f}] | '
                  f'odom={fmt(o)} gt={fmt(g)} pos_err={err:.3f} m '
                  f'yaw_err={math.degrees(wrap(o[2] - g[2])):+.1f} deg '
                  f'(gt dist from start {dist:.2f} m) | phase gt path {self.path:.2f} m '
                  f'turned {math.degrees(self.turned):+.0f} deg', flush=True)
            self.path = 0.0
            self.turned = 0.0
            self.samples = []
            self.phase += 1
            self.phase_start = now
            if self.phase >= len(self.phases):
                self.done = True


VERBOSE = False


def fmt(p):
    return f'({p[0]:+.3f}, {p[1]:+.3f}, {math.degrees(p[2]):+.1f}deg)'


def main():
    rclpy.init(args=sys.argv)
    global VERBOSE
    scenario = sys.argv[1] if len(sys.argv) > 1 else 'all'
    VERBOSE = len(sys.argv) > 2 and sys.argv[2] == 'v'
    v = 0.4
    scenarios = {
        'steer': [('settle', 0.0, 0.0, 3.0),
                  ('steer+0.3', 0.0, 0.3, 6.0), ('steer-0.3', 0.0, -0.3, 6.0),
                  ('steer+0.8', 0.0, 0.8, 6.0), ('steer0', 0.0, 0.0, 6.0)],
        'straight': [('settle', 0.0, 0.0, 3.0), ('straight', v, 0.0, 15.0),
                     ('stop', 0.0, 0.0, 4.0)],
        'right': [('settle', 0.0, 0.0, 2.0), ('circle+0.4', v, 0.4, 12.0), ('stop', 0.0, 0.0, 3.0)],
        'sharp': [('settle', 0.0, 0.0, 2.0), ('sharp+1.0', v, 1.0, 15.0),
                  ('sharp-1.0', v, -1.0, 15.0), ('slalom+0.6', v, 0.6, 5.0),
                  ('slalom-0.6', v, -0.6, 5.0), ('stop', 0.0, 0.0, 4.0)],
        'sharpss': [('settle', 0.0, 0.0, 2.0), ('pre+1.0', 0.0, 1.0, 6.0),
                    ('ramp+1.0', v, 1.0, 4.0), ('ss1+1.0', v, 1.0, 8.0), ('ss2+1.0', v, 1.0, 8.0),
                    ('stop', 0.0, 1.0, 4.0), ('pre-1.0', 0.0, -1.0, 8.0),
                    ('ramp-1.0', v, -1.0, 4.0), ('ss1-1.0', v, -1.0, 8.0), ('ss2-1.0', v, -1.0, 8.0),
                    ('stop', 0.0, 0.0, 4.0)],
        'circles': [('settle', 0.0, 0.0, 3.0),
                    ('circle+0.4', v, 0.4, 30.0), ('stop', 0.0, 0.0, 4.0),
                    ('circle-0.4', v, -0.4, 30.0), ('stop', 0.0, 0.0, 4.0)],
    }
    phases = []
    for key in (['steer', 'straight', 'circles'] if scenario == 'all' else scenario.split(',')):
        phases += scenarios[key]
    node = Eval(phases, authority=1.0)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    o, g = node.odom_rel(), node.gt_rel()
    print(f'END odom={fmt(o)} gt={fmt(g)} pos_err={math.hypot(o[0]-g[0], o[1]-g[1]):.3f} m',
          flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
