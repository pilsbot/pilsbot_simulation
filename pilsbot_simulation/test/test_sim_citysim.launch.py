# Headless end-to-end test of world:=citysim_block: the robot spawns on the
# road at its default pose, the controllers come up, and it drives along the
# road on the street, without falling through it or leaving its lane.
#
# Opt-in: it runs only with PILSBOT_CITYSIM_TESTS=1. citysim's Fuel models
# must be in the Fuel cache (~/.ignition/fuel) or downloadable (about 110 MB,
# two minutes on first launch), and the world takes a while to load, so the
# default test run leaves it out. cameras:=false, so nothing renders. Also
# skipped like test_sim_drive.launch.py (sim_test_support.py); label
# "citysim": `colcon test --ctest-args -L citysim` runs it alone.
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import unittest

from ackermann_msgs.msg import AckermannDriveStamped
from controller_manager_msgs.srv import ListControllers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.parameter import Parameter

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_test_support import gazebo_unavailable, NothingLeftBehind, PARTITION, \
    partition_actions, sim_launch_file  # noqa: E402, I100


def citysim_unavailable():
    """Why the citysim test does not run, or None."""
    if os.environ.get('PILSBOT_CITYSIM_TESTS', '') in ('', '0'):
        return 'opt-in: set PILSBOT_CITYSIM_TESTS=1 (needs the citysim Fuel models)'
    return gazebo_unavailable()


CONTROLLERS = {'joint_state_publisher', 'pilsbot_velocity_controller'}
COMMAND_TOPIC = '/pilsbot_velocity_controller/cmd_vel'
# sim.launch.py's default spawn point in citysim: road_x_3, right lane,
# heading +x. Street level 5.01 m (citysim's README).
SPAWN_X, SPAWN_Y = -35.0, -1.8
STREET_LEVEL = 5.01
DISTANCE = 1.0

# Wall-clock limits: the world loads in about 20 s with a warm Fuel cache,
# minutes with a cold one; the sim runs well below real time.
ACTIVE_TIMEOUT = 600.0
MOTION_TIMEOUT = 300.0


@unittest.skipIf(citysim_unavailable() is not None, citysim_unavailable())
@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription(partition_actions() + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim_launch_file())),
            launch_arguments={'world': 'citysim_block', 'gui': 'false',
                              'foxglove': 'false', 'cameras': 'false'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def model_pose(model):
    """(x, y, z, yaw) of a model in the running world, from `ign model -p`."""
    output = subprocess.run(
        ['ign', 'model', '-m', model, '-p'], capture_output=True, text=True, timeout=30,
        env={**os.environ, 'IGN_PARTITION': PARTITION, 'GZ_PARTITION': PARTITION}).stdout
    # Pose [ XYZ (m) ] [ RPY (rad) ]:
    #  [x y z]
    #  [roll pitch yaw]
    vectors = re.findall(r'\[\s*(-?[\d.e+-]+)\s+(-?[\d.e+-]+)\s+(-?[\d.e+-]+)\s*\]', output)
    if len(vectors) < 2:
        raise AssertionError(f'no pose of {model} from ign model: {output!r}')
    (x, y, z), (_, _, yaw) = [[float(v) for v in vector] for vector in vectors[:2]]
    return x, y, z, yaw


class TestSimCitysim(unittest.TestCase):
    """Runs in name order: on the road, the controllers come up, then drive."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node(
            'sim_citysim_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
        cls.odom = None
        cls.node.create_subscription(Odometry, '/odom', cls.on_odom, 10)
        cls.command = cls.node.create_publisher(AckermannDriveStamped, COMMAND_TOPIC, 10)
        cls.list_controllers = cls.node.create_client(
            ListControllers, '/controller_manager/list_controllers')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def on_odom(cls, message):
        cls.odom = message

    def spin_until(self, condition, timeout, every=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if every is not None:
                every()
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if condition():
                return True
        return False

    def drive(self, speed):
        message = AckermannDriveStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.drive.speed = speed
        message.drive.steering_angle_velocity = 1.0
        self.command.publish(message)

    def active_controllers(self):
        if not self.list_controllers.wait_for_service(timeout_sec=1.0):
            return set()
        future = self.list_controllers.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        if future.result() is None:
            return set()
        return {c.name for c in future.result().controller if c.state == 'active'}

    def assert_on_the_road(self, pose, x_min, x_max):
        x, y, z, yaw = pose
        self.assertTrue(x_min <= x <= x_max, f'x {x:.3f} not in [{x_min}, {x_max}]')
        self.assertAlmostEqual(y, SPAWN_Y, delta=0.2, msg=f'left the lane: y {y:.3f}')
        self.assertAlmostEqual(z, STREET_LEVEL, delta=0.05, msg=f'not on the street: z {z:.3f}')
        self.assertAlmostEqual(yaw, 0.0, delta=0.1, msg=f'heading {yaw:.3f} rad')

    def test_1_both_controllers_become_active(self):
        active = set()

        def all_active():
            nonlocal active
            active = self.active_controllers()
            return CONTROLLERS <= active

        self.assertTrue(self.spin_until(all_active, ACTIVE_TIMEOUT),
                        f'active after {ACTIVE_TIMEOUT} s: {sorted(active)}')

    def test_2_the_robot_stands_on_the_road_at_the_spawn_point(self):
        self.assert_on_the_road(model_pose('pilsbot'), SPAWN_X - 0.1, SPAWN_X + 0.1)

    def test_3_it_drives_along_the_road(self):
        self.assertTrue(self.spin_until(lambda: self.odom is not None, 30.0), 'no /odom')
        start = self.odom.pose.pose.position

        def travelled():
            position = self.odom.pose.pose.position
            return math.hypot(position.x - start.x, position.y - start.y)

        try:
            reached = self.spin_until(lambda: travelled() > DISTANCE, MOTION_TIMEOUT,
                                      every=lambda: self.drive(0.5))
        finally:
            for _ in range(20):
                self.drive(0.0)
                rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(reached, f'odom travelled {travelled():.3f} m')
        self.assert_on_the_road(model_pose('pilsbot'),
                                SPAWN_X + 0.5 * DISTANCE, SPAWN_X + 2.0 * DISTANCE)


@launch_testing.post_shutdown_test()
class TestNothingLeftBehind(NothingLeftBehind, unittest.TestCase):
    pass
