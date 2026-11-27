# Headless end-to-end test: sim.launch.py gui:=false starts Gazebo Fortress
# with the robot stack, the spawners and the hardware activator bring both
# controllers up, and the robot drives and steers.
#
# Needs Gazebo Fortress (ign), ros_gz and gz_ros2_control and takes a few
# minutes (the cameras render, so the sim runs well below real time on
# integrated graphics). It is skipped where Gazebo is missing, or with
# PILSBOT_SKIP_GAZEBO_TESTS=1; `colcon test --ctest-args -LE gazebo` leaves it
# out as well.
#
# The controller's odometry starts at the front pivot (pils_control), not at
# base_footprint's spawn pose, so motion is checked relative to the first odom
# message. Steering stays at 0.4 rad: starting or stopping at 1 rad or more
# makes the sim's wheels slip (pilsbot/pilsbot_simulation#1).
import math
import os
from pathlib import Path
import shutil
import signal
import time
import unittest

from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from controller_manager_msgs.srv import ListControllers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

CONTROLLERS = {'joint_state_publisher', 'pilsbot_velocity_controller'}
COMMAND_TOPIC = '/pilsbot_velocity_controller/cmd_vel'
STEERING_JOINT = 'steering_axle_joint'
# The gamepad's steering authority (teleop_acker_joy): the controller's
# steering loop is passive at 0.
STEERING_AUTHORITY = 1.0

# Wall-clock limits. The sim runs at 0.2-0.6 of real time; the activator's
# period counts in sim time.
ACTIVE_TIMEOUT = 300.0
MOTION_TIMEOUT = 180.0

# Keeps this run's Gazebo transport apart from any other Gazebo on the machine
# (ROS_DOMAIN_ID is isolated by the CMake test runner), and finds this run's
# processes after the launch.
PARTITION = f'pilsbot_sim_test_{os.getpid()}'


def gazebo_unavailable():
    """Why this machine cannot run the test, or None."""
    if os.environ.get('PILSBOT_SKIP_GAZEBO_TESTS', '') not in ('', '0'):
        return 'PILSBOT_SKIP_GAZEBO_TESTS is set'
    if shutil.which('ign') is None:
        return 'Gazebo Fortress (ign) is not installed'
    for package in ['ros_gz_sim', 'ros_gz_bridge', 'gz_ros2_control']:
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            return f'{package} is not installed'
    return None


@unittest.skipIf(gazebo_unavailable() is not None, gazebo_unavailable())
@pytest.mark.launch_test
def generate_test_description():
    sim_launch = Path(get_package_share_directory('pilsbot_simulation')) / 'launch' / \
        'sim.launch.py'
    return LaunchDescription([
        SetEnvironmentVariable('IGN_PARTITION', PARTITION),
        SetEnvironmentVariable('GZ_PARTITION', PARTITION),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim_launch)),
            launch_arguments={'gui': 'false', 'foxglove': 'false'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class TestSimDrive(unittest.TestCase):
    """Runs in name order: the controllers come up first, then drive, then steer."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node(
            'sim_drive_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
        cls.odom = None
        cls.steering = None
        cls.odom_tf = False
        cls.node.create_subscription(Odometry, '/odom', cls.on_odom, 10)
        cls.node.create_subscription(JointState, '/joint_states', cls.on_joint_states, 10)
        cls.node.create_subscription(TFMessage, '/tf', cls.on_tf, 10)
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

    @classmethod
    def on_joint_states(cls, message):
        if STEERING_JOINT in message.name:
            cls.steering = message.position[message.name.index(STEERING_JOINT)]

    @classmethod
    def on_tf(cls, message):
        cls.odom_tf |= any(t.header.frame_id == 'odom' and t.child_frame_id == 'base_footprint'
                           for t in message.transforms)

    def spin_until(self, condition, timeout, every=None):
        """Spin until condition() is true; call every() about 20 times a second."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if every is not None:
                every()
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if condition():
                return True
        return False

    def drive(self, speed, steering_angle):
        message = AckermannDriveStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.drive.speed = speed
        message.drive.steering_angle = steering_angle
        message.drive.steering_angle_velocity = STEERING_AUTHORITY
        self.command.publish(message)

    def stop(self):
        for _ in range(20):
            self.drive(0.0, 0.0)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def active_controllers(self):
        if not self.list_controllers.wait_for_service(timeout_sec=1.0):
            return set()
        future = self.list_controllers.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        if future.result() is None:
            return set()
        return {c.name for c in future.result().controller if c.state == 'active'}

    def test_1_both_controllers_become_active(self):
        active = set()

        def all_active():
            nonlocal active
            active = self.active_controllers()
            return CONTROLLERS <= active

        self.assertTrue(self.spin_until(all_active, ACTIVE_TIMEOUT),
                        f'active after {ACTIVE_TIMEOUT} s: {sorted(active)}')

    def test_2_a_forward_command_moves_odom_forward(self):
        self.assertTrue(self.spin_until(lambda: self.odom is not None, 30.0), 'no /odom')
        self.assertEqual(self.odom.header.frame_id, 'odom')
        self.assertEqual(self.odom.child_frame_id, 'base_footprint')
        start = self.odom.pose.pose
        heading = yaw_of(start.orientation)

        def moved():
            position = self.odom.pose.pose.position
            dx, dy = position.x - start.position.x, position.y - start.position.y
            return (dx * math.cos(heading) + dy * math.sin(heading),
                    -dx * math.sin(heading) + dy * math.cos(heading))

        try:
            reached = self.spin_until(lambda: moved()[0] > 0.5, MOTION_TIMEOUT,
                                      every=lambda: self.drive(0.4, 0.0))
        finally:
            self.stop()
        forward, sideways = moved()
        self.assertTrue(reached, f'odom moved {forward:.3f} m forward')
        self.assertLess(abs(sideways), 0.15, f'odom moved {sideways:.3f} m sideways')
        self.assertTrue(self.odom_tf, 'no odom -> base_footprint on /tf')

    def test_3_a_steering_command_turns_the_steering_axle(self):
        self.assertTrue(self.spin_until(lambda: self.steering is not None, 30.0),
                        f'no {STEERING_JOINT} on /joint_states')
        start = self.steering
        try:
            reached = self.spin_until(lambda: self.steering - start > 0.3, MOTION_TIMEOUT,
                                      every=lambda: self.drive(0.0, 0.4))
        finally:
            self.stop()
        self.assertTrue(reached, f'{STEERING_JOINT} went from {start:.3f} to '
                                 f'{self.steering:.3f} rad, commanded 0.4')


def processes_in_partition():
    """Pids of the processes started with this run's IGN_PARTITION."""
    marker = f'IGN_PARTITION={PARTITION}'.encode()
    found = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            if marker in (entry / 'environ').read_bytes().split(b'\0'):
                found.append(int(entry.name))
        except OSError:
            pass  # gone, or not ours
    return found


@launch_testing.post_shutdown_test()
class TestNoGazeboLeftBehind(unittest.TestCase):

    def test_stop_the_orphaned_gazebo_server(self):
        # launch stops ros_gz_sim's `ign gazebo` (a ruby wrapper) with SIGINT,
        # then SIGTERM; the gz server it forked does not get the signal and
        # outlives the launch. A terminal's Ctrl-C reaches the whole process
        # group and does not leave it behind.
        leftovers = processes_in_partition()
        for pid in leftovers:
            os.kill(pid, signal.SIGTERM)
        deadline = time.monotonic() + 10.0
        while processes_in_partition() and time.monotonic() < deadline:
            time.sleep(0.2)
        for pid in processes_in_partition():
            os.kill(pid, signal.SIGKILL)
        if leftovers:
            print(f'stopped {len(leftovers)} process(es) left behind by the launch: {leftovers}')
