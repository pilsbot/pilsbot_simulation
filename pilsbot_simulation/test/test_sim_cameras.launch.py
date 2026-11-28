# Headless end-to-end test of the camera topics, run twice:
#
# - cameras:=true: every image topic of the robot publishes raw, its
#   compressed transport (and no other) and camera_info, in the image's
#   optical frame, and the stereo pair's camera_info carries the baseline
#   stereo_image_proc needs. Gazebo's own point cloud is not on ROS.
# - cameras:=false: no camera topic exists once the controllers are up.
#
# Needs Gazebo Fortress (ign), ros_gz and gz_ros2_control and takes a few
# minutes. Skipped like test_sim_drive.launch.py (sim_test_support.py).
import posixpath
from pathlib import Path
import sys
import time
import unittest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_test_support import gazebo_unavailable, NothingLeftBehind, \
    partition_actions, sim_launch_file  # noqa: E402, I100

# The robot's image topics: frame and compressed transport. Depth is 32FC1
# in the sim, which only compressedDepth can compress.
IMAGES = {
    '/camera/color/image_raw': ('oak_rgb_camera_optical_frame', 'compressed'),
    '/camera/left/image_rect': ('oak_left_camera_optical_frame', 'compressed'),
    '/camera/right/image_rect': ('oak_right_camera_optical_frame', 'compressed'),
    '/camera/stereo/depth': ('oak_left_camera_optical_frame', 'compressedDepth'),
    '/front_camera/image_raw': ('front_camera_optical_frame', 'compressed'),
}
# OAK-D: the right camera 7.5 cm to the right of the left one (the URDF).
BASELINE = 0.075

# Wall-clock limits: with the cameras the sim runs at about 0.2-0.5 of real
# time, and the robot spawns only once Gazebo runs.
TOPICS_TIMEOUT = 300.0
UP_TIMEOUT = 300.0
QUIET_TIME = 10.0


def camera_info_of(image_topic):
    return posixpath.join(posixpath.dirname(image_topic), 'camera_info')


def expected_topics():
    """{topic: message type} that the cameras publish."""
    topics = {}
    for topic, (_, transport) in IMAGES.items():
        topics[topic] = Image
        topics[f'{topic}/{transport}'] = CompressedImage
        topics[camera_info_of(topic)] = CameraInfo
    return topics


@launch_testing.parametrize('cameras', ['true', 'false'])
@unittest.skipIf(gazebo_unavailable() is not None, gazebo_unavailable())
@pytest.mark.launch_test
def generate_test_description(cameras):
    return LaunchDescription(partition_actions() + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim_launch_file())),
            launch_arguments={'gui': 'false', 'foxglove': 'false',
                              'cameras': cameras}.items()),
        launch_testing.actions.ReadyToTest(),
    ]), {'cameras': cameras}


class TestCameraTopics(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('sim_cameras_test')
        cls.received = {}
        cls.joint_states = False
        cls.subscriptions = {
            topic: cls.node.create_subscription(
                message_type, topic,
                lambda message, topic=topic: cls.received.setdefault(topic, message),
                qos_profile_sensor_data)
            for topic, message_type in expected_topics().items()}
        cls.node.create_subscription(JointState, '/joint_states', cls.on_joint_states, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def on_joint_states(cls, _):
        cls.joint_states = True

    def spin_until(self, condition, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.unsubscribe_received()
            if condition():
                return True
        return False

    def unsubscribe_received(self):
        # One message per topic is enough; raw images cost the sim CPU.
        for topic in set(self.received) & set(self.subscriptions):
            self.node.destroy_subscription(self.subscriptions.pop(topic))

    def publishers(self, topic):
        return self.node.count_publishers(topic)

    def published_topics(self):
        # Topics this test subscribes to exist without a publisher.
        return {name for name, _ in self.node.get_topic_names_and_types()
                if self.publishers(name) > 0}

    def test_1_every_camera_topic_publishes_with_the_cameras(self, cameras):
        if cameras != 'true':
            self.skipTest('cameras:=false')
        expected = set(expected_topics())

        self.spin_until(lambda: expected <= set(self.received), TOPICS_TIMEOUT)

        self.assertEqual(sorted(expected - set(self.received)), [],
                         f'no message within {TOPICS_TIMEOUT} s')

    def test_2_images_and_camera_info_are_in_the_cameras_optical_frame(self, cameras):
        if cameras != 'true':
            self.skipTest('cameras:=false')
        for topic, (frame, transport) in IMAGES.items():
            for name in [topic, f'{topic}/{transport}', camera_info_of(topic)]:
                with self.subTest(topic=name):
                    self.assertIn(name, self.received)
                    self.assertEqual(self.received[name].header.frame_id, frame)

    def test_3_the_stereo_pair_carries_the_baseline(self, cameras):
        # stereo_image_proc: left P[3] = 0, right P[3] = -fx * baseline.
        # Gazebo alone puts -fx * 0.0375 on both (pilsbot/pilsbot_simulation#4).
        if cameras != 'true':
            self.skipTest('cameras:=false')
        left = self.received['/camera/left/camera_info']
        right = self.received['/camera/right/camera_info']
        depth = self.received['/camera/stereo/camera_info']

        self.assertEqual(left.p[3], 0.0)
        self.assertAlmostEqual(right.p[3], -right.p[0] * BASELINE, delta=1e-3 * right.p[0])
        self.assertEqual(depth.p[3], 0.0)

    def test_4_gazebos_point_cloud_is_not_published(self, cameras):
        # Gazebo's cloud is x-forward data under an optical-frame header.
        if cameras != 'true':
            self.skipTest('cameras:=false')
        for topic in ['/camera/stereo/points', '/camera/stereo/depth/points']:
            with self.subTest(topic=topic):
                self.assertEqual(self.publishers(topic), 0)

    def test_5_each_image_offers_only_its_compressed_transport(self, cameras):
        # No theora, no compressedDepth beside colour/mono images, and no
        # /compressed beside depth: JPEG/PNG of 32FC1 fails on every frame.
        if cameras != 'true':
            self.skipTest('cameras:=false')
        published = self.published_topics()
        for topic, (_, transport) in IMAGES.items():
            with self.subTest(topic=topic):
                self.assertEqual(sorted(t for t in published if t.startswith(f'{topic}/')),
                                 [f'{topic}/{transport}'])

    def test_6_without_cameras_no_camera_topic_exists(self, cameras):
        if cameras != 'false':
            self.skipTest('cameras:=true')
        # The robot is in Gazebo once its controllers publish.
        self.assertTrue(self.spin_until(lambda: self.joint_states, UP_TIMEOUT),
                        f'no /joint_states within {UP_TIMEOUT} s')
        self.spin_until(lambda: False, QUIET_TIME)

        self.assertEqual(sorted(self.received), [])
        published = self.published_topics()
        self.assertEqual(sorted(t for t in published if 'camera' in t), [])


@launch_testing.post_shutdown_test()
class TestNothingLeftBehind(NothingLeftBehind, unittest.TestCase):
    pass
