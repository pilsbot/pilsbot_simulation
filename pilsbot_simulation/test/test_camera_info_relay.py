# scripts/camera_info_relay.py: Gazebo Fortress puts -fx * 0.0375 (half the
# OAK-D baseline) into P[3] of the left, the right and the depth camera_info
# (pilsbot/pilsbot_simulation#4). stereo_image_proc takes the baseline from
# the right camera's P[3] and computes half the depth. The relay publishes
# them under the robot's names with left and depth P[3] = 0 and right
# P[3] = -fx * baseline, the baseline from tf (the URDF).
#
# The relay runs in this process; a static transform stands in for
# robot_state_publisher.
import importlib.util
from pathlib import Path
import time

from geometry_msgs.msg import TransformStamped
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'camera_info_relay.py'

LEFT = 'oak_left_camera_optical_frame'
RIGHT = 'oak_right_camera_optical_frame'
FX = 434.31
GAZEBO_P3 = -FX * 0.0375
BASELINE = 0.075


def load_relay():
    spec = importlib.util.spec_from_file_location('camera_info_relay', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def ros():
    rclpy.init()
    try:
        executor = SingleThreadedExecutor()
        relay = load_relay().CameraInfoRelay()
        node = rclpy.create_node('camera_info_relay_test')
        executor.add_node(relay)
        executor.add_node(node)
        yield executor, node
        executor.shutdown()
        relay.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def gazebo_camera_info(frame_id, stamp_sec=7):
    info = CameraInfo()
    info.header.frame_id = frame_id
    info.header.stamp.sec = stamp_sec
    info.header.stamp.nanosec = 250
    info.width, info.height = 640, 400
    info.k = [FX, 0.0, 320.0, 0.0, FX, 200.0, 0.0, 0.0, 1.0]
    info.p = [FX, 0.0, 320.0, GAZEBO_P3, 0.0, FX, 200.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


def broadcast_baseline(node, x=BASELINE):
    """Put the right optical frame x [m] to the right of the left one, as the URDF does."""
    transform = TransformStamped()
    transform.header.frame_id = LEFT
    transform.child_frame_id = RIGHT
    transform.transform.translation.x = x
    transform.transform.rotation.w = 1.0
    broadcaster = StaticTransformBroadcaster(node)
    broadcaster.sendTransform(transform)
    return broadcaster


def relay_once(executor, node, side, frame_id, timeout=5.0):
    """Publish Gazebo's camera_info for one camera; the relay's output, or None."""
    received = []
    # Reliable and volatile, as a default subscriber asks.
    node.create_subscription(CameraInfo, f'/camera/{side}/camera_info', received.append,
                             QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE))
    publisher = node.create_publisher(CameraInfo, f'/gz/camera/{side}/camera_info', 10)
    message = gazebo_camera_info(frame_id)
    deadline = time.monotonic() + timeout
    while not received and time.monotonic() < deadline:
        publisher.publish(message)
        executor.spin_once(timeout_sec=0.1)
    return received[0] if received else None


def test_right_p3_is_minus_fx_times_the_baseline_from_tf(ros):
    executor, node = ros
    broadcast_baseline(node)

    info = relay_once(executor, node, 'right', RIGHT)

    assert info is not None, 'nothing on /camera/right/camera_info'
    assert info.p[3] == pytest.approx(-FX * BASELINE)


def test_left_p3_is_zero(ros):
    executor, node = ros
    broadcast_baseline(node)

    info = relay_once(executor, node, 'left', LEFT)

    assert info is not None, 'nothing on /camera/left/camera_info'
    assert info.p[3] == 0.0


def test_depth_p3_is_zero(ros):
    # The depth image is aligned to the left camera.
    executor, node = ros

    info = relay_once(executor, node, 'stereo', LEFT)

    assert info is not None, 'nothing on /camera/stereo/camera_info'
    assert info.p[3] == 0.0


def test_the_rest_of_the_message_is_unchanged(ros):
    # Stamps unchanged: stereo_image_proc pairs the images by exact time.
    executor, node = ros
    broadcast_baseline(node)
    original = gazebo_camera_info(RIGHT)

    info = relay_once(executor, node, 'right', RIGHT)

    assert info.header == original.header
    assert (info.width, info.height) == (original.width, original.height)
    assert list(info.k) == list(original.k)
    assert [v for i, v in enumerate(info.p) if i != 3] == \
        [v for i, v in enumerate(original.p) if i != 3]


def test_the_left_camera_is_relayed_before_tf_is_there(ros):
    executor, node = ros

    assert relay_once(executor, node, 'left', LEFT) is not None


def test_without_tf_the_right_camera_info_is_held_back(ros):
    # Better none than one with Gazebo's half baseline.
    executor, node = ros

    assert relay_once(executor, node, 'right', RIGHT, timeout=2.0) is None
