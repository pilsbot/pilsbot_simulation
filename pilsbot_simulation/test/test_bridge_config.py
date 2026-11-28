# The bridges against the sensors in pilsbot_description's hardware:=gazebo
# URDF: every camera reaches ROS under the robot's topic names, nothing goes
# from ROS into Gazebo, and Gazebo's own point cloud is not bridged.
#
# parameter_bridge (config/bridge.yaml, cameras:=true, or
# config/bridge_no_cameras.yaml) carries /clock and camera_info. The images go
# through ros_gz_image's image_bridge (sim.launch.py's IMAGE_TRANSPORTS). The
# left, right and depth camera_info go to internal names, and
# scripts/camera_info_relay.py publishes them under the robot's names with
# the stereo baseline fixed.
import functools
import importlib.util
import posixpath
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
BRIDGE_CONFIG = {'true': PACKAGE / 'config' / 'bridge.yaml',
                 'false': PACKAGE / 'config' / 'bridge_no_cameras.yaml'}

# The robot's image topics (Humble pilsbot_depthai mjpeg_node, pilsbot_bringup
# camera.launch.py), by gz topic.
ROBOT_IMAGE_TOPIC = {
    'camera/color/image_raw': '/camera/color/image_raw',
    'camera/left/image_rect': '/camera/left/image_rect',
    'camera/right/image_rect': '/camera/right/image_rect',
    'camera/stereo/depth': '/camera/stereo/depth',
    'front_camera/image_raw': '/front_camera/image_raw',
}

# Where parameter_bridge puts each gz camera_info: the robot's name, or the
# relay's input for the three that carry Gazebo's wrong baseline.
ROS_TOPIC_OF_GZ_CAMERA_INFO = {
    'camera/color/camera_info': '/camera/color/camera_info',
    'camera/left/camera_info': '/gz/camera/left/camera_info',
    'camera/right/camera_info': '/gz/camera/right/camera_info',
    'camera/stereo/camera_info': '/gz/camera/stereo/camera_info',
    'front_camera/camera_info': '/front_camera/camera_info',
}

ROS_TYPE_OF_GZ_TYPE = {
    'ignition.msgs.Clock': 'rosgraph_msgs/msg/Clock',
    'ignition.msgs.CameraInfo': 'sensor_msgs/msg/CameraInfo',
}


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def image_transports():
    return load_module('sim_launch', PACKAGE / 'launch' / 'sim.launch.py').IMAGE_TRANSPORTS


@pytest.fixture(scope='module')
def relayed():
    return load_module('camera_info_relay', PACKAGE / 'scripts' / 'camera_info_relay.py').RELAYED


def bridge(cameras):
    entries = yaml.safe_load(BRIDGE_CONFIG[cameras].read_text())
    return {entry['gz_topic_name']: entry for entry in entries}


@functools.lru_cache()
def sensors(cameras):
    """{gz image topic: sensor type} of the hardware:=gazebo URDF."""
    xacro_file = Path(get_package_share_directory('pilsbot_description')) / 'urdf' / \
        'pilsbot.urdf.xacro'
    # The xacro only needs the overrides file's name; gz_ros2_control reads it.
    urdf = subprocess.run(
        ['xacro', str(xacro_file), 'hardware:=gazebo', 'controller_overrides:=/dev/null',
         f'cameras:={cameras}'],
        check=True, capture_output=True, text=True).stdout
    return {sensor.findtext('topic'): sensor.get('type')
            for sensor in ET.fromstring(urdf).iter('sensor')}


def camera_info_of(image_topic):
    # gz puts camera_info next to the image topic.
    return posixpath.join(posixpath.dirname(image_topic), 'camera_info')


def test_the_urdf_has_the_cameras_only_with_cameras_true():
    assert set(sensors('true')) == set(ROBOT_IMAGE_TOPIC)
    assert sensors('false') == {}


@pytest.mark.parametrize('cameras', ['true', 'false'])
def test_clock_is_bridged(cameras):
    clock = bridge(cameras)['/clock']
    assert clock['ros_topic_name'] == '/clock'
    assert clock['gz_type_name'] == 'ignition.msgs.Clock'


@pytest.mark.parametrize('cameras', ['true', 'false'])
def test_every_entry_goes_from_gazebo_to_ros(cameras):
    assert {entry['direction'] for entry in bridge(cameras).values()} == {'GZ_TO_ROS'}


@pytest.mark.parametrize('cameras', ['true', 'false'])
def test_ros_types_match_the_gz_types(cameras):
    for gz_topic, entry in bridge(cameras).items():
        assert entry['ros_type_name'] == ROS_TYPE_OF_GZ_TYPE[entry['gz_type_name']], gz_topic


@pytest.mark.parametrize('cameras', ['true', 'false'])
def test_parameter_bridge_carries_the_clock_and_each_cameras_camera_info_only(cameras):
    # No images (image_bridge has them) and no point cloud: Gazebo's is
    # x-forward data under an optical-frame header (pilsbot/pilsbot_simulation#4).
    expected = {'/clock'} | {camera_info_of(topic) for topic in sensors(cameras)}

    assert set(bridge(cameras)) == expected


def test_every_camera_info_is_bridged_to_its_ros_name():
    entries = bridge('true')
    for topic in sensors('true'):
        gz_topic = camera_info_of(topic)
        assert entries[gz_topic]['gz_type_name'] == 'ignition.msgs.CameraInfo'
        assert entries[gz_topic]['ros_topic_name'] == ROS_TOPIC_OF_GZ_CAMERA_INFO[gz_topic]


def test_image_bridge_bridges_every_camera_image_under_the_robots_name(image_transports):
    # image_bridge uses one name for gz and ROS: the robot's, with a slash.
    assert set(image_transports) == {'/' + topic for topic in sensors('true')}
    assert set(image_transports) == set(ROBOT_IMAGE_TOPIC.values())


def test_the_relay_takes_the_internal_camera_infos_and_publishes_the_robots_names(relayed):
    internal = {ros for ros in ROS_TOPIC_OF_GZ_CAMERA_INFO.values() if ros.startswith('/gz/')}

    assert set(relayed) == internal
    assert {relayed[topic] for topic in internal} == \
        {'/camera/left/camera_info', '/camera/right/camera_info', '/camera/stereo/camera_info'}
