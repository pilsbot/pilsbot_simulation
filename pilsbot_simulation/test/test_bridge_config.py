# config/bridge.yaml against the sensors in pilsbot_description's
# hardware:=gazebo URDF: every sensor reaches ROS under the robot's topic
# names, and nothing goes from ROS into Gazebo.
import posixpath
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

BRIDGE_CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'bridge.yaml'

# The robot's topic names (Humble pilsbot_depthai mjpeg_node, jetson_camera).
# /camera/stereo/points has no robot counterpart: the robot computes it on
# the host.
ROS_TOPIC_OF_GZ_TOPIC = {
    'camera/color/image_raw': '/camera/color/image_raw',
    'camera/color/camera_info': '/camera/color/camera_info',
    'camera/left/image_rect': '/camera/left/image_rect',
    'camera/left/camera_info': '/camera/left/camera_info',
    'camera/right/image_rect': '/camera/right/image_rect',
    'camera/right/camera_info': '/camera/right/camera_info',
    'camera/stereo/depth': '/camera/stereo/depth',
    'camera/stereo/camera_info': '/camera/stereo/camera_info',
    'camera/stereo/depth/points': '/camera/stereo/points',
    'front_camera/image_raw': '/image_raw',
    'front_camera/camera_info': '/camera_info',
}


@pytest.fixture(scope='module')
def bridge():
    entries = yaml.safe_load(BRIDGE_CONFIG.read_text())
    return {entry['gz_topic_name']: entry for entry in entries}


@pytest.fixture(scope='module')
def sensors(tmp_path_factory):
    """{gz topic: sensor type} of the hardware:=gazebo URDF."""
    overrides = tmp_path_factory.mktemp('overrides') / 'overrides.yaml'
    overrides.write_text('{}\n')
    xacro_file = Path(get_package_share_directory('pilsbot_description')) / 'urdf' / \
        'pilsbot.urdf.xacro'
    urdf = subprocess.run(
        ['xacro', str(xacro_file), 'hardware:=gazebo', f'controller_overrides:={overrides}'],
        check=True, capture_output=True, text=True).stdout
    found = {sensor.findtext('topic'): sensor.get('type')
             for sensor in ET.fromstring(urdf).iter('sensor')}
    assert found, 'the hardware:=gazebo URDF has no sensors'
    return found


def gz_topics_of(topic, sensor_type):
    """{gz topic: gz message type} that Fortress publishes for one camera sensor."""
    # gz puts camera_info next to the image topic, the point cloud below it.
    topics = {topic: 'ignition.msgs.Image',
              posixpath.join(posixpath.dirname(topic), 'camera_info'):
                  'ignition.msgs.CameraInfo'}
    if sensor_type == 'depth_camera':
        topics[topic + '/points'] = 'ignition.msgs.PointCloudPacked'
    return topics


def test_clock_is_bridged(bridge):
    assert bridge['/clock']['ros_topic_name'] == '/clock'
    assert bridge['/clock']['gz_type_name'] == 'ignition.msgs.Clock'


def test_every_entry_goes_from_gazebo_to_ros(bridge):
    assert {entry['direction'] for entry in bridge.values()} == {'GZ_TO_ROS'}


def test_every_sensor_topic_is_bridged_under_the_robots_name(bridge, sensors):
    for topic, sensor_type in sensors.items():
        for gz_topic, gz_type in gz_topics_of(topic, sensor_type).items():
            assert gz_topic in bridge, f'{gz_topic} ({sensor_type}) is not bridged'
            assert bridge[gz_topic]['gz_type_name'] == gz_type
            assert bridge[gz_topic]['ros_topic_name'] == ROS_TOPIC_OF_GZ_TOPIC[gz_topic]


def test_nothing_but_the_clock_and_the_sensors_is_bridged(bridge, sensors):
    expected = {'/clock'}
    for topic, sensor_type in sensors.items():
        expected |= set(gz_topics_of(topic, sensor_type))

    assert set(bridge) == expected


ROS_TYPE_OF_GZ_TYPE = {
    'ignition.msgs.Clock': 'rosgraph_msgs/msg/Clock',
    'ignition.msgs.Image': 'sensor_msgs/msg/Image',
    'ignition.msgs.CameraInfo': 'sensor_msgs/msg/CameraInfo',
    'ignition.msgs.PointCloudPacked': 'sensor_msgs/msg/PointCloud2',
}


def test_ros_types_match_the_gz_types(bridge):
    for gz_topic, entry in bridge.items():
        assert entry['ros_type_name'] == ROS_TYPE_OF_GZ_TYPE[entry['gz_type_name']], gz_topic
