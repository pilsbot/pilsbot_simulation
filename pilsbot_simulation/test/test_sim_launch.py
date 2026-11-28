# What sim.launch.py starts. The launch description is expanded without
# starting anything: the included launch files are identified by their
# package, not loaded.
import importlib.util
from pathlib import Path
import re
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, \
    OpaqueFunction, PopLaunchConfigurations, PushLaunchConfigurations, SetLaunchConfiguration
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest

PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE / 'launch' / 'sim.launch.py'


def expand(**arguments):
    """
    Declare the arguments and run the OpaqueFunctions and groups, like `ros2 launch` does.

    Each include is returned with the launch configurations it would see, in
    LAUNCH_CONFIGURATIONS[id(include)].
    """
    spec = importlib.util.spec_from_file_location('sim_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    context = LaunchContext()
    context.launch_configurations.update(arguments)
    pending = list(module.generate_launch_description().entities)
    actions = []
    while pending:
        entity = pending.pop(0)
        if entity.condition is not None and not entity.condition.evaluate(context):
            continue
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(context)  # applies the default and checks the choices
        elif isinstance(entity, (OpaqueFunction, GroupAction)):
            pending[:0] = entity.visit(context) or []
        elif isinstance(entity, (SetLaunchConfiguration, PushLaunchConfigurations,
                                 PopLaunchConfigurations)):
            entity.visit(context)
        else:
            if isinstance(entity, IncludeLaunchDescription):
                LAUNCH_CONFIGURATIONS[id(entity)] = dict(context.launch_configurations)
            actions.append(entity)
    return context, actions


LAUNCH_CONFIGURATIONS = {}


def evaluate(context, substitutions):
    return perform_substitutions(context, normalize_to_list_of_substitutions(substitutions))


def includes(context, actions):
    """{package of the included launch file: {argument: value}} of what is started."""
    started = {}
    for include in actions:
        if isinstance(include, IncludeLaunchDescription):
            location = include.launch_description_source.location
            # Humble's launch prints an unresolved substitution path as
            # "...FindPackageShare(pkg='<name>')..."; no public accessor.
            [package] = re.findall(r"FindPackageShare\(pkg='([^']+)'\)", location)
            started[package] = {evaluate(context, key): evaluate(context, value)
                                for key, value in include.launch_arguments}
    return started


def nodes(actions, executable):
    return [a for a in actions if isinstance(a, Node) and a.node_executable == executable]


def node_arguments(context, node):
    # Node._Node__arguments: Humble's launch_ros has no public accessor.
    return [evaluate(context, argument) for argument in node._Node__arguments]


def node_parameters(context, node):
    merged = {}
    for entry in evaluate_parameters(context, node._Node__parameters):
        merged.update(entry)
    return merged


def include(actions, package):
    [found] = [a for a in actions if isinstance(a, IncludeLaunchDescription) and
               f"FindPackageShare(pkg='{package}')" in a.launch_description_source.location]
    return found


def gz_args(**arguments):
    context, actions = expand(**arguments)
    return shlex.split(includes(context, actions)['ros_gz_sim']['gz_args'])


# --- Gazebo -------------------------------------------------------------------

def test_starts_gazebo_running_with_the_flat_world_by_default():
    args = gz_args()

    assert '-r' in args
    assert Path(args[-1]).name == 'flat.sdf'
    assert Path(args[-1]).parent.name == 'worlds'


def test_flat_is_the_packages_flat_world():
    assert gz_args(world='flat')[-1] == gz_args()[-1]


@pytest.mark.parametrize('world, sdf', [('citysim', 'simple_city.sdf'),
                                        ('citysim_block', 'simple_city_block.sdf')])
def test_citysim_names_resolve_to_the_installed_citysim_worlds(world, sdf):
    # citysim: the full city; citysim_block: the cropped block around the
    # spawn road (pilsbot/pilsbot_simulation#7).
    path = Path(gz_args(world=world)[-1])

    assert path == Path(get_package_share_directory('citysim')) / 'worlds' / sdf
    assert path.is_file()


def test_world_argument_replaces_the_world():
    assert gz_args(world='/somewhere/city.sdf')[-1] == '/somewhere/city.sdf'


@pytest.mark.parametrize('world', ['city.sdf', 'worlds/city.sdf', './city', 'old.world'])
def test_a_relative_path_or_a_file_name_reaches_gazebo_unchanged(world):
    # Gazebo looks a bare file name up on its resource path.
    assert gz_args(world=world)[-1] == world


@pytest.mark.parametrize('world', ['city', 'Citysim', 'flat_ground', 'citysim-block'])
def test_an_unknown_world_name_fails_with_the_names_it_knows(world):
    with pytest.raises(RuntimeError) as error:
        expand(world=world)

    message = str(error.value)
    assert f'"{world}"' in message
    for known in ['flat', 'citysim', 'citysim_block']:
        assert known in message
    assert 'path' in message


@pytest.mark.parametrize('world', ['/my worlds/city.sdf', "/w/it's;$(rm -rf x)&.sdf"])
def test_a_world_path_with_spaces_or_shell_characters_reaches_gazebo_unchanged(world):
    # ros_gz_sim runs gz_args through a shell.
    assert gz_args(world=world)[-1] == world


def test_the_shell_is_replaced_by_gazebo_so_a_signal_to_the_launch_reaches_it():
    # ros_gz_sim starts `ruby ign gazebo` with shell=True. Without exec, the
    # shell gets launch's SIGINT/SIGTERM on shutdown and the gz server
    # outlives the launch.
    _, actions = expand()

    gazebo = LAUNCH_CONFIGURATIONS[id(include(actions, 'ros_gz_sim'))]
    assert gazebo['launch-prefix'] == 'exec'
    assert gazebo.get('launch-prefix-filter', '') == ''


def test_the_exec_prefix_stays_with_gazebo():
    _, actions = expand(**{'launch-prefix': 'valgrind'})

    robot = LAUNCH_CONFIGURATIONS[id(include(actions, 'pilsbot_bringup'))]
    assert robot['launch-prefix'] == 'valgrind'


def test_gui_is_the_default():
    args = gz_args()

    assert '-s' not in args
    assert '--headless-rendering' not in args


def test_without_gui_runs_the_server_only_with_headless_rendering():
    args = gz_args(gui='false')

    assert '-s' in args
    assert '--headless-rendering' in args


def test_the_whole_launch_ends_when_gazebo_ends():
    context, actions = expand()

    assert includes(context, actions)['ros_gz_sim']['on_exit_shutdown'] == 'true'


@pytest.mark.parametrize('value', ['', 'yes', 'headless'])
def test_an_unknown_gui_value_fails_the_launch(value):
    with pytest.raises(RuntimeError, match='"gui"'):
        expand(gui=value)


# --- Spawn --------------------------------------------------------------------

def test_spawns_pilsbot_from_robot_description_at_the_origin_by_default():
    context, actions = expand()

    [create] = nodes(actions, 'create')
    arguments = node_arguments(context, create)
    assert arguments[arguments.index('-topic') + 1] == 'robot_description'
    assert arguments[arguments.index('-name') + 1] == 'pilsbot'
    pose = {flag: float(arguments[arguments.index(flag) + 1])
            for flag in ['-x', '-y', '-z', '-Y']}
    # z: just above the ground, so the wheels do not start inside it.
    assert pose == {'-x': 0.0, '-y': 0.0, '-z': pytest.approx(0.02), '-Y': 0.0}


def spawn_pose(**arguments):
    context, actions = expand(**arguments)
    [create] = nodes(actions, 'create')
    arguments = node_arguments(context, create)
    return [float(arguments[arguments.index(flag) + 1]) for flag in ['-x', '-y', '-z', '-Y']]


def test_spawn_pose_comes_from_the_arguments():
    context, actions = expand(x='1.5', y='-2', z='0.1', yaw='1.57')

    [create] = nodes(actions, 'create')
    arguments = node_arguments(context, create)
    assert [arguments[arguments.index(flag) + 1] for flag in ['-x', '-y', '-z', '-Y']] == \
        ['1.5', '-2', '0.1', '1.57']


@pytest.mark.parametrize('world', ['citysim', 'citysim_block'])
def test_in_citysim_the_robot_spawns_on_a_road_by_default(world):
    # road_x_3, right lane, heading along the road; street level is 5.01 m
    # (citysim's README), 2 cm above it as on the flat world.
    assert spawn_pose(world=world) == [-35.0, -1.8, pytest.approx(5.03), 0.0]


@pytest.mark.parametrize('world', ['flat', '/somewhere/city.sdf'])
def test_on_the_flat_world_and_on_a_path_the_robot_spawns_at_the_origin(world):
    assert spawn_pose(world=world) == [0.0, 0.0, pytest.approx(0.02), 0.0]


def test_a_spawn_argument_overrides_the_worlds_default_alone():
    assert spawn_pose(world='citysim', x='-20', yaw='3.14') == \
        [-20.0, -1.8, pytest.approx(5.03), pytest.approx(3.14)]


# --- Bridges and cameras -------------------------------------------------------

# The robot's image topics, and the compressed transport the robot offers on
# each besides raw. Depth is 32FC1 in the sim: compressed (JPEG/PNG) fails on
# every frame, compressedDepth works (pilsbot/pilsbot_simulation#4).
IMAGE_TRANSPORTS = {
    '/camera/color/image_raw': 'compressed',
    '/camera/left/image_rect': 'compressed',
    '/camera/right/image_rect': 'compressed',
    '/camera/stereo/depth': 'compressedDepth',
    '/front_camera/image_raw': 'compressed',
}


@pytest.mark.parametrize('cameras, config', [('true', 'bridge.yaml'),
                                             ('false', 'bridge_no_cameras.yaml')])
def test_bridges_from_the_package_config_on_sim_time(cameras, config):
    context, actions = expand(cameras=cameras)

    [bridge] = nodes(actions, 'parameter_bridge')
    parameters = node_parameters(context, bridge)
    assert Path(parameters['config_file']).name == config
    assert Path(parameters['config_file']).parent.name == 'config'
    assert parameters['use_sim_time'] is True


def test_cameras_are_on_by_default():
    context, actions = expand()

    assert includes(context, actions)['pilsbot_bringup']['cameras'] == 'true'
    assert len(nodes(actions, 'image_bridge')) == 1


def test_image_bridge_bridges_the_robots_image_topics_on_sim_time():
    context, actions = expand(cameras='true')

    [image_bridge] = nodes(actions, 'image_bridge')
    assert sorted(node_arguments(context, image_bridge)) == sorted(IMAGE_TRANSPORTS)
    assert node_parameters(context, image_bridge)['use_sim_time'] is True


def test_each_image_gets_raw_and_the_robots_compressed_transport_only():
    # image_transport's <topic>.enable_pub_plugins (topic without the leading
    # slash, dots for slashes). Without it every installed plugin publishes,
    # and /camera/stereo/depth/compressed logs an error per frame.
    context, actions = expand(cameras='true')

    [image_bridge] = nodes(actions, 'image_bridge')
    parameters = node_parameters(context, image_bridge)
    for topic, transport in IMAGE_TRANSPORTS.items():
        plugins = parameters[topic[1:].replace('/', '.') + '.enable_pub_plugins']
        assert sorted(plugins) == sorted(['image_transport/raw', f'image_transport/{transport}'])


def test_the_camera_info_relay_runs_on_sim_time_with_the_cameras():
    context, actions = expand(cameras='true')

    [relay] = nodes(actions, 'camera_info_relay.py')
    assert relay.node_package == 'pilsbot_simulation'
    assert node_parameters(context, relay)['use_sim_time'] is True


def test_without_cameras_no_image_bridge_and_no_relay_start():
    _, actions = expand(cameras='false')

    assert nodes(actions, 'image_bridge') == []
    assert nodes(actions, 'camera_info_relay.py') == []


def test_cameras_reaches_bringup():
    context, actions = expand(cameras='false')

    assert includes(context, actions)['pilsbot_bringup']['cameras'] == 'false'


@pytest.mark.parametrize('value', ['', 'False', 'no'])
def test_an_unknown_cameras_value_fails_the_launch(value):
    with pytest.raises(RuntimeError, match='"cameras"'):
        expand(cameras=value)


# --- The robot's stack --------------------------------------------------------

def test_starts_the_robot_stack_through_bringup_with_gazebo_hardware_and_sim_time():
    context, actions = expand()

    assert includes(context, actions)['pilsbot_bringup'] == {
        'hardware': 'gazebo', 'use_sim_time': 'true', 'robot_profile': 'pilsbot',
        'cameras': 'true'}


def test_robot_profile_reaches_bringup():
    context, actions = expand(robot_profile='/profiles/other.yaml')

    assert includes(context, actions)['pilsbot_bringup']['robot_profile'] == \
        '/profiles/other.yaml'


@pytest.mark.parametrize('cameras, sim_nodes', [
    ('true', {'create', 'parameter_bridge', 'image_bridge', 'camera_info_relay.py'}),
    ('false', {'create', 'parameter_bridge'})])
def test_starts_nothing_the_robot_stack_starts_itself(cameras, sim_nodes):
    # robot_state_publisher, the controller manager, the spawners and the
    # activator come from bringup, as on the robot.
    context, actions = expand(cameras=cameras)

    started = {a.node_executable for a in actions if isinstance(a, Node)}
    assert started == sim_nodes
    assert set(includes(context, actions)) == {'ros_gz_sim', 'pilsbot_bringup'}
