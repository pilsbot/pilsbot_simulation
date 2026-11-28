# pilsbot_simulation

Pilsbot in **Gazebo Fortress**, driven by the robot's own stack. `sim.launch.py` starts `gz sim` with a world (flat ground, or [citysim](https://github.com/pilsbot/citysim)'s city), spawns the robot from `robot_description`, bridges `/clock` and the cameras to ROS (images through `ros_gz_image`, so each has a compressed transport beside it), and starts `pilsbot_bringup`'s `pilsbot.launch.py` with `hardware:=gazebo use_sim_time:=true`. That is the same launch code as on the robot: robot_state_publisher, the spawners (`--inactive`) and the hardware activator, teleop and foxglove_bridge. The controller manager runs inside Gazebo's `gz_ros2_control` plugin and loads the real `acker_diff_controller.yaml` with the robot profile's geometry. The steering pivot is free, so the robot steers passively by the difference in front wheel speeds, as on the robot. GNSS and lighting are not started.

Gazebo Classic is not supported.

## Requirements

Gazebo Fortress with `ros-humble-ros-gz`, `ros-humble-gz-ros2-control` and `ros-humble-image-transport-plugins` (apt), and the Humble Pilsbot stack (`pilsbot_bringup`, `pilsbot_hw_ros`, `pils_control`, `pilsbot_description`, `teleop_acker_joy`) with `citysim` (its `humble` branch). `colcon build --packages-up-to pilsbot_simulation` builds the Pilsbot packages it needs, `pilsbot_driver` and `citysim` included.

## Run

```
ros2 launch pilsbot_simulation sim.launch.py              # with the Gazebo GUI
ros2 launch pilsbot_simulation sim.launch.py gui:=false   # server only, cameras render headless
ros2 launch pilsbot_simulation sim.launch.py cameras:=false   # no camera sensors: faster, for control work
ros2 launch pilsbot_simulation sim.launch.py world:=citysim   # the city, the robot on a road
```

Stop it with Ctrl-C, or with a signal to the `ros2 launch` process alone (`kill <pid>`, systemd): Gazebo runs as `exec`, so it gets launch's signal and stops with the rest.

The controllers become active a few seconds of sim time after the launch (the hardware activator's period). The sim usually runs below real time with the cameras on, so this takes longer on the wall clock.

| argument | default | |
|---|---|---|
| `world` | `flat` | `flat`, `citysim`, `citysim_block` or a path to an SDF file (see [Worlds](#worlds)) |
| `gui` | `true` | `false`: no GUI, `gz sim -s --headless-rendering` |
| `robot_profile` | `pilsbot` | profile name shipped with `pilsbot_description`, or a path |
| `x`, `y`, `z`, `yaw` | the world's spawn point | spawn pose [m, rad]; each one given overrides the world's value alone |
| `cameras` | `true` | `false`: no camera sensors in the URDF (passed through bringup to xacro), no camera topics; nothing renders |

Arguments of `pilsbot_bringup` and the driver launch files pass through, e.g. `foxglove:=false` or `joystick_device_name:=...`.

### Worlds

| `world` | SDF | spawn point (`x`, `y`, `z`, `yaw`) | |
|---|---|---|---|
| `flat` (default) | `worlds/flat.sdf` | `0`, `0`, `0.02`, `0` | flat ground: tests and control work |
| `citysim` | citysim's `worlds/simple_city.sdf` | `-35`, `-1.8`, `5.03`, `0` | the full city: streets, houses, trees, parked cars, walking people |
| `citysim_block` | citysim's `worlds/simple_city_block.sdf` | `-35`, `-1.8`, `5.03`, `0` | a block of the city around the spawn road, for perception and mapping |
| a path, e.g. `/home/me/my.sdf` | that file | `0`, `0`, `0.02`, `0` | a value with a `/` or ending in `.sdf` or `.world`; a bare file name is looked up on Gazebo's resource path |

Any other name fails the launch with the list of names. In both citysim worlds the robot spawns on `road_x_3` in the right lane, heading along the road (+x), 2 cm above the street (street level is 5.01 m).

citysim is slower than the flat world: with the cameras on, the full city runs at about 0.2 of real time on a laptop, the block at about 0.23; the controllers become active within about a minute of the launch (35–70 s measured in the full city). `cameras:=false` makes either faster when no images are needed.

**First launch of citysim:** its 54 external models (buildings, trees, cars, street furniture) come from [Gazebo Fuel](https://app.gazebosim.org/OpenRobotics) and are downloaded to `~/.ignition/fuel` on first use: about 110 MB, two minutes, during which Gazebo shows no world yet and the robot is not spawned. Later launches load from that cache (about 20 s for the full city). Without a network, the models are not in the cache and the city cannot load.

### With a gamepad

Plug in the gamepad before the launch. Teleop is the robot's: `joy_node` picks the gamepad and `teleop_acker_joy` its mapping from the robot profile; override them with `joystick_device_id:=`, `joystick_device_name:=` or `joystick_conf:=` (a mapping file in `pilsbot_driver/config`). It sends `AckermannDriveStamped` to `/pilsbot_velocity_controller/cmd_vel`.

Without a gamepad, publish a command (it times out after 0.5 s, hence the rate):

```
ros2 topic pub -r 20 /pilsbot_velocity_controller/cmd_vel ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 0.4, steering_angle: 0.3, steering_angle_velocity: 1.0}}"
```

`steering_angle_velocity` is the steering authority (the gamepad sends 1.0); at 0 the steering loop is passive.

### Foxglove

foxglove_bridge runs on port 8765 as on the robot (read-only by default). Open `ws://localhost:8765` in Foxglove and use `pilsbot_bringup`'s layout.

## Topics and frames

The same as on the robot: `odom` → `base_footprint` from the controller, `/joint_states`, `/tf` from robot_state_publisher. Gazebo to ROS only: `/clock` and `camera_info` through `parameter_bridge` (`config/bridge.yaml`), the images through `ros_gz_image`'s `image_bridge` (image_transport).

| ROS topic | frame | |
|---|---|---|
| `/clock` | | sim time |
| `/camera/color/image_raw` (+ `/compressed`), `/camera/color/camera_info` | `oak_rgb_camera_optical_frame` | OAK colour, rgb8 1280×720 |
| `/camera/left/image_rect`, `/camera/right/image_rect` (+ `/compressed`, `camera_info`) | `oak_{left,right}_camera_optical_frame` | OAK mono, mono8 640×400 |
| `/camera/stereo/depth` (+ `/compressedDepth`), `/camera/stereo/camera_info` | `oak_left_camera_optical_frame` | OAK depth, 32FC1 in m (the robot: 16UC1 in mm) |
| `/front_camera/image_raw` (+ `/compressed`), `/front_camera/camera_info` | `front_camera_optical_frame` | CSI front camera, rgb8 1280×720, no distortion |

Each image has raw and exactly one compressed transport (`sim.launch.py` sets image_transport's `enable_pub_plugins`):
- `compressed` on the colour, mono and CSI images. The robot's OAK-D has the same three `/compressed` topics; its CSI camera publishes every installed image_transport plugin, so the sim leaves out `theora` and `compressedDepth` there.
- `compressedDepth` on depth: JPEG/PNG cannot carry the sim's 32FC1 (`/camera/stereo/depth/compressed` would log an error per frame). The robot has no compressed depth topic.

Not bridged: Gazebo's own depth point cloud. Its data are in the sensor link's x-forward convention while its header names the optical frame (pilsbot/pilsbot_simulation#4); point clouds come from `depth_image_proc` or `stereo_image_proc` on the topics above.

Stereo baseline: Fortress puts P[3] = −fx·0.0375 (half the OAK-D baseline) into the left, right and depth `camera_info`, and `stereo_image_proc` would compute half the depth. `parameter_bridge` puts those three on `/gz/camera/...`, and `scripts/camera_info_relay.py` (sim only) publishes them under the names above with left and depth P[3] = 0 and right P[3] = −fx·B, B from tf (0.075 m in the URDF). The right `camera_info` appears once tf is there.

The controller's odometry starts at the front pivot's position, not at `base_footprint`'s spawn pose (as on the robot).

Known limit of the sim: starting or stopping with the steering at 1 rad or more makes the wheels slip, and odometry then loses tens of degrees of yaw. Steady driving at those angles tracks.

## Tests

```
colcon test --packages-select pilsbot_simulation
```

The launch tests run the simulation headless and take up to a few minutes each. After each launch has ended, no process of the run may be left (the run has its own `IGN_PARTITION`).

- `test_sim_drive.launch.py` (`cameras:=false`): both controllers become active, a forward command moves `odom` forward, and a steering command turns `steering_axle_joint`.
- `test_sim_citysim.launch.py` (`world:=citysim_block cameras:=false`), **opt-in**: runs only with `PILSBOT_CITYSIM_TESTS=1`, since it needs citysim's Fuel models (downloaded on its first run). The robot stands on the street at its spawn point once the controllers are up, and drives 1 m along the road without leaving its lane. `colcon test --packages-select pilsbot_simulation --ctest-args -L citysim` with `PILSBOT_CITYSIM_TESTS=1` runs it alone.
- `test_sim_cameras.launch.py`, run twice: with `cameras:=true` every image topic, its compressed transport and its `camera_info` publish in the camera's optical frame, the stereo pair carries the baseline, and neither Gazebo's point cloud nor depth `/compressed` exists; with `cameras:=false` no camera topic exists.

They skip themselves when Gazebo Fortress, `ros_gz` (`ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`) or `gz_ros2_control` is missing, or with `PILSBOT_SKIP_GAZEBO_TESTS=1`; `colcon test --packages-select pilsbot_simulation --ctest-args -LE gazebo` leaves them out.

Mapping (rtabmap) is `pilsbot_navigation`'s `mapping.launch.py` (pilsbot/pilsbot_navigation#2, not written yet); the old `rtabmap_test.launch.py` is gone.
