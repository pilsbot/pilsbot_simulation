# pilsbot_simulation

Pilsbot in **Gazebo Fortress**, driven by the robot's own stack. `sim.launch.py` starts `gz sim` with a world, spawns the robot from `robot_description`, bridges `/clock` and the cameras to ROS, and starts `pilsbot_bringup`'s `pilsbot.launch.py` with `hardware:=gazebo use_sim_time:=true`. That is the same launch code as on the robot: robot_state_publisher, the spawners (`--inactive`) and the hardware activator, teleop and foxglove_bridge. The controller manager runs inside Gazebo's `gz_ros2_control` plugin and loads the real `acker_diff_controller.yaml` with the robot profile's geometry. The steering pivot is free, so the robot steers passively by the difference in front wheel speeds, as on the robot. GNSS and lighting are not started.

Gazebo Classic is not supported.

## Requirements

Gazebo Fortress with `ros-humble-ros-gz` and `ros-humble-gz-ros2-control` (apt), and the Humble Pilsbot stack (`pilsbot_bringup`, `pilsbot_hw_ros`, `pils_control`, `pilsbot_description`, `teleop_acker_joy`). `colcon build --packages-up-to pilsbot_simulation` builds the Pilsbot packages it needs, `pilsbot_driver` included.

## Run

```
ros2 launch pilsbot_simulation sim.launch.py              # with the Gazebo GUI
ros2 launch pilsbot_simulation sim.launch.py gui:=false   # server only, cameras render headless
```

Stop it with Ctrl-C, or with a signal to the `ros2 launch` process alone (`kill <pid>`, systemd): Gazebo runs as `exec`, so it gets launch's signal and stops with the rest.

The controllers become active a few seconds of sim time after the launch (the hardware activator's period). The sim usually runs below real time with the cameras on, so this takes longer on the wall clock.

| argument | default | |
|---|---|---|
| `world` | `worlds/flat.sdf` | SDF world file |
| `gui` | `true` | `false`: no GUI, `gz sim -s --headless-rendering` |
| `robot_profile` | `pilsbot` | profile name shipped with `pilsbot_description`, or a path |
| `x`, `y`, `z`, `yaw` | `0`, `0`, `0.02`, `0` | spawn pose [m, rad] |

Arguments of `pilsbot_bringup` and the driver launch files pass through, e.g. `foxglove:=false` or `joystick_device_name:=...`.

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

The same as on the robot: `odom` → `base_footprint` from the controller, `/joint_states`, `/tf` from robot_state_publisher. `config/bridge.yaml` bridges Gazebo to ROS only:

| ROS topic | |
|---|---|
| `/clock` | sim time |
| `/camera/color/image_raw`, `/camera/color/camera_info` | OAK colour, rgb8 1280×720 |
| `/camera/left/image_rect`, `/camera/right/image_rect` + `camera_info` | OAK mono, mono8 640×400 |
| `/camera/stereo/depth`, `/camera/stereo/camera_info` | OAK depth, 32FC1 in m (the robot: 16UC1 in mm) |
| `/camera/stereo/points` | point cloud, sim only |
| `/image_raw`, `/camera_info` | CSI front camera |

The controller's odometry starts at the front pivot's position, not at `base_footprint`'s spawn pose (as on the robot).

Known limit of the sim: starting or stopping with the steering at 1 rad or more makes the wheels slip, and odometry then loses tens of degrees of yaw. Steady driving at those angles tracks.

## Tests

```
colcon test --packages-select pilsbot_simulation
```

`test_sim_drive.launch.py` runs the simulation headless: both controllers become active, a forward command moves `odom` forward, and a steering command turns `steering_axle_joint`. After the launch has ended, no process of the run may be left (the run has its own `IGN_PARTITION`). It takes a few minutes. It skips itself when Gazebo Fortress, `ros_gz` or `gz_ros2_control` is missing, or with `PILSBOT_SKIP_GAZEBO_TESTS=1`; `colcon test --packages-select pilsbot_simulation --ctest-args -LE gazebo` leaves it out.

## rtabmap_test.launch.py

Left over from Foxy and not yet adapted to the Fortress topics; milestone 2 (perception) fixes it.
