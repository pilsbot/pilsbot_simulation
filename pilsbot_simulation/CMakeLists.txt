cmake_minimum_required(VERSION 3.8)
project(pilsbot_simulation)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # No license is chosen for this package yet (package.xml), so there is no
  # copyright notice to check. Remove this line once there is one.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_sim_launch test/test_sim_launch.py)
  ament_add_pytest_test(test_bridge_config test/test_bridge_config.py)

  find_package(ament_cmake_ros REQUIRED)
  # In-process ROS nodes: a ROS_DOMAIN_ID of its own.
  ament_add_ros_isolated_pytest_test(test_camera_info_relay test/test_camera_info_relay.py)

  # Headless Gazebo runs; they skip themselves where Gazebo is missing. Label
  # "gazebo": `colcon test --ctest-args -LE gazebo` leaves them out.
  find_package(launch_testing_ament_cmake REQUIRED)
  # RUNNER picks a free ROS_DOMAIN_ID. It goes before LABELS, which would
  # swallow it (add_launch_test does not know RUNNER).
  add_launch_test(test/test_sim_drive.launch.py
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
    TIMEOUT 900
    LABELS "launch_test;gazebo")
  # Two launches: cameras:=true and cameras:=false.
  add_launch_test(test/test_sim_cameras.launch.py
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
    TIMEOUT 900
    LABELS "launch_test;gazebo")
endif()

install(DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
