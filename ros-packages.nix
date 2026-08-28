# Single source of truth for the ROS packages in this workspace.
#
# Imported by both flake.nix and .moveit-shell.nix so that everything ends up in
# ONE ros.buildEnv. Splitting ROS packages across two buildEnvs puts two
# separate prefixes on AMENT_PREFIX_PATH, each holding a partial closure, and
# ament's find_package() then fails to resolve deps across them
# ("Could not find a package configuration file provided by ament_cmake_core").
ros:
with ros; [
  ros-core

  # ament build tooling, needed to colcon-build C++ packages in this workspace.
  ament-cmake
  ament-cmake-core
  ament-cmake-auto

  # Visualization / description
  rviz2
  robot-state-publisher
  joint-state-publisher
  joint-state-publisher-gui
  xacro

  # ros2_control. NOTE: deliberately not the `ros2-control` / `ros2-controllers`
  # metapackages, and not `moveit` -- those drag in chomp/stomp/pilz/benchmarks/
  # setup-assistant and every steering/gpio/admittance controller, none of which
  # this workspace loads and none of which are cached (~44 source builds).
  controller-manager
  hardware-interface
  joint-trajectory-controller
  joint-state-broadcaster

  # MoveIt runtime pieces the launch files load by plugin name.
  moveit-kinematics
  moveit-msgs
  moveit-planners-ompl
  moveit-ros-control-interface
  moveit-ros-move-group
  moveit-ros-visualization
  moveit-servo
  moveit-simple-controller-manager

  # Dynamixel bus (dynamixel_node build + runtime deps).
  dynamixel-sdk
  dynamixel-sdk-custom-interfaces

  # Common messages / utilities
  sensor-msgs
  geometry-msgs
  trajectory-msgs
  control-msgs
  std-srvs
  tf2
  tf2-ros

  # .envrc pins RMW_IMPLEMENTATION to this.
  rmw-fastrtps-cpp
]
