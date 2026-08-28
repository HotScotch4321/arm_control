cmake_minimum_required(VERSION 3.16)

project(
  arm_moveit_config
  VERSION 0.0.1
  LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

# This package has no compiled targets: it is MoveIt configuration, launch files
# and a Python teleop script. Hardware drivers live outside it on purpose.

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY src/moveit DESTINATION share/${PROJECT_NAME}/src)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS scripts/keyboard_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_control)

install(
  PROGRAMS scripts/gripper_velocity_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gripper_velocity_bridge)

install(
  PROGRAMS scripts/wrist_velocity_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wrist_velocity_bridge)

ament_package()
