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

ament_package()
