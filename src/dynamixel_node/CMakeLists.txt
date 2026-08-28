cmake_minimum_required(VERSION 3.16)

project(
  dynamixel_node
  VERSION 0.0.1
  LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(dynamixel_node src/dynamixel_node.cpp)

target_link_libraries(
  dynamixel_node
  PRIVATE dynamixel_sdk::dynamixel_sdk
          rclcpp::rclcpp
          ${dynamixel_sdk_custom_interfaces_TARGETS}
          ${sensor_msgs_TARGETS}
          ${std_srvs_TARGETS})

install(TARGETS dynamixel_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
