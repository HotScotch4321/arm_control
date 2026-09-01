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
find_package(diagnostic_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(transmission_interface REQUIRED)

add_library(dynamixel_system SHARED src/dynamixel_controller.cpp src/dynamixel_system.cpp)

target_include_directories(
  dynamixel_system
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  dynamixel_system
  PUBLIC dynamixel_sdk::dynamixel_sdk
         hardware_interface::hardware_interface
         pluginlib::pluginlib
         rclcpp::rclcpp
         rclcpp_lifecycle::rclcpp_lifecycle
         transmission_interface::transmission_interface
         ${diagnostic_msgs_TARGETS}
         ${std_srvs_TARGETS})

pluginlib_export_plugin_description_file(hardware_interface
                                         dynamixel_hardware_plugin.xml)

install(
  TARGETS dynamixel_system
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(diagnostic_msgs dynamixel_sdk hardware_interface pluginlib rclcpp
                          rclcpp_lifecycle std_srvs transmission_interface)

ament_package()
