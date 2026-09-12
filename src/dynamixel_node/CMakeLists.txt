cmake_minimum_required(VERSION 3.16)
project(dynamixel_node LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(transmission_interface REQUIRED)

install(DIRECTORY include/ DESTINATION include)

# Uncomment once src/dynamixel_servos.cpp exists.
#
# add_library(dynamixel_servos SHARED src/dynamixel_servos.cpp)
# target_compile_features(dynamixel_servos PUBLIC cxx_std_17)
# target_include_directories(dynamixel_servos PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_link_libraries(dynamixel_servos PUBLIC
#   dynamixel_sdk::dynamixel_sdk
#   hardware_interface::hardware_interface
#   pluginlib::pluginlib
#   rclcpp::rclcpp
#   rclcpp_lifecycle::rclcpp_lifecycle
#   transmission_interface::transmission_interface)
#
# pluginlib_export_plugin_description_file(hardware_interface dynamixel_servos_plugin.xml)
#
# install(TARGETS dynamixel_servos
#   EXPORT export_dynamixel_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)
#
# ament_export_targets(export_dynamixel_node HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_dependencies(
  dynamixel_sdk hardware_interface pluginlib rclcpp rclcpp_lifecycle transmission_interface)

ament_package()
