#ifndef DYNAMIXEL_NODE__DYNAMIXEL_SERVOS_HPP_
#define DYNAMIXEL_NODE__DYNAMIXEL_SERVOS_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace dynamixel_node
{

/// Placeholder Dynamixel bus plugin: echoes commands back as states.
class DynamixelServos : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// One <joint> of the ros2_control tag, in URDF order.
  struct Joint
  {
    std::string name;
    int servo_id = 0;
    std::string transmission;
    double mechanical_reduction = 1.0;
    double offset = 0.0;
  };

  hardware_interface::CallbackReturn parseJoints();
  hardware_interface::CallbackReturn parseTransmissions();

  std::vector<Joint> joints_;
};

}  // namespace dynamixel_node

#endif  // DYNAMIXEL_NODE__DYNAMIXEL_SERVOS_HPP_
