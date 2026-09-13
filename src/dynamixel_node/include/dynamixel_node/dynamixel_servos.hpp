#ifndef DYNAMIXEL_NODE__DYNAMIXEL_SERVOS_HPP_
#define DYNAMIXEL_NODE__DYNAMIXEL_SERVOS_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

class DynamixelController;

namespace dynamixel_node
{

/// ros2_control SystemInterface plugin for the Dynamixel servo bus.
class DynamixelServos : public hardware_interface::SystemInterface
{
public:
  DynamixelServos() = default;
  ~DynamixelServos() override;

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
    double joint_reduction = 1.0;
    double joint_offset = 0.0;
    double actuator_reduction = 1.0;
    bool has_transmission = false;
    bool in_differential = false;
    bool is_prismatic = false;
    double prismatic_scale = 1.0;
  };

  /// A DifferentialTransmission linking two joints to two servo actuators.
  struct DifferentialGroup
  {
    int actuator1_servo_id = 0;
    int actuator2_servo_id = 0;
    double ar[2] = {1.0, 1.0};
    double jr[2] = {1.0, 1.0};
    double off[2] = {0.0, 0.0};
    int joint1_index = 0;
    int joint2_index = 0;
  };

  hardware_interface::CallbackReturn parseJoints();
  hardware_interface::CallbackReturn parseTransmissions();

  /// Apply forward transmission (actuator -> joint) for a single joint.
  double forwardTransform(const Joint & joint, double actuator_pos) const;

  /// Apply inverse transmission (joint -> actuator) for a single joint.
  double inverseTransform(const Joint & joint, double joint_pos) const;

  std::unique_ptr<DynamixelController> controller_;
  std::vector<Joint> joints_;
  std::vector<DifferentialGroup> differentials_;
  std::string device_ = "/dev/ttyUSB0";
  int baud_rate_ = 57600;
};

}  // namespace dynamixel_node

#endif  // DYNAMIXEL_NODE__DYNAMIXEL_SERVOS_HPP_
