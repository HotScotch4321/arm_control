#ifndef DYNAMIXEL_NODE__DYNAMIXEL_SYSTEM_HPP_
#define DYNAMIXEL_NODE__DYNAMIXEL_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "dynamixel_node/dynamixel_controller.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "transmission_interface/transmission.hpp"

namespace dynamixel_node
{

class DynamixelSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Joint
  {
    std::string name;
    std::string transmission;
    uint8_t id{0};
    double offset{0.0};
    double direction{1.0};
    bool online{false};
    double position{0.0};
    double velocity{0.0};
  };

  // Written by the control loop, read by the diagnostics timer. 
  struct JointStatus
  {
    std::atomic<double> position{0.0};
    std::atomic<double> velocity{0.0};
    std::atomic<bool> online{false};
  };

  struct TransmissionGroup
  {
    std::string name;
    std::shared_ptr<transmission_interface::Transmission> impl;
    std::vector<std::size_t> joint_index;
    std::vector<uint8_t> servo_id;
    std::vector<double> actuator_position;
    std::vector<double> actuator_velocity;
    std::vector<double> joint_position;
    std::vector<double> joint_velocity;
  };

  bool bringUp(const Joint & joint);
  hardware_interface::CallbackReturn loadTransmissions();
  void startDiagnostics();
  void publishDiagnostics();

  std::unique_ptr<DynamixelController> controller_;
  std::vector<Joint> joints_;
  std::vector<std::unique_ptr<TransmissionGroup>> transmissions_;
  std::unordered_map<uint8_t, double> targets_;

  std::vector<std::unique_ptr<JointStatus>> status_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::atomic<uint64_t> read_errors_{0};
  std::atomic<uint64_t> write_errors_{0};
  std::atomic<bool> bus_ok_{false};

  std::string port_;
  int baud_rate_{1000000};
  bool bus_open_{false};
};

}  // namespace dynamixel_node

#endif  // DYNAMIXEL_NODE__DYNAMIXEL_SYSTEM_HPP_