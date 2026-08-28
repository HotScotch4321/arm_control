// Minimal ROS 2 interface to Dynamixel MX-106 servos on one U2D2 bus.
//
// Interfaces are the ones shipped with the SDK (dynamixel_sdk_custom_interfaces)
// so the ROBOTIS example commands work against this node unchanged:
//
//   ros2 topic pub -1 /dynamixel_node/set_position \
//       dynamixel_sdk_custom_interfaces/msg/SetPosition "{id: 1, position: 2048}"
//   ros2 service call /dynamixel_node/get_position \
//       dynamixel_sdk_custom_interfaces/srv/GetPosition "{id: 1}"
//   ros2 service call /dynamixel_node/set_torque std_srvs/srv/SetBool "{data: true}"

#include <cmath>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "dynamixel_easy_sdk/dynamixel_easy_sdk.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace
{
using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;
using SetBool = std_srvs::srv::SetBool;
using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;

// MX-106 in position control mode: 4096 ticks per turn, 2048 is the centre.
constexpr int32_t k_position_min = 0;
constexpr int32_t k_position_max = 4095;
constexpr int32_t k_position_centre = 2048;
constexpr double k_radians_per_tick = 2.0 * M_PI / 4096.0;

double toRadians(int32_t position)
{
  return (position - k_position_centre) * k_radians_per_tick;
}
}  // namespace

class DynamixelNode : public rclcpp::Node
{
public:
  DynamixelNode()
  : Node("dynamixel_node")
  {
    const auto port = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    const auto baud_rate = declare_parameter<int>("baud_rate", 57600);
    const auto ids = declare_parameter<std::vector<int64_t>>("ids", std::vector<int64_t>{});
    const auto publish_rate = declare_parameter<double>("publish_rate", 10.0);
    const auto torque_on_start = declare_parameter<bool>("enable_torque_on_start", false);
    disable_torque_on_shutdown_ = declare_parameter<bool>("disable_torque_on_shutdown", false);

    // Throws DxlRuntimeError if the port cannot be opened; main() reports it.
    connector_ = std::make_unique<dynamixel::Connector>(port, baud_rate);
    RCLCPP_INFO(get_logger(), "Opened %s at %d baud (protocol 2.0)", port.c_str(), baud_rate);

    discover(ids);

    if (torque_on_start) {
      const auto result = setTorque(0, true);
      RCLCPP_INFO(get_logger(), "%s", result.second.c_str());
    }

    set_position_subscriber_ = create_subscription<SetPosition>(
      "~/set_position", 10,
      [this](const SetPosition::SharedPtr msg) {onSetPosition(*msg);});

    get_position_service_ = create_service<GetPosition>(
      "~/get_position",
      [this](
        const std::shared_ptr<GetPosition::Request> request,
        std::shared_ptr<GetPosition::Response> response) {onGetPosition(*request, *response);});

    set_torque_service_ = create_service<SetBool>(
      "~/set_torque",
      [this](
        const std::shared_ptr<SetBool::Request> request,
        std::shared_ptr<SetBool::Response> response) {
        const auto result = setTorque(0, request->data);
        response->success = result.first;
        response->message = result.second;
      });

    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);

    if (publish_rate > 0.0) {
      const auto period = std::chrono::duration<double>(1.0 / publish_rate);
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this] {publishJointStates();});
    }

    RCLCPP_INFO(get_logger(), "Ready. Torque is %s.", torque_on_start ? "on" : "off");
  }

  ~DynamixelNode() override
  {
    if (!disable_torque_on_shutdown_ || !connector_) {
      return;
    }
    // Off by default: releasing torque lets an unbalanced arm drop.
    setTorque(0, false);
  }

private:
  // Populates motors_ either from an explicit ID list or from a broadcast ping.
  void discover(const std::vector<int64_t> & ids)
  {
    std::vector<uint8_t> found;

    if (ids.empty()) {
      auto scan = connector_->broadcastPing();
      if (!scan.isSuccess()) {
        throw dynamixel::DxlRuntimeError(
                "Bus scan failed: " + dynamixel::getErrorMessage(scan.error()));
      }
      found = scan.value();
    } else {
      for (int64_t id : ids) {
        if (id < 1 || id > 252) {
          throw dynamixel::DxlRuntimeError("Invalid servo ID: " + std::to_string(id));
        }
        found.push_back(static_cast<uint8_t>(id));
      }
    }

    if (found.empty()) {
      throw dynamixel::DxlRuntimeError("No servos answered on the bus.");
    }

    for (uint8_t id : found) {
      // createMotor() pings and throws if the servo does not answer.
      motors_.emplace(id, connector_->createMotor(id));
      auto & motor = *motors_.at(id);
      RCLCPP_INFO(
        get_logger(), "Servo ID %u: %s (model %u)",
        id, motor.getModelName().c_str(), motor.getModelNumber());

      auto mode = motor.getOperatingMode();
      if (mode.isSuccess() && mode.value() != dynamixel::OperatingMode::POSITION) {
        RCLCPP_WARN(
          get_logger(),
          "Servo ID %u is not in position control mode; goal positions will be refused.", id);
      }
    }
  }

  // An ID of 0 means "every servo found at startup".
  std::vector<uint8_t> resolve(uint8_t id) const
  {
    std::vector<uint8_t> targets;
    if (id == 0) {
      for (const auto & [motor_id, motor] : motors_) {
        (void)motor;
        targets.push_back(motor_id);
      }
    } else if (motors_.count(id) != 0) {
      targets.push_back(id);
    }
    return targets;
  }

  void onSetPosition(const SetPosition & msg)
  {
    if (msg.position < k_position_min || msg.position > k_position_max) {
      RCLCPP_WARN(
        get_logger(), "Position %d is outside 0-%d.", msg.position, k_position_max);
      return;
    }

    const auto targets = resolve(msg.id);
    if (targets.empty()) {
      RCLCPP_WARN(get_logger(), "Unknown servo ID %u.", msg.id);
      return;
    }

    std::lock_guard<std::mutex> lock(bus_mutex_);
    auto executor = connector_->createGroupExecutor();
    for (uint8_t id : targets) {
      auto command = motors_.at(id)->stageSetGoalPosition(msg.position);
      if (!command.isSuccess()) {
        RCLCPP_ERROR(
          get_logger(), "Servo ID %u: %s (torque enabled?)",
          id, dynamixel::getErrorMessage(command.error()).c_str());
        return;
      }
      executor->addCmd(command.value());
    }

    auto result = executor->executeWrite();
    if (!result.isSuccess()) {
      RCLCPP_ERROR(
        get_logger(), "Move failed: %s", dynamixel::getErrorMessage(result.error()).c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Set [ID %u] [Goal Position %d]", msg.id, msg.position);
  }

  void onGetPosition(const GetPosition::Request & request, GetPosition::Response & response)
  {
    if (motors_.count(request.id) == 0) {
      RCLCPP_WARN(get_logger(), "Unknown servo ID %u.", request.id);
      response.position = -1;
      return;
    }

    std::lock_guard<std::mutex> lock(bus_mutex_);
    auto position = motors_.at(request.id)->getPresentPosition();
    if (!position.isSuccess()) {
      RCLCPP_ERROR(
        get_logger(), "Read failed for ID %u: %s",
        request.id, dynamixel::getErrorMessage(position.error()).c_str());
      response.position = -1;
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Get [ID %u] [Present Position %d] [%.3f rad]",
      request.id, position.value(), toRadians(position.value()));
    response.position = position.value();
  }

  std::pair<bool, std::string> setTorque(uint8_t id, bool enable)
  {
    const auto targets = resolve(id);
    if (targets.empty()) {
      return {false, "Unknown servo ID " + std::to_string(id) + "."};
    }

    std::lock_guard<std::mutex> lock(bus_mutex_);
    for (uint8_t target : targets) {
      auto & motor = *motors_.at(target);
      auto result = enable ? motor.enableTorque() : motor.disableTorque();
      if (!result.isSuccess()) {
        return {false,
          "Servo ID " + std::to_string(target) + ": " +
          dynamixel::getErrorMessage(result.error())};
      }
    }
    return {true,
      std::string("Torque ") + (enable ? "enabled" : "disabled") + " on " +
      std::to_string(targets.size()) + " servo(s)."};
  }

  // One sync read for the whole bus, published as radians for RViz/MoveIt.
  void publishJointStates()
  {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    auto executor = connector_->createGroupExecutor();
    std::vector<uint8_t> ids;

    for (const auto & [id, motor] : motors_) {
      auto command = motor->stageGetPresentPosition();
      if (!command.isSuccess()) {
        warnThrottled("Read failed: " + dynamixel::getErrorMessage(command.error()));
        return;
      }
      ids.push_back(id);
      executor->addCmd(command.value());
    }

    auto values = executor->executeRead();
    if (!values.isSuccess()) {
      warnThrottled("Read failed: " + dynamixel::getErrorMessage(values.error()));
      return;
    }

    sensor_msgs::msg::JointState state;
    state.header.stamp = now();
    for (std::size_t index = 0; index < ids.size(); ++index) {
      auto & value = values.value()[index];
      if (!value.isSuccess()) {
        warnThrottled(
          "Read failed for ID " + std::to_string(ids[index]) + ": " +
          dynamixel::getErrorMessage(value.error()));
        return;
      }
      state.name.push_back("dxl_" + std::to_string(ids[index]));
      state.position.push_back(toRadians(value.value()));
    }
    joint_state_publisher_->publish(state);
  }

  void warnThrottled(const std::string & message)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", message.c_str());
  }

  bool disable_torque_on_shutdown_{false};
  std::mutex bus_mutex_;  // The bus is one wire: never two transactions at once.
  std::unique_ptr<dynamixel::Connector> connector_;
  std::map<uint8_t, std::unique_ptr<dynamixel::Motor>> motors_;

  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_service_;
  rclcpp::Service<SetBool>::SharedPtr set_torque_service_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DynamixelNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("dynamixel_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
