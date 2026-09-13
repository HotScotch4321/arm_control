#include "dynamixel_node/dynamixel_servos.hpp"

#include <algorithm>
#include <cstdlib>
#include <string>

#include "dynamixel_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace dynamixel_node
{

DynamixelServos::~DynamixelServos() = default;

// ---------------------------------------------------------------------------
// on_init: parse URDF hardware parameters, joints, and transmissions.
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn DynamixelServos::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & /*params*/)
{
  // Hardware-level parameters (device, baud_rate).
  auto hw = info_.hardware_parameters.find("device");
  if (hw != info_.hardware_parameters.end()) {
    device_ = hw->second;
  }
  auto baud = info_.hardware_parameters.find("baud_rate");
  if (baud != info_.hardware_parameters.end()) {
    baud_rate_ = std::atoi(baud->second.c_str());
  }

  auto ret = parseJoints();
  if (ret != hardware_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  ret = parseTransmissions();
  if (ret != hardware_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// parseJoints: extract name, servo_id, prismatic_scale from info_.joints.
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn DynamixelServos::parseJoints()
{
  joints_.reserve(info_.joints.size());
  for (const auto & joint_info : info_.joints) {
    Joint j;
    j.name = joint_info.name;

    auto servo = joint_info.parameters.find("servo_id");
    if (servo == joint_info.parameters.end()) {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' is missing required 'servo_id' parameter",
        j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    j.servo_id = std::atoi(servo->second.c_str());

    auto scale = joint_info.parameters.find("prismatic_scale");
    if (scale != joint_info.parameters.end()) {
      j.prismatic_scale = std::atof(scale->second.c_str());
      if (j.prismatic_scale != 1.0) {
        j.is_prismatic = true;
      }
    }

    joints_.push_back(std::move(j));
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// parseTransmissions: SimpleTransmission and DifferentialTransmission.
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn DynamixelServos::parseTransmissions()
{
  for (const auto & trans : info_.transmissions) {
    // SimpleTransmission: one joint, one actuator.
    if (trans.type == "transmission_interface/SimpleTransmission") {
      if (trans.joints.size() != 1) {
        RCLCPP_ERROR(
          get_logger(), "SimpleTransmission '%s' must have exactly one joint",
          trans.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      const auto & j = trans.joints[0];
      auto it = std::find_if(
        joints_.begin(), joints_.end(),
        [&](const Joint & joint) { return joint.name == j.name; });
      if (it == joints_.end()) {
        RCLCPP_ERROR(
          get_logger(),
          "SimpleTransmission '%s' references unknown joint '%s'",
          trans.name.c_str(), j.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      it->has_transmission = true;
      it->joint_reduction = j.mechanical_reduction;
      it->joint_offset = j.offset;
      if (!trans.actuators.empty()) {
        it->actuator_reduction = trans.actuators[0].mechanical_reduction;
      }
    }
    // DifferentialTransmission: two joints, two actuators.
    else if (trans.type == "transmission_interface/DifferentialTransmission") {
      if (trans.joints.size() != 2 || trans.actuators.size() != 2) {
        RCLCPP_ERROR(
          get_logger(),
          "DifferentialTransmission '%s' must have exactly two joints and two actuators",
          trans.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      DifferentialGroup diff;

      // Map joints by role: joint1 -> index, joint2 -> index.
      int j1 = -1, j2 = -1;
      for (const auto & tj : trans.joints) {
        auto it = std::find_if(
          joints_.begin(), joints_.end(),
          [&](const Joint & joint) { return joint.name == tj.name; });
        if (it == joints_.end()) {
          RCLCPP_ERROR(
            get_logger(),
            "DifferentialTransmission '%s' references unknown joint '%s'",
            trans.name.c_str(), tj.name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
        int idx = static_cast<int>(std::distance(joints_.begin(), it));
        it->in_differential = true;
        if (tj.role == "joint1") {
          j1 = idx;
          diff.jr[0] = tj.mechanical_reduction;
          diff.off[0] = tj.offset;
        } else if (tj.role == "joint2") {
          j2 = idx;
          diff.jr[1] = tj.mechanical_reduction;
          diff.off[1] = tj.offset;
        }
      }
      if (j1 < 0 || j2 < 0) {
        RCLCPP_ERROR(
          get_logger(),
          "DifferentialTransmission '%s' is missing joint1 or joint2 role",
          trans.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      diff.joint1_index = j1;
      diff.joint2_index = j2;

      // Map actuators by role: actuator1 -> joint1's servo, actuator2 -> joint2's servo.
      for (const auto & ta : trans.actuators) {
        if (ta.role == "actuator1") {
          diff.ar[0] = ta.mechanical_reduction;
          diff.actuator1_servo_id = joints_[j1].servo_id;
        } else if (ta.role == "actuator2") {
          diff.ar[1] = ta.mechanical_reduction;
          diff.actuator2_servo_id = joints_[j2].servo_id;
        }
      }

      differentials_.push_back(std::move(diff));
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Forward / inverse transforms for single joints (SimpleTransmission or none).
// ---------------------------------------------------------------------------
double DynamixelServos::forwardTransform(const Joint & joint, double actuator_pos) const
{
  // SimpleTransmission forward: j = a / jr + offset
  double joint_pos = joint.has_transmission
    ? actuator_pos / joint.joint_reduction + joint.joint_offset
    : actuator_pos;
  if (joint.is_prismatic) {
    joint_pos *= joint.prismatic_scale;
  }
  return joint_pos;
}

double DynamixelServos::inverseTransform(const Joint & joint, double joint_pos) const
{
  // Prismatic conversion first: meters -> radians in joint space.
  if (joint.is_prismatic) {
    joint_pos /= joint.prismatic_scale;
  }
  // SimpleTransmission inverse: a = (j - offset) * jr
  return joint.has_transmission
    ? (joint_pos - joint.joint_offset) * joint.joint_reduction
    : joint_pos;
}

// ---------------------------------------------------------------------------
// on_configure: open the bus, scan, read initial positions.
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn DynamixelServos::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  controller_ = std::make_unique<DynamixelController>(device_, baud_rate_);

  auto scan_result = controller_->scan();
  if (!scan_result.isSuccess()) {
    RCLCPP_ERROR(
      get_logger(), "Dynamixel scan failed: %s",
      dynamixel::getErrorMessage(scan_result.error()).c_str());
    controller_.reset();
    return hardware_interface::CallbackReturn::FAILURE;
  }

  const auto & found = scan_result.value();
  for (const auto & joint : joints_) {
    if (std::find(found.begin(), found.end(),
        static_cast<uint8_t>(joint.servo_id)) == found.end())
    {
      RCLCPP_ERROR(
        get_logger(), "Servo ID %d (joint '%s') was not found on the bus",
        joint.servo_id, joint.name.c_str());
      controller_.reset();
      return hardware_interface::CallbackReturn::FAILURE;
    }
  }

  // Read current positions and set initial state interface values.
  auto pos_result = controller_->readPositions();
  if (!pos_result.isSuccess()) {
    RCLCPP_ERROR(
      get_logger(), "Initial position read failed: %s",
      dynamixel::getErrorMessage(pos_result.error()).c_str());
    controller_.reset();
    return hardware_interface::CallbackReturn::FAILURE;
  }
  const auto & positions = pos_result.value();

  for (const auto & diff : differentials_) {
    double a1 = positions.at(static_cast<uint8_t>(diff.actuator1_servo_id));
    double a2 = positions.at(static_cast<uint8_t>(diff.actuator2_servo_id));
    double j1 = (a1 / diff.ar[0] + a2 / diff.ar[1]) / (2.0 * diff.jr[0]) + diff.off[0];
    double j2 = (a1 / diff.ar[0] - a2 / diff.ar[1]) / (2.0 * diff.jr[1]) + diff.off[1];
    set_state(joints_[diff.joint1_index].name + "/position", j1);
    set_state(joints_[diff.joint1_index].name + "/velocity", 0.0);
    set_state(joints_[diff.joint2_index].name + "/position", j2);
    set_state(joints_[diff.joint2_index].name + "/velocity", 0.0);
  }

  for (const auto & joint : joints_) {
    if (joint.in_differential) {
      continue;
    }
    double actuator_pos = positions.at(static_cast<uint8_t>(joint.servo_id));
    double joint_pos = forwardTransform(joint, actuator_pos);
    set_state(joint.name + "/position", joint_pos);
    set_state(joint.name + "/velocity", 0.0);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_activate: enable torque on all servos.
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn DynamixelServos::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (const auto & joint : joints_) {
    auto result = controller_->enableTorque(static_cast<uint8_t>(joint.servo_id));
    if (!result.isSuccess()) {
      RCLCPP_ERROR(
        get_logger(), "Failed to enable torque on servo %d: %s",
        joint.servo_id, dynamixel::getErrorMessage(result.error()).c_str());
      return hardware_interface::CallbackReturn::FAILURE;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_deactivate: disable torque on all servos.
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn DynamixelServos::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (const auto & joint : joints_) {
    auto result = controller_->disableTorque(static_cast<uint8_t>(joint.servo_id));
    if (!result.isSuccess()) {
      RCLCPP_WARN(
        get_logger(), "Failed to disable torque on servo %d: %s",
        joint.servo_id, dynamixel::getErrorMessage(result.error()).c_str());
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// read: fetch present positions, apply forward transmissions, set states.
// ---------------------------------------------------------------------------
hardware_interface::return_type DynamixelServos::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto pos_result = controller_->readPositions();
  if (!pos_result.isSuccess()) {
    RCLCPP_WARN(
      get_logger(), "readPositions failed: %s",
      dynamixel::getErrorMessage(pos_result.error()).c_str());
    return hardware_interface::return_type::OK;
  }
  const auto & positions = pos_result.value();

  // Differential joints.
  for (const auto & diff : differentials_) {
    double a1 = positions.at(static_cast<uint8_t>(diff.actuator1_servo_id));
    double a2 = positions.at(static_cast<uint8_t>(diff.actuator2_servo_id));
    double j1 = (a1 / diff.ar[0] + a2 / diff.ar[1]) / (2.0 * diff.jr[0]) + diff.off[0];
    double j2 = (a1 / diff.ar[0] - a2 / diff.ar[1]) / (2.0 * diff.jr[1]) + diff.off[1];
    set_state(joints_[diff.joint1_index].name + "/position", j1);
    set_state(joints_[diff.joint1_index].name + "/velocity", 0.0);
    set_state(joints_[diff.joint2_index].name + "/position", j2);
    set_state(joints_[diff.joint2_index].name + "/velocity", 0.0);
  }

  // Simple-transmission and direct joints.
  for (const auto & joint : joints_) {
    if (joint.in_differential) {
      continue;
    }
    auto it = positions.find(static_cast<uint8_t>(joint.servo_id));
    if (it == positions.end()) {
      continue;
    }
    double joint_pos = forwardTransform(joint, it->second);
    set_state(joint.name + "/position", joint_pos);
    set_state(joint.name + "/velocity", 0.0);
  }

  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// write: get commands, apply inverse transmissions, send to servos.
// ---------------------------------------------------------------------------
hardware_interface::return_type DynamixelServos::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::unordered_map<uint8_t, double> targets;

  // Simple-transmission and direct joints.
  for (const auto & joint : joints_) {
    if (joint.in_differential) {
      continue;
    }
    double cmd = get_command(joint.name + "/position");
    double actuator_pos = inverseTransform(joint, cmd);
    targets[static_cast<uint8_t>(joint.servo_id)] = actuator_pos;
  }

  // Differential joints.
  for (const auto & diff : differentials_) {
    double j1 = get_command(joints_[diff.joint1_index].name + "/position");
    double j2 = get_command(joints_[diff.joint2_index].name + "/position");
    double a1 = ((j1 - diff.off[0]) * diff.jr[0] + (j2 - diff.off[1]) * diff.jr[1]) * diff.ar[0];
    double a2 = ((j1 - diff.off[0]) * diff.jr[0] - (j2 - diff.off[1]) * diff.jr[1]) * diff.ar[1];
    targets[static_cast<uint8_t>(diff.actuator1_servo_id)] = a1;
    targets[static_cast<uint8_t>(diff.actuator2_servo_id)] = a2;
  }

  if (!targets.empty()) {
    auto result = controller_->setTargetPosition(targets);
    if (!result.isSuccess()) {
      RCLCPP_WARN(
        get_logger(), "setTargetPosition failed: %s",
        dynamixel::getErrorMessage(result.error()).c_str());
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace dynamixel_node

PLUGINLIB_EXPORT_CLASS(
  dynamixel_node::DynamixelServos, hardware_interface::SystemInterface)
