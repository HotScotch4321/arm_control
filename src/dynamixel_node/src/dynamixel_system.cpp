// ros2_control system interface for Dynamixel servos on one U2D2 bus.
//
// Joints run in Extended Position (multi-turn) mode. Any servo that does not
// answer runs "offline": its state echoes its command, so a missing (or
// entirely absent) bus degrades to mock behaviour instead of refusing to
// start. Warnings say exactly which joints are affected.
//
// Joints named by a <transmission> in the ros2_control block are driven
// through it: each such joint's <param name="id"> is the servo wired to the
// actuator in the matching slot.

#include "dynamixel_node/dynamixel_system.hpp"

#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "transmission_interface/differential_transmission_loader.hpp"
#include "transmission_interface/handle.hpp"
#include "transmission_interface/simple_transmission_loader.hpp"

namespace dynamixel_node
{

hardware_interface::CallbackReturn DynamixelSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & hardware = get_hardware_info().hardware_parameters;
  port_ = hardware.count("port") ? hardware.at("port") : "/dev/ttyUSB0";
  baud_rate_ = hardware.count("baud_rate") ? std::stoi(hardware.at("baud_rate")) : 1000000;

  for (const auto & info : get_hardware_info().joints) {
    Joint joint;
    joint.name = info.name;

    const auto id = info.parameters.find("id");
    if (id == info.parameters.end()) {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' has no <param name=\"id\">; add the servo ID to the ros2_control "
        "block.", info.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint.id = static_cast<uint8_t>(std::stoi(id->second));

    const auto offset = info.parameters.find("offset");
    if (offset != info.parameters.end()) {
      joint.offset = std::stod(offset->second);
    }
    const auto direction = info.parameters.find("direction");
    if (direction != info.parameters.end()) {
      joint.direction = std::stod(direction->second) < 0.0 ? -1.0 : 1.0;
    }

    // Where an offline joint sits until commanded, same as mock hardware.
    for (const auto & state : info.state_interfaces) {
      if (state.name == hardware_interface::HW_IF_POSITION && !state.initial_value.empty()) {
        joint.position = std::stod(state.initial_value);
      }
    }
    joints_.push_back(std::move(joint));
  }

  if (loadTransmissions() != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  targets_.reserve(joints_.size());
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    status_.push_back(std::make_unique<JointStatus>());
    status_.back()->position.store(joints_[index].position);
  }

  RCLCPP_INFO(
    get_logger(), "Configured %zu joints (%zu transmissions) on %s at %d baud.", joints_.size(),
    transmissions_.size(), port_.c_str(), baud_rate_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::loadTransmissions()
{
  transmission_interface::SimpleTransmissionLoader simple_loader;
  transmission_interface::DifferentialTransmissionLoader differential_loader;

  for (const auto & info : get_hardware_info().transmissions) {
    transmission_interface::TransmissionLoader * loader = nullptr;
    std::size_t slots = 0;
    if (info.type == "transmission_interface/SimpleTransmission") {
      loader = &simple_loader;
      slots = 1;
    } else if (info.type == "transmission_interface/DifferentialTransmission") {
      loader = &differential_loader;
      slots = 2;
    } else {
      RCLCPP_ERROR(
        get_logger(), "Transmission '%s' has type '%s'; only SimpleTransmission and "
        "DifferentialTransmission are supported.", info.name.c_str(), info.type.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (info.joints.size() != slots || info.actuators.size() != slots) {
      RCLCPP_ERROR(
        get_logger(), "Transmission '%s' needs %zu joints and %zu actuators, got %zu and %zu.",
        info.name.c_str(), slots, slots, info.joints.size(), info.actuators.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    auto group = std::make_unique<TransmissionGroup>();
    group->name = info.name;
    try {
      group->impl = loader->load(info);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Transmission '%s' failed to load: %s", info.name.c_str(), error.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!group->impl) {
      RCLCPP_ERROR(get_logger(), "Transmission '%s' failed to load.", info.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Sized before any handle is taken; the handles hold raw pointers into these.
    group->joint_index.resize(slots);
    group->servo_id.resize(slots);
    group->actuator_position.assign(slots, 0.0);
    group->actuator_velocity.assign(slots, 0.0);
    group->joint_position.assign(slots, 0.0);
    group->joint_velocity.assign(slots, 0.0);

    std::vector<transmission_interface::JointHandle> joint_handles;
    std::vector<transmission_interface::ActuatorHandle> actuator_handles;

    for (std::size_t slot = 0; slot < slots; ++slot) {
      const auto & joint_name = info.joints[slot].name;
      std::size_t index = joints_.size();
      for (std::size_t candidate = 0; candidate < joints_.size(); ++candidate) {
        if (joints_[candidate].name == joint_name) {
          index = candidate;
          break;
        }
      }
      if (index == joints_.size()) {
        RCLCPP_ERROR(
          get_logger(), "Transmission '%s' names joint '%s', which is not in the ros2_control "
          "block.", info.name.c_str(), joint_name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      joints_[index].transmission = info.name;
      group->joint_index[slot] = index;
      group->servo_id[slot] = joints_[index].id;

      joint_handles.emplace_back(
        joint_name, hardware_interface::HW_IF_POSITION, &group->joint_position[slot]);
      joint_handles.emplace_back(
        joint_name, hardware_interface::HW_IF_VELOCITY, &group->joint_velocity[slot]);
      actuator_handles.emplace_back(
        info.actuators[slot].name, hardware_interface::HW_IF_POSITION,
        &group->actuator_position[slot]);
      actuator_handles.emplace_back(
        info.actuators[slot].name, hardware_interface::HW_IF_VELOCITY,
        &group->actuator_velocity[slot]);
    }

    try {
      group->impl->configure(joint_handles, actuator_handles);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Transmission '%s' failed to configure: %s", info.name.c_str(), error.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::string mapping;
    for (std::size_t slot = 0; slot < slots; ++slot) {
      mapping += (slot ? ", " : "") + info.joints[slot].name + " -> id " +
        std::to_string(group->servo_id[slot]);
    }
    RCLCPP_INFO(
      get_logger(), "Transmission '%s' (%s): %s", info.name.c_str(), info.type.c_str(),
      mapping.c_str());
    transmissions_.push_back(std::move(group));
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

void DynamixelSystem::startDiagnostics()
{
  if (diagnostics_publisher_ || !get_node()) {
    return;
  }

  diagnostics_publisher_ = get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "~/diagnostics", rclcpp::QoS(1));
  diagnostics_timer_ = get_node()->create_wall_timer(
    std::chrono::seconds(1), [this] {publishDiagnostics();});

  RCLCPP_INFO(
    get_logger(), "Diagnostics publishing at %s/diagnostics.",
    get_node()->get_fully_qualified_name());
}

void DynamixelSystem::publishDiagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = get_clock()->now();

  auto add = [](diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key,
      const std::string & value) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = key;
      pair.value = value;
      status.values.push_back(pair);
    };

  std::size_t online = 0;
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    const auto & joint = joints_[index];
    const bool joint_online = status_[index]->online.load();
    online += joint_online ? 1 : 0;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "dynamixel: " + joint.name;
    status.hardware_id = "id " + std::to_string(joint.id);
    status.level = joint_online ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = joint_online ? "online" : "offline, echoing command";
    add(status, "position", std::to_string(status_[index]->position.load()));
    add(status, "velocity", std::to_string(status_[index]->velocity.load()));
    add(status, "servo_id", std::to_string(joint.id));
    if (!joint.transmission.empty()) {
      add(status, "transmission", joint.transmission);
    }
    message.status.push_back(std::move(status));
  }

  diagnostic_msgs::msg::DiagnosticStatus bus;
  bus.name = "dynamixel: bus";
  bus.hardware_id = port_;
  const bool open = bus_ok_.load();
  bus.level = !open ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
    online < joints_.size() ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
    diagnostic_msgs::msg::DiagnosticStatus::OK;
  bus.message = !open ? "port not open, whole arm echoing" :
    std::to_string(online) + " of " + std::to_string(joints_.size()) + " joints online";
  add(bus, "port", port_);
  add(bus, "baud_rate", std::to_string(baud_rate_));
  add(bus, "read_errors", std::to_string(read_errors_.load()));
  add(bus, "write_errors", std::to_string(write_errors_.load()));
  message.status.push_back(std::move(bus));

  diagnostics_publisher_->publish(message);
}

bool DynamixelSystem::bringUp(const Joint & joint)
{
  auto added = controller_->addServo(joint.id);
  if (!added.isSuccess()) {
    RCLCPP_ERROR(
      get_logger(), "Joint '%s' (ID %u) did not answer: %s. Echoing commands for this joint.",
      joint.name.c_str(), joint.id, dynamixel::getErrorMessage(added.error()).c_str());
    return false;
  }

  // turn on Multi-turn
  auto mode = controller_->setMode(joint.id, DynamixelController::Mode::EXTENDED_POSITION);
  if (!mode.isSuccess()) {
    RCLCPP_ERROR(
      get_logger(), "Joint '%s' (ID %u): extended position mode failed: %s. Echoing commands for "
      "this joint.", joint.name.c_str(), joint.id,
      dynamixel::getErrorMessage(mode.error()).c_str());
    return false;
  }

  auto torque = controller_->enableTorque(joint.id);
  if (!torque.isSuccess()) {
    RCLCPP_ERROR(
      get_logger(), "Joint '%s' (ID %u): torque enable failed: %s. Echoing commands for this "
      "joint.", joint.name.c_str(), joint.id,
      dynamixel::getErrorMessage(torque.error()).c_str());
    return false;
  }
  return true;
}

hardware_interface::CallbackReturn DynamixelSystem::on_activate(const rclcpp_lifecycle::State &)
{
  startDiagnostics();

  try {
    controller_ = std::make_unique<DynamixelController>(port_, baud_rate_);
    bus_open_ = true;
    RCLCPP_INFO(get_logger(), "Opened %s at %d baud (protocol 2.0).", port_.c_str(), baud_rate_);
  } catch (const std::exception & error) {
    bus_open_ = false;
    RCLCPP_ERROR(
      get_logger(), "Could not open %s (%s). Every joint will echo its command, so the arm behaves "
      "like mock hardware.", port_.c_str(), error.what());
  }

  for (auto & joint : joints_) {
    joint.online = false;
    joint.velocity = 0.0;

    if (!bus_open_ || !bringUp(joint)) {
      continue;
    }
    joint.online = true;

    if (!joint.transmission.empty()) {
      continue;
    }
    auto present = controller_->readPosition(joint.id);
    if (present.isSuccess()) {
      joint.position = joint.direction * present.value() + joint.offset;
    }
  }

  for (const auto & group : transmissions_) {
    const std::size_t slots = group->servo_id.size();
    bool all_online = true;
    for (std::size_t slot = 0; slot < slots; ++slot) {
      all_online = all_online && joints_[group->joint_index[slot]].online;
    }
    if (!all_online) {
      for (std::size_t slot = 0; slot < slots; ++slot) {
        joints_[group->joint_index[slot]].online = false;
      }
      continue;
    }

    bool read_ok = true;
    for (std::size_t slot = 0; slot < slots; ++slot) {
      auto present = controller_->readPosition(group->servo_id[slot]);
      read_ok = read_ok && present.isSuccess();
      group->actuator_position[slot] = present.isSuccess() ? present.value() : 0.0;
      group->actuator_velocity[slot] = 0.0;
    }
    if (!read_ok) {
      continue;
    }

    group->impl->actuator_to_joint();
    for (std::size_t slot = 0; slot < slots; ++slot) {
      auto & joint = joints_[group->joint_index[slot]];
      joint.position = joint.direction * group->joint_position[slot] + joint.offset;
    }
  }

  std::size_t online = 0;
  for (const auto & joint : joints_) {
    online += joint.online ? 1 : 0;
    if (joint.online) {
      RCLCPP_INFO(
        get_logger(), "Joint '%s' (ID %u) online at %.3f rad.", joint.name.c_str(), joint.id,
        joint.position);
    }
  }
  if (online == 0) {
    RCLCPP_WARN(get_logger(), "No servos answered; the whole arm is echoing commands.");
  } else if (online < joints_.size()) {
    RCLCPP_WARN(get_logger(), "%zu of %zu joints online.", online, joints_.size());
  }

  bus_ok_.store(bus_open_);
  read_errors_.store(0);
  write_errors_.store(0);
  // Always SUCCESS: a partly (or wholly) absent bus is reported, not fatal.
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & joint : joints_) {
    if (joint.online) {
      auto result = controller_->disableTorque(joint.id);
      if (!result.isSuccess()) {
        RCLCPP_WARN(
          get_logger(), "Joint '%s' (ID %u): torque disable failed: %s.", joint.name.c_str(),
          joint.id, dynamixel::getErrorMessage(result.error()).c_str());
      }
    }
    joint.online = false;
  }
  controller_.reset();
  bus_open_ = false;
  bus_ok_.store(false);
  for (auto & status : status_) {
    status->online.store(false);
    status->velocity.store(0.0);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DynamixelSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::unordered_map<uint8_t, double> positions;
  std::unordered_map<uint8_t, double> velocities;

  if (bus_open_) {
    auto read_positions = controller_->readPositions();
    if (read_positions.isSuccess()) {
      positions = read_positions.value();
      // Velocity is advisory: a failure here just reports zero.
      auto read_velocities = controller_->readVelocities();
      if (read_velocities.isSuccess()) {
        velocities = read_velocities.value();
      }
    } else {
      read_errors_.fetch_add(1);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Bus read failed: %s.",
        dynamixel::getErrorMessage(read_positions.error()).c_str());
    }
  }

  for (auto & joint : joints_) {
    if (!joint.transmission.empty()) {
      continue;
    }
    auto found = positions.find(joint.id);
    if (found != positions.end()) {
      joint.position = joint.direction * found->second + joint.offset;
      auto velocity = velocities.find(joint.id);
      joint.velocity = velocity == velocities.end() ? 0.0 : joint.direction * velocity->second;
      joint.online = true;
    } else if (joint.online) {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' (ID %u) went offline. Echoing commands for this joint; the rest "
        "of the arm keeps running.", joint.name.c_str(), joint.id);
      joint.online = false;
    }
  }

  for (const auto & group : transmissions_) {
    const std::size_t slots = group->servo_id.size();

    bool have_position = true;
    bool have_velocity = true;
    for (std::size_t slot = 0; slot < slots; ++slot) {
      auto position = positions.find(group->servo_id[slot]);
      auto velocity = velocities.find(group->servo_id[slot]);
      have_position = have_position && position != positions.end();
      have_velocity = have_velocity && velocity != velocities.end();
      group->actuator_position[slot] = position == positions.end() ? 0.0 : position->second;
      group->actuator_velocity[slot] = velocity == velocities.end() ? 0.0 : velocity->second;
    }

    if (!have_position) {
      bool was_online = false;
      for (std::size_t slot = 0; slot < slots; ++slot) {
        auto & joint = joints_[group->joint_index[slot]];
        was_online = was_online || joint.online;
        joint.online = false;
      }
      if (was_online) {
        RCLCPP_ERROR(
          get_logger(), "Transmission '%s' went offline. Echoing commands for its joints.",
          group->name.c_str());
      }
      continue;
    }

    group->impl->actuator_to_joint();

    for (std::size_t slot = 0; slot < slots; ++slot) {
      auto & joint = joints_[group->joint_index[slot]];
      joint.position = joint.direction * group->joint_position[slot] + joint.offset;
      joint.velocity = have_velocity ? joint.direction * group->joint_velocity[slot] : 0.0;
      joint.online = true;
    }
  }

  for (std::size_t index = 0; index < joints_.size(); ++index) {
    auto & joint = joints_[index];
    if (!joint.online) {
      // Offline: follow the command so the controller sees its goal reached.
      const double command =
        get_command<double>(joint.name + "/" + hardware_interface::HW_IF_POSITION);
      if (std::isfinite(command)) {
        joint.position = command;
      }
      joint.velocity = 0.0;
    }
    set_state(joint.name + "/" + hardware_interface::HW_IF_POSITION, joint.position);
    set_state(joint.name + "/" + hardware_interface::HW_IF_VELOCITY, joint.velocity);

    status_[index]->position.store(joint.position);
    status_[index]->velocity.store(joint.velocity);
    status_[index]->online.store(joint.online);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!bus_open_) {
    return hardware_interface::return_type::OK;
  }

  targets_.clear();

  for (const auto & joint : joints_) {
    if (!joint.transmission.empty() || !joint.online) {
      continue;
    }
    const double command =
      get_command<double>(joint.name + "/" + hardware_interface::HW_IF_POSITION);
    if (!std::isfinite(command)) {
      continue;
    }
    targets_[joint.id] = (command - joint.offset) * joint.direction;
  }

  for (const auto & group : transmissions_) {
    const std::size_t slots = group->servo_id.size();

    // Every joint of a group must be online and commanded before any of its
    // servo targets is meaningful.
    bool ready = true;
    for (std::size_t slot = 0; slot < slots && ready; ++slot) {
      const auto & joint = joints_[group->joint_index[slot]];
      const double command =
        get_command<double>(joint.name + "/" + hardware_interface::HW_IF_POSITION);
      ready = joint.online && std::isfinite(command);
      group->joint_position[slot] = (command - joint.offset) * joint.direction;
    }
    if (!ready) {
      continue;
    }

    group->impl->joint_to_actuator();
    for (std::size_t slot = 0; slot < slots; ++slot) {
      targets_[group->servo_id[slot]] = group->actuator_position[slot];
    }
  }

  auto result = controller_->setTargetPosition(targets_);
  if (!result.isSuccess()) {
    write_errors_.fetch_add(1);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Bus write failed: %s.",
      dynamixel::getErrorMessage(result.error()).c_str());
  }
  return hardware_interface::return_type::OK;
}

}  // namespace dynamixel_node

PLUGINLIB_EXPORT_CLASS(dynamixel_node::DynamixelSystem, hardware_interface::SystemInterface)