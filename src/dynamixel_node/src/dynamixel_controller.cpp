#include "dynamixel_node/dynamixel_controller.hpp"

#include <cmath>

namespace dynamixel_node
{

double DynamixelController::unitsToRadians(int32_t units)
{
  return static_cast<double>(units) * tau / counts_per_turn;
}

Result<int32_t> DynamixelController::radiansToUnits(double radians)
{
  const double units = radians * counts_per_turn / tau;
  if (!std::isfinite(units) || units < position_min || units > position_max) {
    return dynamixel::DxlError::SDK_ERRNUM_DATA_RANGE;
  }
  return static_cast<int32_t>(std::llround(units));
}

DynamixelController::DynamixelController(const std::string & device, int baud_rate)
: connector_(device, baud_rate)
{
}

DynamixelController::~DynamixelController() noexcept
{
  // Torque is left as-is: releasing it drops an unbalanced arm.
  servos_.clear();
}

dynamixel::Motor * DynamixelController::servo(uint8_t id)
{
  auto found = servos_.find(id);
  return found == servos_.end() ? nullptr : found->second.get();
}

bool DynamixelController::has(uint8_t id) const
{
  return servos_.find(id) != servos_.end();
}

Result<void> DynamixelController::addServo(uint8_t id)
{
  if (has(id)) {
    return {};
  }
  std::lock_guard<std::mutex> lock(bus_mutex_);
  try {
    // createMotor() pings and throws if the servo does not answer.
    servos_.emplace(id, connector_.createMotor(id));
  } catch (const std::exception &) {
    return dynamixel::DxlError::SDK_COMM_RX_TIMEOUT;
  }
  return {};
}

Result<std::vector<uint8_t>> DynamixelController::scan()
{
  std::vector<uint8_t> found;
  {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    auto result = connector_.broadcastPing();
    if (!result.isSuccess()) {
      return result.error();
    }
    found = result.value();
  }
  for (uint8_t id : found) {
    auto added = addServo(id);
    if (!added.isSuccess()) {
      return added.error();
    }
  }
  return found;
}

Result<void> DynamixelController::enableTorque(uint8_t id)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  auto * motor = servo(id);
  if (motor == nullptr) {
    return dynamixel::DxlError::EASY_SDK_FAIL_TO_GET_DATA;
  }
  return motor->enableTorque();
}

Result<void> DynamixelController::disableTorque(uint8_t id)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  auto * motor = servo(id);
  if (motor == nullptr) {
    return dynamixel::DxlError::EASY_SDK_FAIL_TO_GET_DATA;
  }
  return motor->disableTorque();
}

Result<void> DynamixelController::setMode(uint8_t id, Mode mode)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  auto * motor = servo(id);
  if (motor == nullptr) {
    return dynamixel::DxlError::EASY_SDK_FAIL_TO_GET_DATA;
  }
  // Operating Mode is EEPROM; the servo refuses the write while torque is on.
  auto torque_off = motor->disableTorque();
  if (!torque_off.isSuccess()) {
    return torque_off.error();
  }
  return motor->setOperatingMode(mode);
}

Result<DynamixelController::Mode> DynamixelController::mode(uint8_t id)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  auto * motor = servo(id);
  if (motor == nullptr) {
    return dynamixel::DxlError::EASY_SDK_FAIL_TO_GET_DATA;
  }
  return motor->getOperatingMode();
}

Result<void> DynamixelController::setTargetPosition(uint8_t id, double radians)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  auto * motor = servo(id);
  if (motor == nullptr) {
    return dynamixel::DxlError::EASY_SDK_FAIL_TO_GET_DATA;
  }
  auto units = radiansToUnits(radians);
  if (!units.isSuccess()) {
    return units.error();
  }
  return motor->setGoalPosition(units.value());
}

Result<void> DynamixelController::setTargetPosition(
  const std::unordered_map<uint8_t, double> & radians)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  return setTargetPositionsLocked(radians);
}

Result<void> DynamixelController::setTargetPositionsLocked(
  const std::unordered_map<uint8_t, double> & radians)
{
  if (radians.empty()) {
    return {};
  }
  auto executor = connector_.createGroupExecutor();
  for (const auto & [id, target] : radians) {
    auto * motor = servo(id);
    if (motor == nullptr) {
      return dynamixel::DxlError::EASY_SDK_FAIL_TO_GET_DATA;
    }
    auto units = radiansToUnits(target);
    if (!units.isSuccess()) {
      return units.error();
    }
    auto command = motor->stageSetGoalPosition(units.value());
    if (!command.isSuccess()) {
      return command.error();
    }
    executor->addCmd(command.value());
  }
  // One sync write for the whole bus: a single packet, no status packets back.
  return executor->executeWrite();
}

Result<double> DynamixelController::readPosition(uint8_t id)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  auto * motor = servo(id);
  if (motor == nullptr) {
    return dynamixel::DxlError::EASY_SDK_FAIL_TO_GET_DATA;
  }
  auto result = motor->getPresentPosition();
  if (!result.isSuccess()) {
    return result.error();
  }

  const double radians = unitsToRadians(result.value());
  {
    std::lock_guard<std::mutex> positions_lock(positions_mutex_);
    positions_[id] = radians;
  }
  return radians;
}

Result<std::unordered_map<uint8_t, double>> DynamixelController::readGroupLocked(bool velocity)
{
  auto executor = connector_.createGroupExecutor();
  std::vector<uint8_t> ids;
  ids.reserve(servos_.size());

  for (auto & [id, motor] : servos_) {
    auto command = velocity ?
      motor->stageGetPresentVelocity() :
      motor->stageGetPresentPosition();
    if (!command.isSuccess()) {
      return command.error();
    }
    ids.push_back(id);
    executor->addCmd(command.value());
  }

  auto values = executor->executeRead();
  if (!values.isSuccess()) {
    return values.error();
  }
  if (values.value().size() != ids.size()) {
    return dynamixel::DxlError::EASY_SDK_FAIL_TO_GET_DATA;
  }

  // Servos that did not answer are simply absent from the map.
  std::unordered_map<uint8_t, double> out;
  for (std::size_t index = 0; index < ids.size(); ++index) {
    auto & value = values.value()[index];
    if (!value.isSuccess()) {
      continue;
    }
    out.emplace(
      ids[index],
      velocity ? value.value() * velocity_unit : unitsToRadians(value.value()));
  }
  return out;
}

Result<std::unordered_map<uint8_t, double>> DynamixelController::readPositions()
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  auto positions = readGroupLocked(false);
  if (!positions.isSuccess()) {
    return positions.error();
  }
  {
    std::lock_guard<std::mutex> positions_lock(positions_mutex_);
    positions_ = positions.value();
  }
  return positions;
}

Result<std::unordered_map<uint8_t, double>> DynamixelController::readVelocities()
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  return readGroupLocked(true);
}

std::unordered_map<uint8_t, double> DynamixelController::memPositions() const
{
  std::lock_guard<std::mutex> lock(positions_mutex_);
  return positions_;
}

}  // namespace dynamixel_node
