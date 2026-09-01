// Bus-level driver for Dynamixel servos on one U2D2, protocol 2.0.
// Ported from proj/dynamixal-controller-poc.

#ifndef DYNAMIXEL_NODE__DYNAMIXEL_CONTROLLER_HPP_
#define DYNAMIXEL_NODE__DYNAMIXEL_CONTROLLER_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "dynamixel_easy_sdk/dynamixel_easy_sdk.hpp"

namespace dynamixel_node
{

template<typename T>
using Result = dynamixel::Result<T, dynamixel::DxlError>;

class DynamixelController
{
public:
  using Mode = dynamixel::OperatingMode;
  // modes:
  // CURRENT, VELOCITY, POSITION, EXTENDED_POSITION, CURRENT_BASED_POSITION, PWM

  static constexpr float protocol_version = 2.0F;
  static constexpr double tau = 6.28318530717958647692;
  static constexpr int32_t counts_per_turn = 4096;
  // Extended Position Control Mode spans +-256 turns.
  static constexpr int32_t position_min = -1048575;
  static constexpr int32_t position_max = 1048575;
  // "Present Velocity" unit from the MX-106 control table, in rad/s.
  static constexpr double velocity_unit = 0.0239691227;

  static double unitsToRadians(int32_t units);
  static Result<int32_t> radiansToUnits(double radians);

  // Throws dynamixel::DxlRuntimeError if the port cannot be opened. Nothing
  // else here throws: this runs inside the ros2_control loop.
  DynamixelController(const std::string & device, int baud_rate);
  ~DynamixelController() noexcept;

  DynamixelController(const DynamixelController &) = delete;
  DynamixelController & operator=(const DynamixelController &) = delete;

  Result<std::vector<uint8_t>> scan();
  Result<void> addServo(uint8_t id);
  bool has(uint8_t id) const;

  Result<void> enableTorque(uint8_t id);
  Result<void> disableTorque(uint8_t id);
  Result<void> setMode(uint8_t id, Mode mode);
  Result<Mode> mode(uint8_t id);

  Result<void> setTargetPosition(uint8_t id, double radians);
  Result<void> setTargetPosition(const std::unordered_map<uint8_t, double> & radians);
  Result<double> readPosition(uint8_t id);
  Result<std::unordered_map<uint8_t, double>> readPositions();
  Result<std::unordered_map<uint8_t, double>> readVelocities();
  std::unordered_map<uint8_t, double> memPositions() const;

private:
  dynamixel::Motor * servo(uint8_t id);
  Result<void> setTargetPositionsLocked(const std::unordered_map<uint8_t, double> & radians);
  Result<std::unordered_map<uint8_t, double>> readGroupLocked(bool velocity);

  std::mutex bus_mutex_;
  mutable std::mutex positions_mutex_;
  dynamixel::Connector connector_;
  std::unordered_map<uint8_t, std::unique_ptr<dynamixel::Motor>> servos_;
  std::unordered_map<uint8_t, double> positions_{};
};

}  // namespace dynamixel_node

#endif  // DYNAMIXEL_NODE__DYNAMIXEL_CONTROLLER_HPP_
