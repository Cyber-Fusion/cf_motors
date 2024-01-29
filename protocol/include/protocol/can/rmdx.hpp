#pragma once

#include <bridges/can/types.hpp>

#include <mutex>
#include <shared_mutex>

namespace cf_motors {
namespace rmdx {

enum acceleration_functions {
  position_planning_acceleration = 0x00,
  position_planning_deceleration = 0x01,
  speed_planning_acceleration = 0x02,
  speed_planning_deceleration = 0x03
};

class RMDX {

public:
  /* PID commands.  -------------------------------*/

  ::cf_motors::bridges::CanMsg NewReadPIDMessage(const uint32_t id) const;
  ::std::array<double, 6>
  AsReadPIDResponse(const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg
  NewWritePIDParametersMessage(const uint32_t id,
                               const ::std::array<double, 6> &params,
                               bool toRAM = true) const;
  ::std::array<double, 6>
  AsWritePIDResponse(const ::cf_motors::bridges::CanMsg &msg,
                     bool toRAM = true) const;

  /* Acceleration commands.  ------------------------------- */

  ::cf_motors::bridges::CanMsg
  NewReadAccelerationCommand(const uint32_t id) const;
  uint32_t
  AsReadAccelerationResponse(const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg
  NewWriteAccelerationCommand(const uint32_t id, const uint32_t acceleration,
                              const acceleration_functions af) const;
  uint32_t AsWriteAccelerationCommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  /* Multi-turn encoder commands. --------------------------------- */

  ::cf_motors::bridges::CanMsg
  NewReadMultiTurnEncoderCommand(const uint32_t id) const;
  uint32_t AsReadMultiTurnEncoderCommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg
  NewReadMultiTurnEncoderOriginalPositionCommand(const uint32_t id) const;
  uint32_t AsReadMultiTurnEncoderOriginalPositionCommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg
  NewReadMultiTurnEncoderZeroOffsetCommand(const uint32_t id) const;
  uint32_t AsReadMultiTurnEncoderZeroOffsetCommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg
  NewWriteMultiTurnValueToROMAsMotorZeroCommand(const uint32_t id,
                                                const uint32_t value) const;
  uint32_t AsWriteMultiTurnValueToROMAsMotorZeroCommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

private:
  ::std::array<double, 6>
  parse_pid_parameters(const ::std::array<uint8_t, 8> &data) const;
  uint32_t parse_last_4_bytes(const ::std::array<uint8_t, 8> &data) const;
  void place_parameter_in_last_4_bytes(::std::array<uint8_t, 8> &data,
                                       uint32_t parameter) const;

private:
  double maximum_value_of_current_loop_;
  double maximum_value_of_current_loop_ki_;
  double maximum_value_of_speed_loop_;
  double maximum_value_of_speed_loop_ki_;
  double maximum_value_of_position_loop_;
  double maximum_value_of_position_loop_ki_;

private:
  using mutex_type = ::std::shared_timed_mutex;
  using read_lock = ::std::shared_lock<mutex_type>;
  using write_lock = ::std::unique_lock<mutex_type>;

  mutable mutex_type guard_;
};

} // namespace rmdx
} // namespace cf_motors
