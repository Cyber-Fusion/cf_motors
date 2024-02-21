#pragma once

#include <bridges/can/types.hpp>

#include <mutex>
#include <shared_mutex>

#include <protocol/can/msg.hpp>

namespace cf_motors {
namespace protocol {

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

  ::cf_motors::bridges::CanMsg
  NewWriteCurrentMultiTurnPositionEncoderToROMAsMotorZeroCommand(
      const uint32_t id) const;

  uint32_t AsWriteCurrentMultiTurnPositionEncoderToROMAsMotorZeroCommand(
      const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg
  NewReadMultiTurnAngleCommand(const uint32_t id) const;
  uint32_t AsReadMultiTurnAngleCommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  /* Motor status reding commands. --------------------------------- */

  ::cf_motors::bridges::CanMsg
  NewReadMotorStatus1AndErrorFlagCommand(const uint32_t id) const;
  motor_status_1 AsReadMotorStatus1AndErrorFlagCommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg
  NewReadMotorStatus2Command(const uint32_t id) const;
  motor_status_2 AsReadMotorStatus2CommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg
  NewReadMotorStatus3Command(const uint32_t id) const;
  motor_status_3 AsReadMotorStatus3CommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg
  NewMotorStopCommand(const uint32_t id, const bool need_shutdown) const;
  void AsMotorStopCommandResponse(const ::cf_motors::bridges::CanMsg &msg,
                                  const bool need_shutdown) const;

  ::cf_motors::bridges::CanMsg
  NewTorqueClosedLoopControlCommand(const uint32_t id,
                                    const int16_t torque) const;
  torque_set_response AsTorqueClosedLoopControlCommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg
  NewSpeedLoopControlCommand(const uint32_t id,
                             const uint32_t speed_control) const;
  torque_set_response AsSpeedLoopControlCommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  /* Position tracking control commands. --------------------------------- */
  ::cf_motors::bridges::CanMsg
  NewPositionTrackingControlCommand(const uint32_t id,
                                    const uint32_t angle) const;
  torque_set_response AsPositionTrackingControlCommand(
      const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg NewAbsolutePositionClosedLoopControlCommand(
      const uint32_t id, const int32_t angle, const uint16_t speed) const;
  torque_set_response AsAbsolutePositionClosedLoopControlCommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg NewPositionTrackingCommandWithSpeedLimit(
      const uint32_t id, const int32_t angle, const uint16_t speed) const;
  torque_set_response AsPositionTrackingCommandWithSpeedLimitResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg NewIncrementalPositionClosedLoopCommand(
      const uint32_t id, const int32_t angle, const int16_t speed) const;
  torque_set_response AsIncrementalPositionClosedLoopCommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  /* System commands. --------------------------------- */
  ::cf_motors::bridges::CanMsg
  NewSystemOperatingModeAcquisition(const uint32_t id) const;
  uint8_t AsSystemOperatingModeAcquisitionResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg
  NewMotorPowerAcquisitionCommand(const uint32_t id) const;
  uint16_t AsMotorPowerAcquisitionCommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg
  NewSystemResetControlCommand(const uint32_t id) const;
  void AsSystemResetControlCommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg
  NewSystemBreakReleaseCommand(const uint32_t id) const;
  void AsSystemBreakReleaseCommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg NewSystemBreakLockCommand(const uint32_t) const;
  void AsSystemBreakLockCommand(const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg
  NewSystemRuntimeReadCommand(const uint32_t id) const;
  uint32_t AsSystemRuntimeReadCommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg
  NewSystemSoftwareVersionDateReadCommand(const uint32_t id) const;
  uint32_t AsSystemSoftwareVersionDateReadCommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg
  NewCommunicationInterruptProtectionTimeSettingCommand(
      const uint32_t id, const uint32_t time) const;
  uint32_t AsCommunicationInterruptProtectionTimeSettingCommand(
      const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg
  NewCommunicationBaudRateSettingCommand(const uint32_t id,
                                         const uint8_t baud_rate) const;
  void AsCommunicationBaudRateSettingCommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg
  NewMotorModelReadingCommand(const uint32_t id) const;
  void
  AsMotorModelReadingCommand(const ::cf_motors::bridges::CanMsg &msg) const;

  ::cf_motors::bridges::CanMsg
  NewFunctionControlCommand(const uint32_t id, const uint8_t index,
                            const uint32_t value) const;
  uint8_t AsFunctionControlCommandResponse(
      const ::cf_motors::bridges::CanMsg &msg) const;

private:
  ::std::array<double, 6>
  parse_pid_parameters(const ::std::array<uint8_t, 8> &data) const;
  ::cf_motors::bridges::CanMsg
  create_message_with_command_code(const uint32_t id, const uint8_t cid) const;

  void validate_message_command_id(const ::cf_motors::bridges::CanMsg &msg,
                                   const uint8_t cid) const;

  motor_status_2
  parse_motor_status_2_message(const ::cf_motors::bridges::CanMsg &msg) const;

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

} // namespace protocol
} // namespace cf_motors
