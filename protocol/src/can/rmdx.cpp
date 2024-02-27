#include <protocol/can/rmdx.hpp>

#include <cassert>

namespace cf_motors {

/*
 Single motor commands.
*/

// PID parameters.
#define READ_PID_PARAMETER_COMMAND 0x30
#define WRITE_PID_PARAMETERS_TO_RAM_COMMAND 0x31
#define WRITE_PID_PARAMETERS_TO_ROM_COMMAND 0x32
#define READ_ACCELERATION_COMMAND 0x42
#define WRITE_ACCELERATION_COMMAND_TO_RAM 0x43
#define READ_MULTI_TURN_ENCODER_POSITION_DATA_COMMAND 0x60
#define READ_MULTI_TURN_ENCODER_ORIGINAL_POSITION_DATA_COMMAND 0x61
#define READ_MULTI_TURN_ENCODER_ZERO_OFFSET_DATA_COMMAND 0x62
#define WRITE_ENCODER_MULTI_TURN_VALUE_TO_ROM_AS_MOTOR_ZERO_COMMAND 0x63
#define WRITE_MULTI_TURN_POSITION_OF_THE_ENCODER_TO_THE_ROM_AS_ZERO_COMMAND 0x64
#define READ_MULTI_TURN_ANGLE_COMMAND 0x92
#define READ_MOTOR_STATUS_1_AND_ERROR_FLAG_COMMAND 0x9A
#define READ_MOTOR_STATUS_2_COMMAND 0x9C
#define READ_MOTOR_STATUS_3_COMMAND 0x9D
#define MOTOR_SHUTDOWN_COMMAND 0x80
#define MOTOR_STOP_COMMAND 0x81
#define TORQUE_CLOSED_LOOP_COMMAND 0xA1
#define SPEED_CLOSED_LOOP_CONTROL_COMMAND 0xA2
#define POSITION_TRACKING_CONTROL_COMMAND 0xA3
#define ABSOLUTE_POSITION_CLOSED_LOOP_CONTROL_COMMAND 0xA4
#define POSITION_TRACKING_CONTROL_COMMAND_WITH_SPEED_LIMIT_COMMAND 0xA5
#define INCREMENTAL_POSITION_CLOSED_LOOP_CONTROL_COMMAND 0xA8
#define SYSTEM_OPERATING_MODE_ACQUISITION_COMMAND 0x70
#define MOTOR_POWER_ACQUISITION_COMMAND 0x71
#define SYSTEM_RESET_COMMAND 0x76
#define SYSTEM_BREAK_RELEASE_COMMAND 0x77
#define SYSTEM_BREAK_LOCK_COMMAND 0x78
#define SYSTEM_RUNTIME_READ_COMMAND 0xB1
#define SYSTEM_SOFTWARE_VERSION_READ_COMMAND 0xB2
#define COMMUNICATION_INTERRUPTION_PROTECTION_TIME_SETTING_COMMAND 0xB3
#define COMMUNICATION_BAUD_RATE_SETTING_COMMAND 0xB4
#define MOTOR_MODEL_READING_COMMAND 0xB5
#define FUNCTION_CONTROL_COMMAND 0x20

namespace protocol {

namespace detail {

template <typename T>
concept PrimitiveType = std::is_arithmetic_v<T>;

template <PrimitiveType T>
T parse_value(const ::cf_motors::bridges::CanMsg &msg, const size_t from,
              const size_t to) {
  assert(sizeof(T) == (to - from));
  assert((to > from) && (to < 8));

  T res;
  for (size_t i = from; i <= to; ++i) {
    res = (res << 8) | msg.data[i];
  }
  return res;
}

template <PrimitiveType T>
void place_value(::cf_motors::bridges::CanMsg &msg, const T value,
                 const size_t from, const size_t to) {
  size_t vsize = sizeof(T) - 1;
  assert(vsize == (to - from));
  assert((to > from) && (to < 8));

  for (size_t i = from; i <= to; ++i) {
    msg.data[i] = static_cast<uint8_t>((value >> (vsize-- * 8)) & 0xFF);
  }
}
} // namespace detail

/*
  This command can read the parameters of current, speed, position loop KP and
  KI at one time, and the data type is uint8_t. The system sets the maximum
  range of PI parameters according to the motor model, and then divides it
  equally according to the maximum range of uint8_t of 256 units. Users only
  need to adjust 0-256 units.
*/
::cf_motors::bridges::CanMsg RMDX::NewReadPIDMessage(const uint32_t id) const {
  return create_message_with_command_code(id, READ_PID_PARAMETER_COMMAND);
}

::std::array<double, 6>
RMDX::AsReadPIDResponse(const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg, READ_PID_PARAMETER_COMMAND);
  return parse_pid_parameters(msg.data);
}

/*
  This command can write the parameters of current, speed, position loop KP and
  KI to RAM at one time, and it will not be saved after power off. The data type
  is uint8_t. The system sets the maximum range of PI parameters according to
  the motor model, and then divides it equally according to the maximum range of
  uint8_t of 256 units. Users only need to adjust 0-256 units.
*/
::cf_motors::bridges::CanMsg
RMDX::NewWritePIDParametersMessage(const uint32_t id,
                                   const ::std::array<double, 6> &params,
                                   bool toRAM) const {
  ::cf_motors::bridges::CanMsg msg;
  if (toRAM) {
    msg = create_message_with_command_code(id,
                                           WRITE_PID_PARAMETERS_TO_RAM_COMMAND);
  } else {
    msg = create_message_with_command_code(id,
                                           WRITE_PID_PARAMETERS_TO_ROM_COMMAND);
  }

  msg.data[1] = 0x00;
  ::std::copy(params.begin(), params.end(), msg.data.begin() + 2);
  return msg;
}

::std::array<double, 6>
RMDX::AsWritePIDResponse(const ::cf_motors::bridges::CanMsg &msg,
                         bool toRAM) const {
  if (toRAM) {
    validate_message_command_id(msg, WRITE_PID_PARAMETERS_TO_RAM_COMMAND);
  } else {
    validate_message_command_id(msg, WRITE_PID_PARAMETERS_TO_ROM_COMMAND);
  }

  return this->parse_pid_parameters(msg.data);
}

/*
  The host sends this command to read the acceleration parameters of the current
  motor
*/
::cf_motors::bridges::CanMsg
RMDX::NewReadAccelerationCommand(const uint32_t id) const {
  return create_message_with_command_code(id, READ_ACCELERATION_COMMAND);
}

/*
  The acceleration parameter is included in the drive response data.
  Acceleration data Accel is int32_t type, the unit is 1dps/s, and the parameter
  range is 50-60000.
*/
uint32_t RMDX::AsReadAccelerationResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg, READ_ACCELERATION_COMMAND);
  return detail::parse_value<uint32_t>(msg, 4, 7);
}

/*
  The host sends this command to write the acceleration and deceleration into
  RAM and ROM, which can be saved after power off. Acceleration data Accel is of
  uint32_t type, the unit is 1dps/s, and the parameter range is 100-60000. The
  command contains the acceleration and deceleration values in the position and
  velocity planning, which are determined by the index value.
*/
::cf_motors::bridges::CanMsg
RMDX::NewWriteAccelerationCommand(const uint32_t id,
                                  const uint32_t acceleration,
                                  const acceleration_functions af) const {
  // Convert the uint32_t value to a std::array<uint8_t, 8>
  auto msg =
      create_message_with_command_code(id, WRITE_ACCELERATION_COMMAND_TO_RAM);
  msg.data[1] = af;
  detail::place_value<uint32_t>(msg, acceleration, 4, 7);
  return msg;
}

/*
  The motor will reply to the host after receiving the command, and the reply
  command is the same as the received command.
*/
uint32_t RMDX::AsWriteAccelerationCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg, WRITE_ACCELERATION_COMMAND_TO_RAM);
  return detail::parse_value<uint32_t>(msg, 4, 7);
}

/*
  The host sends this command to read the multi-turn position of the encoder,
  which represents the rotation angle of the motor output shaft, including the
  multi-turn angle.
*/
::cf_motors::bridges::CanMsg
RMDX::NewReadMultiTurnEncoderCommand(const uint32_t id) const {
  return create_message_with_command_code(
      id, READ_MULTI_TURN_ENCODER_POSITION_DATA_COMMAND);
}

/*
  The motor replies to the host after receiving the command, and the frame data
  contains the following parameters. Encoder multi-turn position encoder
  (int32_t type, value range of multi-turn encoder, 4 bytes of valid data),
  which is the value after subtracting the encoder's multi-turn zero offset
  (initial position) from the original position of the encoder.
*/
uint32_t RMDX::AsReadMultiTurnEncoderCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg,
                              READ_MULTI_TURN_ENCODER_POSITION_DATA_COMMAND);
  return detail::parse_value<uint32_t>(msg, 4, 7);
}

/*
  The host sends this command to read the multi-turn encoder home position, ie
  the multi-turn encoder value without the zero offset (home position).
*/
::cf_motors::bridges::CanMsg
RMDX::NewReadMultiTurnEncoderOriginalPositionCommand(const uint32_t id) const {
  return create_message_with_command_code(
      id, READ_MULTI_TURN_ENCODER_ORIGINAL_POSITION_DATA_COMMAND);
}

/*
  The motor replies to the host after receiving the command, and the frame data
  contains the following parameters. Encoder multi-turn raw position encoderRaw
  (int32_t type, value range, valid data 4 bytes).
*/
uint32_t RMDX::AsReadMultiTurnEncoderOriginalPositionCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(
      msg, READ_MULTI_TURN_ENCODER_ORIGINAL_POSITION_DATA_COMMAND);
  return detail::parse_value<uint32_t>(msg, 4, 7);
}

/*
  The host sends this command to read the multi-turn zero offset value (initial
  position) of the encoder.
*/
::cf_motors::bridges::CanMsg
RMDX::NewReadMultiTurnEncoderZeroOffsetCommand(const uint32_t id) const {
  return create_message_with_command_code(
      id, READ_MULTI_TURN_ENCODER_ZERO_OFFSET_DATA_COMMAND);
}

/*
  The motor replies to the host after receiving the command, and the frame data
  contains the following parameters. Encoder multi-turn zero offset
  encoderOffset (int32_t type, value range, valid data 4 bytes).
*/
uint32_t RMDX::AsReadMultiTurnEncoderZeroOffsetCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg,
                              READ_MULTI_TURN_ENCODER_ZERO_OFFSET_DATA_COMMAND);
  return detail::parse_value<uint32_t>(msg, 4, 7);
}

/*
  The host sends this command to set the zero offset (initial position) of the
  encoder, where the encoder multi-turn value to be written, encoderOffset, is
  of type int32_t, (value range, 4 bytes of valid data). Note: After writing the
  position of the new zero point, the motor needs to be restarted to be
  effective. Because of the change of the zero offset, the new zero offset
  (initial position) should be used as a reference when setting the target
  position.
*/
::cf_motors::bridges::CanMsg
RMDX::NewWriteMultiTurnValueToROMAsMotorZeroCommand(
    const uint32_t id, const uint32_t value) const {
  auto msg = create_message_with_command_code(
      id, WRITE_ENCODER_MULTI_TURN_VALUE_TO_ROM_AS_MOTOR_ZERO_COMMAND);
  detail::place_value<uint32_t>(msg, value, 4, 7);
  return msg;
}

/*
  The motor replies to the host after receiving the command, and the frame data
  is the same as the command sent by the host.
*/
uint32_t RMDX::AsWriteMultiTurnValueToROMAsMotorZeroCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(
      msg, WRITE_ENCODER_MULTI_TURN_VALUE_TO_ROM_AS_MOTOR_ZERO_COMMAND);
  return detail::parse_value<uint32_t>(msg, 4, 7);
}

::cf_motors::bridges::CanMsg
RMDX::NewWriteCurrentMultiTurnPositionEncoderToROMAsMotorZeroCommand(
    const uint32_t id) const {
  return create_message_with_command_code(
      id, WRITE_MULTI_TURN_POSITION_OF_THE_ENCODER_TO_THE_ROM_AS_ZERO_COMMAND);
}

uint32_t RMDX::AsWriteCurrentMultiTurnPositionEncoderToROMAsMotorZeroCommand(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(
      msg, WRITE_MULTI_TURN_POSITION_OF_THE_ENCODER_TO_THE_ROM_AS_ZERO_COMMAND);
  return detail::parse_value<uint32_t>(msg, 4, 7);
}

::cf_motors::bridges::CanMsg
RMDX::NewReadMultiTurnAngleCommand(const uint32_t id) const {
  return create_message_with_command_code(id, READ_MULTI_TURN_ANGLE_COMMAND);
}

uint32_t RMDX::AsReadMultiTurnAngleCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg, READ_MULTI_TURN_ANGLE_COMMAND);
  return detail::parse_value<uint32_t>(msg, 4, 7);
}

::cf_motors::bridges::CanMsg
RMDX::NewReadMotorStatus1AndErrorFlagCommand(const uint32_t id) const {
  return create_message_with_command_code(
      id, READ_MOTOR_STATUS_1_AND_ERROR_FLAG_COMMAND);
}

motor_status_1 RMDX::AsReadMotorStatus1AndErrorFlagCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {

  validate_message_command_id(msg, READ_MOTOR_STATUS_1_AND_ERROR_FLAG_COMMAND);
  motor_status_1 status;
  status.temperature = msg.data[1];
  status.break_release_command = msg.data[3];

  // Parsing voltage and error_state.
  const uint16_t voltage = detail::parse_value<uint16_t>(msg, 4, 5);
  const uint16_t error_status = detail::parse_value<uint16_t>(msg, 6, 7);
  status.voltage = float(voltage) * 0.1;
  status.error_status = error_status;
  return status;
}

::cf_motors::bridges::CanMsg
RMDX::NewReadMotorStatus2Command(const uint32_t id) const {
  return create_message_with_command_code(id, READ_MOTOR_STATUS_2_COMMAND);
}
motor_status_2 RMDX::AsReadMotorStatus2CommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg, READ_MOTOR_STATUS_2_COMMAND);
  return parse_motor_status_2_message(msg);
}

::cf_motors::bridges::CanMsg
RMDX::NewReadMotorStatus3Command(const uint32_t id) const {
  return create_message_with_command_code(id, READ_MOTOR_STATUS_3_COMMAND);
}

motor_status_3 RMDX::AsReadMotorStatus3CommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg, READ_MOTOR_STATUS_3_COMMAND);
  motor_status_3 status;
  status.temperature = msg.data[1];
  const uint16_t phase_A = detail::parse_value<uint16_t>(msg, 2, 3);
  const uint16_t phase_B = detail::parse_value<uint16_t>(msg, 4, 5);
  const uint16_t phase_C = detail::parse_value<uint16_t>(msg, 6, 7);
  status.phase_A = float(phase_A) * 0.01;
  status.phase_B = float(phase_B) * 0.01;
  status.phase_C = float(phase_C) * 0.01;
  return status;
}

::cf_motors::bridges::CanMsg
RMDX::NewMotorStopCommand(const uint32_t id, const bool need_shutdown) const {
  if (need_shutdown) {
    return create_message_with_command_code(id, MOTOR_SHUTDOWN_COMMAND);
  } else {
    return create_message_with_command_code(id, MOTOR_STOP_COMMAND);
  }
}

void RMDX::AsMotorStopCommandResponse(const ::cf_motors::bridges::CanMsg &msg,
                                      const bool need_shutdown) const {
  if (need_shutdown) {
    validate_message_command_id(msg, MOTOR_SHUTDOWN_COMMAND);
  } else {
    validate_message_command_id(msg, MOTOR_STOP_COMMAND);
  }
}

::cf_motors::bridges::CanMsg
RMDX::NewTorqueClosedLoopControlCommand(const uint32_t id,
                                        const int16_t torque) const {
  auto msg = create_message_with_command_code(id, TORQUE_CLOSED_LOOP_COMMAND);
  detail::place_value<int16_t>(msg, torque, 0, 1);
  return msg;
}

torque_set_response RMDX::AsTorqueClosedLoopControlCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg, TORQUE_CLOSED_LOOP_COMMAND);
  /*
  Data[1] = 0x32 is 50 in decimal, which means the motor temperature is 50
  degrees at the moment. The composite data of Data[2] and Data[3] 0x0064 is 100
  in decimal, and it is 100*0.01=1A when scaled down by 100 times, which means
  that the actual current of the current motor is 1A. The composite data 0x01F4
  of Data[4] and Data[5] is 500 in decimal, which means the motor output shaft
  speed is 500dps. There is a reduction ratio relationship between the motor
  output shaft speed and the motor speed. If the reduction ratio is 6, then the
  motor speed is 6 times higher than the output shaft speed. The composite data
  of Data[6] and Data[7] 0x002D is 45 in decimal, which means that the motor
  output shaft moves 45 degrees in the positive direction relative to the zero
  position. The position of the motor output shaft is related to the number of
  lines of the motor encoder and the reduction ratio. For example, if the number
  of lines of the motor encoder is 65536 and the reduction ratio is 6, then 360
  degrees of the motor output shaft corresponds to 65536*6 = 393216 pulses.
  */
  torque_set_response resp;
  resp.temperature = msg.data[1];
  resp.iq = detail::parse_value<uint16_t>(msg, 2, 3);
  resp.speed = detail::parse_value<uint16_t>(msg, 4, 5);
  resp.degree = detail::parse_value<uint16_t>(msg, 6, 7);
  return resp;
}

::cf_motors::bridges::CanMsg
RMDX::NewSpeedLoopControlCommand(const uint32_t id,
                                 const uint32_t speed_control) const {
  auto msg =
      create_message_with_command_code(id, SPEED_CLOSED_LOOP_CONTROL_COMMAND);
  detail::place_value<uint32_t>(msg, speed_control, 4, 7);
  return msg;
}

torque_set_response RMDX::AsSpeedLoopControlCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {

  /*
   Data[1] = 0x32 is 50 in decimal, which means the motor temperature is 50
   degrees at the moment. The composite data of Data[2] and Data[3] 0x0064 is
   100 in decimal, and it is 100*0.01=1A when scaled down by 100 times, which
   means that the actual current of the current motor is 1A. The composite data
   0x01F4 of Data[4] and Data[5] is 500 in decimal, which means the motor output
   shaft speed is 500dps. There is a reduction ratio relationship between the
   motor output shaft speed and the motor speed. If the reduction ratio is 6,
   then the motor speed is 6 times higher than the output shaft speed. The
   composite data of Data[6] and Data[7] 0x002D is 45 in decimal, which means
   that the motor output shaft moves 45 degrees in the positive direction
   relative to the zero position. The position of the motor output shaft is
   related to the number of lines of the motor encoder and the reduction ratio.
   For example, if the number of lines of the motor encoder is 65536 and the
   reduction ratio is 6, then 360 degrees of the motor output shaft corresponds
   to 65536*6 = 393216 pulses.
  */
  torque_set_response resp;
  resp.temperature = msg.data[1];
  resp.iq = detail::parse_value<uint16_t>(msg, 2, 3);
  resp.speed = detail::parse_value<uint16_t>(msg, 4, 5);
  resp.degree = detail::parse_value<uint16_t>(msg, 6, 7);
  return resp;
}

::cf_motors::bridges::CanMsg
RMDX::NewPositionTrackingControlCommand(const uint32_t id,
                                        const uint32_t angle) const {
  auto msg =
      create_message_with_command_code(id, POSITION_TRACKING_CONTROL_COMMAND);
  detail::place_value<uint32_t>(msg, angle, 4, 7);
  return msg;
}

torque_set_response RMDX::AsPositionTrackingControlCommand(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg, POSITION_TRACKING_CONTROL_COMMAND);
  torque_set_response resp;
  resp.temperature = msg.data[1];
  resp.iq = detail::parse_value<uint16_t>(msg, 2, 3);
  resp.speed = detail::parse_value<uint16_t>(msg, 4, 5);
  resp.degree = detail::parse_value<uint16_t>(msg, 6, 7);
  return resp;
}

::cf_motors::bridges::CanMsg RMDX::NewAbsolutePositionClosedLoopControlCommand(
    const uint32_t id, const int32_t angle, const uint16_t speed) const {
  auto msg = create_message_with_command_code(
      id, ABSOLUTE_POSITION_CLOSED_LOOP_CONTROL_COMMAND);
  detail::place_value<uint16_t>(msg, speed, 2, 3);
  detail::place_value<int32_t>(msg, angle, 4, 7);
  return msg;
}

torque_set_response RMDX::AsAbsolutePositionClosedLoopControlCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg,
                              ABSOLUTE_POSITION_CLOSED_LOOP_CONTROL_COMMAND);
  torque_set_response resp;
  resp.temperature = msg.data[1];
  resp.iq = detail::parse_value<uint16_t>(msg, 2, 3);
  resp.speed = detail::parse_value<uint16_t>(msg, 4, 5);
  resp.degree = detail::parse_value<uint16_t>(msg, 6, 7);
  return resp;
}

::cf_motors::bridges::CanMsg RMDX::NewPositionTrackingCommandWithSpeedLimit(
    const uint32_t id, const int32_t angle, const uint16_t speed) const {
  auto msg = create_message_with_command_code(
      id, POSITION_TRACKING_CONTROL_COMMAND_WITH_SPEED_LIMIT_COMMAND);
  detail::place_value<uint16_t>(msg, speed, 2, 3);
  detail::place_value<int32_t>(msg, angle, 4, 7);
  return msg;
}

torque_set_response RMDX::AsPositionTrackingCommandWithSpeedLimitResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(
      msg, POSITION_TRACKING_CONTROL_COMMAND_WITH_SPEED_LIMIT_COMMAND);
  torque_set_response resp;
  resp.temperature = msg.data[1];
  resp.iq = detail::parse_value<uint16_t>(msg, 2, 3);
  resp.speed = detail::parse_value<uint16_t>(msg, 4, 5);
  resp.degree = detail::parse_value<uint16_t>(msg, 6, 7);
  return resp;
}

::cf_motors::bridges::CanMsg RMDX::NewIncrementalPositionClosedLoopCommand(
    const uint32_t id, const int32_t angle, const int16_t speed) const {
  auto msg = create_message_with_command_code(
      id, INCREMENTAL_POSITION_CLOSED_LOOP_CONTROL_COMMAND);
  detail::place_value<int16_t>(msg, speed, 2, 3);
  detail::place_value<uint32_t>(msg, angle, 4, 7);
  return msg;
}

torque_set_response RMDX::AsIncrementalPositionClosedLoopCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg,
                              INCREMENTAL_POSITION_CLOSED_LOOP_CONTROL_COMMAND);
  torque_set_response resp;
  resp.temperature = msg.data[1];
  resp.iq = detail::parse_value<uint16_t>(msg, 2, 3);
  resp.speed = detail::parse_value<uint16_t>(msg, 4, 5);
  resp.degree = detail::parse_value<uint16_t>(msg, 6, 7);
  return resp;
}

::cf_motors::bridges::CanMsg
RMDX::NewSystemOperatingModeAcquisition(const uint32_t id) const {
  return create_message_with_command_code(
      id, SYSTEM_OPERATING_MODE_ACQUISITION_COMMAND);
}

uint8_t RMDX::AsSystemOperatingModeAcquisitionResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg, SYSTEM_OPERATING_MODE_ACQUISITION_COMMAND);
  return msg.data[7];
}

::cf_motors::bridges::CanMsg
RMDX::NewMotorPowerAcquisitionCommand(const uint32_t id) const {
  return create_message_with_command_code(id, MOTOR_POWER_ACQUISITION_COMMAND);
}
uint16_t RMDX::AsMotorPowerAcquisitionCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg, MOTOR_POWER_ACQUISITION_COMMAND);
  return detail::parse_value<uint16_t>(msg, 6, 7);
}

::cf_motors::bridges::CanMsg
RMDX::NewSystemResetControlCommand(const uint32_t id) const {
  return create_message_with_command_code(id, SYSTEM_RESET_COMMAND);
}
void RMDX::AsSystemResetControlCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg, SYSTEM_RESET_COMMAND);
}

::cf_motors::bridges::CanMsg
RMDX::NewSystemBreakReleaseCommand(const uint32_t id) const {
  return create_message_with_command_code(id, SYSTEM_BREAK_RELEASE_COMMAND);
}
void RMDX::AsSystemBreakReleaseCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg, SYSTEM_BREAK_RELEASE_COMMAND);
}

::cf_motors::bridges::CanMsg
RMDX::NewSystemBreakLockCommand(const uint32_t id) const {
  return create_message_with_command_code(id, SYSTEM_BREAK_LOCK_COMMAND);
}
void RMDX::AsSystemBreakLockCommand(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg, SYSTEM_BREAK_LOCK_COMMAND);
}

::cf_motors::bridges::CanMsg
RMDX::NewSystemRuntimeReadCommand(const uint32_t id) const {
  return create_message_with_command_code(id, SYSTEM_RUNTIME_READ_COMMAND);
}
uint32_t RMDX::AsSystemRuntimeReadCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg, SYSTEM_RUNTIME_READ_COMMAND);
  return detail::parse_value<uint32_t>(msg, 4, 7);
}

::cf_motors::bridges::CanMsg
RMDX::NewSystemSoftwareVersionDateReadCommand(const uint32_t id) const {
  return create_message_with_command_code(id,
                                          SYSTEM_SOFTWARE_VERSION_READ_COMMAND);
}
uint32_t RMDX::AsSystemSoftwareVersionDateReadCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg, SYSTEM_SOFTWARE_VERSION_READ_COMMAND);
  return detail::parse_value<uint32_t>(msg, 4, 7);
}

::cf_motors::bridges::CanMsg
RMDX::NewCommunicationInterruptProtectionTimeSettingCommand(
    const uint32_t id, const uint32_t time) const {
  auto msg = create_message_with_command_code(
      id, COMMUNICATION_INTERRUPTION_PROTECTION_TIME_SETTING_COMMAND);
  detail::place_value<uint32_t>(msg, time, 4, 7);
  return msg;
}

uint32_t RMDX::AsCommunicationInterruptProtectionTimeSettingCommand(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(
      msg, COMMUNICATION_INTERRUPTION_PROTECTION_TIME_SETTING_COMMAND);
  return detail::parse_value<uint32_t>(msg, 4, 7);
}

::cf_motors::bridges::CanMsg
RMDX::NewCommunicationBaudRateSettingCommand(const uint32_t id,
                                             const uint8_t baud_rate) const {
  auto msg = create_message_with_command_code(
      id, COMMUNICATION_BAUD_RATE_SETTING_COMMAND);
  msg.data[7] = baud_rate;
  return msg;
}
void RMDX::AsCommunicationBaudRateSettingCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg, COMMUNICATION_BAUD_RATE_SETTING_COMMAND);
}

::cf_motors::bridges::CanMsg
RMDX::NewMotorModelReadingCommand(const uint32_t id) const {
  return create_message_with_command_code(id, MOTOR_MODEL_READING_COMMAND);
}

void RMDX::AsMotorModelReadingCommand(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg, MOTOR_MODEL_READING_COMMAND);
}

::cf_motors::bridges::CanMsg
RMDX::NewFunctionControlCommand(const uint32_t id, const uint8_t index,
                                const uint32_t value) const {
  auto msg = create_message_with_command_code(id, FUNCTION_CONTROL_COMMAND);
  msg.data[1] = index;
  detail::place_value<uint32_t>(msg, value, 4, 7);
  return msg;
}
uint8_t RMDX::AsFunctionControlCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  validate_message_command_id(msg, FUNCTION_CONTROL_COMMAND);
  return msg.data[1];
}

// Helper functions with buff opts.
::std::array<double, 6>
RMDX::parse_pid_parameters(const ::std::array<uint8_t, 8> &data) const {
  auto guard = read_lock(this->guard_);

  auto actualParameters = ::std::array<double, 6>();

  const double KP = (double)data[2];
  actualParameters[0] =
      (this->maximum_value_of_current_loop_ / double(256)) * KP;

  const double KI = (double)data[3];
  actualParameters[1] =
      (this->maximum_value_of_current_loop_ki_ / double(256)) * KI;

  const double KPSpeed = (double)data[4];
  actualParameters[2] =
      (this->maximum_value_of_speed_loop_ / double(256)) * KPSpeed;

  const double KISpeed = (double)data[5];
  actualParameters[3] =
      (this->maximum_value_of_speed_loop_ki_ / double(256)) * KISpeed;

  const double KPPosition = (double)data[6];
  actualParameters[4] =
      (this->maximum_value_of_position_loop_ / double(256)) * KPPosition;

  const double KIPostion = (double)data[7];
  actualParameters[5] =
      (this->maximum_value_of_position_loop_ki_ / double(256)) / KIPostion;
  return actualParameters;
}

::cf_motors::bridges::CanMsg
RMDX::create_message_with_command_code(const uint32_t id,
                                       const uint8_t cid) const {
  ::std::array<uint8_t, 8> data{0};
  data[0] = cid;
  return ::cf_motors::bridges::CanMsg{.id = id, .data = data};
}

void RMDX::validate_message_command_id(const ::cf_motors::bridges::CanMsg &msg,
                                       const uint8_t cid) const {
  if (msg.data[0] != cid) {
    throw ::std::invalid_argument("can't parse other command");
  }
}

motor_status_2 RMDX::parse_motor_status_2_message(
    const ::cf_motors::bridges::CanMsg &msg) const {
  motor_status_2 status;
  status.temperature = msg.data[1];
  status.torque = detail::parse_value<uint16_t>(msg, 2, 3);
  status.speed = detail::parse_value<uint16_t>(msg, 4, 5);
  status.degree = detail::parse_value<uint16_t>(msg, 6, 7);
  return status;
}

} // namespace protocol

} // namespace cf_motors
