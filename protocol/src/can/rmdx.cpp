#include <protocol/can/rmdx.hpp>

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

namespace rmdx {

/*
This command can read the parameters of current, speed, position loop KP and
KI at one time, and the data type is uint8_t. The system sets the maximum
range of PI parameters according to the motor model, and then divides it
equally according to the maximum range of uint8_t of 256 units. Users only
need to adjust 0-256 units.
*/
::cf_motors::bridges::CanMsg RMDX::NewReadPIDMessage(const uint32_t id) const {
  auto data = std::array<uint8_t, 8>();
  data[0] = READ_PID_PARAMETER_COMMAND;
  return ::cf_motors::bridges::CanMsg{.id = id, .data = data};
}

::std::array<double, 6>
RMDX::AsReadPIDResponse(const ::cf_motors::bridges::CanMsg &msg) const {
  if (msg.data[0] != READ_PID_PARAMETER_COMMAND) {
    throw ::std::invalid_argument("can't parse other command");
  }
  return this->parse_pid_parameters(msg.data);
}

/*
This command can write the parameters of current, speed, position loop KP and KI
to RAM at one time, and it will not be saved after power off. The data type is
uint8_t. The system sets the maximum range of PI parameters according to the
motor model, and then divides it equally according to the maximum range of
uint8_t of 256 units. Users only need to adjust 0-256 units.
*/
::cf_motors::bridges::CanMsg
RMDX::NewWritePIDParametersMessage(const uint32_t id,
                                   const ::std::array<double, 6> &params,
                                   bool toRAM) const {

  auto data = ::std::array<uint8_t, 8>();
  if (toRAM) {
    data[0] = WRITE_PID_PARAMETERS_TO_RAM_COMMAND;
  } else {
    data[0] = WRITE_PID_PARAMETERS_TO_ROM_COMMAND;
  }

  data[1] = 0x00;
  ::std::copy(params.begin(), params.end(), data.begin() + 2);
  return ::cf_motors::bridges::CanMsg{.id = id, .data = data};
}

::std::array<double, 6>
RMDX::AsWritePIDResponse(const ::cf_motors::bridges::CanMsg &msg,
                         bool toRAM) const {

  if ((toRAM && msg.data[0] != WRITE_PID_PARAMETERS_TO_RAM_COMMAND) ||
      (!toRAM && msg.data[0] != WRITE_PID_PARAMETERS_TO_ROM_COMMAND)) {
    throw ::std::invalid_argument("can't parse other command");
  }
  return this->parse_pid_parameters(msg.data);
}

/*
The host sends this command to read the acceleration parameters of the current
motor
*/
::cf_motors::bridges::CanMsg
RMDX::NewReadAccelerationCommand(const uint32_t id) const {
  auto data = ::std::array<uint8_t, 8>();
  data[0] = READ_ACCELERATION_COMMAND;
  return ::cf_motors::bridges::CanMsg{.id = id, .data = data};
}

/*
The acceleration parameter is included in the drive response data. Acceleration
data Accel is int32_t type, the unit is 1dps/s, and the parameter range is
50-60000.
*/
uint32_t RMDX::AsReadAccelerationResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  if (msg.data[0] != READ_ACCELERATION_COMMAND) {
    throw ::std::invalid_argument("can't parse other command");
  }
  return this->parse_last_4_bytes(msg.data);
}

/*
The host sends this command to write the acceleration and deceleration into RAM
and ROM, which can be saved after power off. Acceleration data Accel is of
uint32_t type, the unit is 1dps/s, and the parameter range is 100-60000. The
command contains the acceleration and deceleration values in the position and
velocity planning, which are determined by the index value.
*/
::cf_motors::bridges::CanMsg
RMDX::NewWriteAccelerationCommand(const uint32_t id,
                                  const uint32_t acceleration,
                                  const acceleration_functions af) const {
  // Convert the uint32_t value to a std::array<uint8_t, 8>
  std::array<uint8_t, 8> data;
  data[0] = WRITE_ACCELERATION_COMMAND_TO_RAM;
  data[1] = af;
  this->place_parameter_in_last_4_bytes(data, acceleration);
  return ::cf_motors::bridges::CanMsg{.id = id, .data = data};
}

/*
The motor will reply to the host after receiving the command, and the reply
command is the same as the received command.
*/
uint32_t RMDX::AsWriteAccelerationCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  if (msg.data[0] != WRITE_ACCELERATION_COMMAND_TO_RAM) {
    throw ::std::invalid_argument("can't parse other command");
  }
  return parse_last_4_bytes(msg.data);
}

/*
The host sends this command to read the multi-turn position of the encoder,
which represents the rotation angle of the motor output shaft, including the
multi-turn angle.
*/
::cf_motors::bridges::CanMsg
RMDX::NewReadMultiTurnEncoderCommand(const uint32_t id) const {
  ::std::array<uint8_t, 8> data;
  data[0] = READ_MULTI_TURN_ENCODER_POSITION_DATA_COMMAND;
  return ::cf_motors::bridges::CanMsg{.id = id, .data = data};
}

/*
The motor replies to the host after receiving the command, and the frame data
contains the following parameters. Encoder multi-turn position encoder (int32_t
type, value range of multi-turn encoder, 4 bytes of valid data), which is the
value after subtracting the encoder's multi-turn zero offset (initial position)
from the original position of the encoder.
*/
uint32_t RMDX::AsReadMultiTurnEncoderCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  if (msg.data[0] != READ_MULTI_TURN_ENCODER_POSITION_DATA_COMMAND) {
    throw ::std::invalid_argument("can't parse other command");
  }

  return this->parse_last_4_bytes(msg.data);
}

/*
The host sends this command to read the multi-turn encoder home position, ie the
multi-turn encoder value without the zero offset (home position).
*/
::cf_motors::bridges::CanMsg
RMDX::NewReadMultiTurnEncoderOriginalPositionCommand(const uint32_t id) const {
  ::std::array<uint8_t, 8> data;
  data[0] = READ_MULTI_TURN_ENCODER_ORIGINAL_POSITION_DATA_COMMAND;
  return ::cf_motors::bridges::CanMsg{.id = id, .data = data};
}

/*
The motor replies to the host after receiving the command, and the frame data
contains the following parameters. Encoder multi-turn raw position encoderRaw
(int32_t type, value range, valid data 4 bytes).
*/
uint32_t RMDX::AsReadMultiTurnEncoderOriginalPositionCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  if (msg.data[0] != READ_MULTI_TURN_ENCODER_ORIGINAL_POSITION_DATA_COMMAND) {
    throw ::std::invalid_argument("can't parse other command");
  }
  return this->parse_last_4_bytes(msg.data);
}

/*
The host sends this command to read the multi-turn zero offset value (initial
position) of the encoder.
*/
::cf_motors::bridges::CanMsg
RMDX::NewReadMultiTurnEncoderZeroOffsetCommand(const uint32_t id) const {
  ::std::array<uint8_t, 8> data;
  data[0] = READ_MULTI_TURN_ENCODER_ZERO_OFFSET_DATA_COMMAND;
  return ::cf_motors::bridges::CanMsg{.id = id, .data = data};
}

/*
The motor replies to the host after receiving the command, and the frame data
contains the following parameters. Encoder multi-turn zero offset encoderOffset
(int32_t type, value range, valid data 4 bytes).
*/
uint32_t RMDX::AsReadMultiTurnEncoderZeroOffsetCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  if (msg.data[0] != READ_MULTI_TURN_ENCODER_ZERO_OFFSET_DATA_COMMAND) {
    throw ::std::invalid_argument("can't parse other command");
  }
  return this->parse_last_4_bytes(msg.data);
}

/*
The host sends this command to set the zero offset (initial position) of the
encoder, where the encoder multi-turn value to be written, encoderOffset, is of
type int32_t, (value range, 4 bytes of valid data). Note: After writing the
position of the new zero point, the motor needs to be restarted to be effective.
Because of the change of the zero offset, the new zero offset (initial position)
should be used as a reference when setting the target position.
*/
::cf_motors::bridges::CanMsg
RMDX::NewWriteMultiTurnValueToROMAsMotorZeroCommand(
    const uint32_t id, const uint32_t value) const {
  ::std::array<uint8_t, 8> data;
  data[0] = WRITE_ENCODER_MULTI_TURN_VALUE_TO_ROM_AS_MOTOR_ZERO_COMMAND;
  this->place_parameter_in_last_4_bytes(data, value);
  return ::cf_motors::bridges::CanMsg{.id = id, .data = data};
}

/*
The motor replies to the host after receiving the command, and the frame data is
the same as the command sent by the host.
*/
uint32_t RMDX::AsWriteMultiTurnValueToROMAsMotorZeroCommandResponse(
    const ::cf_motors::bridges::CanMsg &msg) const {
  if (msg.data[0] !=
      WRITE_ENCODER_MULTI_TURN_VALUE_TO_ROM_AS_MOTOR_ZERO_COMMAND) {
    throw ::std::invalid_argument("can't parse other command");
  }
  return this->parse_last_4_bytes(msg.data);
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

uint32_t RMDX::parse_last_4_bytes(const ::std::array<uint8_t, 8> &data) const {
  uint32_t acceleration = 0;

  // (data[4] is the lowest bit, data[7] is the highest bit)
  for (size_t i = 4; i < 8; ++i) {
    acceleration = (acceleration << 8) | data[i];
  }
  return acceleration;
}

void RMDX::place_parameter_in_last_4_bytes(::std::array<uint8_t, 8> &data,
                                           uint32_t parameter) const {
  for (size_t i = 0; i < 4; ++i) {
    data[4 + i] = static_cast<uint8_t>((parameter >> ((3 - i) * 8)) & 0xFF);
  }
}

} // namespace rmdx

} // namespace cf_motors