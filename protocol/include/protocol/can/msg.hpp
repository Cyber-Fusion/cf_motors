#pragma once
#include <cstdint>

namespace cf_motors {
namespace protocol {

enum acceleration_functions {
  position_planning_acceleration = 0x00,
  position_planning_deceleration = 0x01,
  speed_planning_acceleration = 0x02,
  speed_planning_deceleration = 0x03
};

enum operating_mode_acquisition {
  current_loop_mode = 0x01,
  speed_loop_mode = 0x02,
  position_loop_mode = 0x03
};

struct motor_status_1 {
  int8_t temperature = 0;
  uint8_t break_release_command = 0;
  float voltage = 0.0;
  uint16_t error_status = 0;
};

struct motor_status_2 {
  uint8_t temperature = 0;
  uint16_t torque = 0;
  uint16_t speed = 0;
  uint16_t degree = 0;
};

struct motor_status_3 {
  uint8_t temperature = 0;
  float phase_A = 0.0;
  float phase_B = 0;
  float phase_C = 0;
};

struct torque_set_response {
  uint8_t temperature = 0;
  uint16_t iq = 0;
  uint16_t speed = 0;
  uint16_t degree = 0;
};

} // namespace protocol
} // namespace cf_motors