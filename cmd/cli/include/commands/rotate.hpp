#pragma once

// #include <boost/archive/json_iarchive.h>
// #include <boost/archive/json_oarchive.h>
#include <bridges/can/can.hpp>
#include <protocol/can/msg.hpp>

enum rotation_type {
  position_tracking = 1,
  absolute_position_closed_loop = 2,
  position_tracking_with_speed_limit = 3,
  incremental_position_loop = 4
};

template <typename T>
concept CommandCreatorConcept =
    requires(T creator, uint32_t id, int32_t angle, uint16_t speed,
             const cf_motors::bridges::CanMsg &msg) {
  {
    creator.NewAbsolutePositionClosedLoopControlCommand(id, angle, speed)
    } -> std::same_as<cf_motors::bridges::CanMsg>;
  {
    creator.AsAbsolutePositionClosedLoopControlCommandResponse(msg)
    } -> std::same_as<cf_motors::protocol::torque_set_response>;

  {
    creator.NewPositionTrackingCommandWithSpeedLimit(id, angle, speed)
    } -> std::same_as<cf_motors::bridges::CanMsg>;
  {
    creator.AsPositionTrackingCommandWithSpeedLimitResponse(msg)
    } -> std::same_as<cf_motors::protocol::torque_set_response>;

  {
    creator.NewIncrementalPositionClosedLoopCommand(id, angle, speed)
    } -> std::same_as<cf_motors::bridges::CanMsg>;
  {
    creator.AsIncrementalPositionClosedLoopCommandResponse(msg)
    } -> std::same_as<cf_motors::protocol::torque_set_response>;
};

template <CommandCreatorConcept Motor> struct rotate_command : public Command {
  uint32_t id_;
  int32_t angle;
  uint16_t speed;
  rotation_type rt_;
  Motor &m_;

public:
  rotate_command(Motor &m, const uint32_t id, const int32_t angle,
                 const int16_t speed, const rotation_type t)
      : m_(m), angle(angle), speed(speed), rt_(t) {}
  rotate_command() = delete;

  // template <typename ArchiveT>
  // inline void serialize(ArchiveT &ar, const unsigned int file_version) {
  //   ar &BOOST_SERIALIZATION_NVP(angle);
  //   ar &BOOST_SERIALIZATION_NVP(speed);
  //   ar &BOOST_SERIALIZATION_NVP(rotation_type);
  // }
  cf_motors::bridges::CanMsg CanMessage() override {
    switch (rt_) {
    case position_tracking:
      return m_.NewPositionTrackingControlCommand(id_, angle);
    case absolute_position_closed_loop:
      return m_.NewAbsolutePositionClosedLoopControlCommand(id_, angle, speed);
    case position_tracking_with_speed_limit:
      return m_.NewPositionTrackingCommandWithSpeedLimit(id_, angle, speed);
    case incremental_position_loop:
      return m_.NewIncrementalPositionClosedLoopCommand(id_, angle, speed);
    default:
      throw std::runtime_error("invalid command type");
    }
  }
};
