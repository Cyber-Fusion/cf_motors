#pragma once
#include <bridges/can/can.hpp>
#include <protocol/can/msg.hpp>

enum status_type {
  status_1 = 1,
  status_2 = 2,
  status_3 = 3,
};

template <typename T>
concept StatusCommandCreatorConcept =
    requires(T creator, uint32_t id, const cf_motors::bridges::CanMsg &msg) {
  {
    creator.NewReadMotorStatus1AndErrorFlagCommand(id)
    } -> std::same_as<cf_motors::bridges::CanMsg>;
  {
    creator.NewReadMotorStatus2Command(id)
    } -> std::same_as<cf_motors::bridges::CanMsg>;
  {
    creator.NewReadMotorStatus3Command(id)
    } -> std::same_as<cf_motors::bridges::CanMsg>;
};

template <StatusCommandCreatorConcept Motor>
struct status_command : public Command {
  status_command(Motor &m, const uint32_t id, status_type st)
      : m_(m), id_(id), st_(st) {}
  cf_motors::bridges::CanMsg command;
  UARTProxyCommand Message() override {
    switch (st_) {
    case status_1:
      command = m_.NewReadMotorStatus1AndErrorFlagCommand(id_);
      break;
    case status_2:
      command = m_.NewReadMotorStatus2Command(id_);
      break;
    case status_3:
      command = m_.NewReadMotorStatus3Command(id_);
      break;
    default:
      break;
    }
    return translateCANToUART(command);
  }

private:
  Motor &m_;
  status_type st_;
  uint32_t id_;
};
