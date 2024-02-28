#pragma once
#include <bridges/can/can.hpp>
#include <sstream>

struct UARTProxyCommand {
  uint16_t id;
  std::array<uint8_t, 8> data;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &id;
    for (std::size_t i = 0; i < 8; ++i) {
      ar &data[i];
    }
  }

  void deserialize(std::istringstream &is) {
    is >> id;
    for (auto &d : data) {
      is >> d;
    }
  }
};

struct Command {
  virtual UARTProxyCommand CanMessage() = 0;
};
