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

  std::vector<uint8_t> Serialize() const {
    std::vector<uint8_t> buf(10);

    buf[0] = (id >> 8) & 0xFF; // MSB first
    buf[1] = id & 0xFF;        // LSB second

    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&data);
    std::copy(bytes, bytes + sizeof(data), buf.begin() + sizeof(id));

    return buf;
  }
};

struct Command {
  virtual UARTProxyCommand Message() = 0;
};

UARTProxyCommand translateCANToUART(cf_motors::bridges::CanMsg &msg) {
  return UARTProxyCommand{.id = static_cast<uint16_t>(msg.id),
                          .data = std::array<uint8_t, 8>(std::move(msg.data))};
}
