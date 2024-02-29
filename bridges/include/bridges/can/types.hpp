
#pragma once
#include <array>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <vector>

namespace cf_motors {
namespace bridges {

struct CanMsg {
  uint32_t id; // Component id.
  uint8_t dlc;
  std::array<uint8_t, 8> data;
  uint8_t err;
  uint8_t rtr;
  uint8_t eff;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &id;
    ar &dlc;
    for (std::size_t i = 0; i < 8; ++i) {
      ar &data[i];
    }
    ar &err;
    ar &rtr;
    ar &eff;
  }

  void deserialize(std::istringstream &is) {
    is >> id >> dlc;
    for (auto &d : data) {
      is >> d;
    }
    is >> err >> rtr >> eff;
  }

  std::vector<uint8_t> Serialize() const {
    std::vector<uint8_t> buf;
    // TODO: Complete the Serialize function for current struct.
    return buf;
  }
};

// Concept for a serializable type
template <typename T>
concept Serializable = requires(T t, std::ostringstream os,
                                std::istringstream is) {
  { t.Serialize() } -> std::same_as<std::vector<uint8_t>>;
};

template <typename T>
concept CanMessageHandler = requires(T v, CanMsg msg, uint8_t id) {
  { v.Delegate(msg) } -> ::std::same_as<void>;
  { v.ConfirmSend(id) } -> ::std::same_as<void>;
};

// Fake class that satisfies the CanMessageHandler concept
class FakeCanMessageHandler {
public:
  // Member function to delegate CanMsg
  void Delegate(CanMsg msg) {
    ::std::cout << "Delegating CanMsg with ID: " << msg.id << std::endl;
  }

  // Member function to confirm send with ID
  void ConfirmSend(uint32_t id) {
    ::std::cout << "Confirming send for ID: " << id << std::endl;
  }
};

} // namespace bridges
} // namespace cf_motors
