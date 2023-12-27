#include <array>
#include <cstdint>
#include <iostream>

namespace cf_motors {
namespace bridges {

struct CanMsg {
  uint32_t id;
  uint8_t dlc;
  std::array<uint8_t, 8> data;
  uint8_t err;
  uint8_t rtr;
  uint8_t eff;
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
