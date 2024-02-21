#pragma once
#include <bridges/can/can.hpp>

struct Command {
  virtual cf_motors::bridges::CanMsg CanMessage() = 0;
};
