#pragma once

#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <vector>

#include <commands/base.hpp>
#include <commands/rotate.hpp>
#include <exception>

template <CommandCreatorConcept Motor> class CommandBuilder {
public:
  CommandBuilder(Motor &m) : motor_(m) {}

  const std::vector<cf_motors::bridges::CanMsg> &
  BuildCommands(const boost::program_options::variables_map &vm);

  const std::vector<cf_motors::bridges::CanMsg> &
  BuildCommands(const boost::property_tree::ptree pt, const uint32_t can_id);

private:
  Motor &motor_;
  std::vector<Command> commands_;
};

template <CommandCreatorConcept Motor>
const std::vector<cf_motors::bridges::CanMsg> &
CommandBuilder<Motor>::BuildCommands(
    const boost::program_options::variables_map &vm) {
  // TODO:
  return std::vector<cf_motors::bridges::CanMsg>();
}

template <CommandCreatorConcept Motor>
const std::vector<cf_motors::bridges::CanMsg> &
CommandBuilder<Motor>::BuildCommands(const boost::property_tree::ptree pt,
                                     const uint32_t can_id) {
  try {
    for (const auto &action : pt.get_child("actions")) {
      std::string type = action.second.get<std::string>("type");

      if (type == "rotate") {
        int32_t angle = action.second.get<int32_t>("options.angle");
        int16_t speed = action.second.get<int16_t>("options.speed");
        commands_.push_back(rotate_command<Motor>(
            motor_, can_id, angle, speed, absolute_position_closed_loop));
        // TODO: Handle this.
        bool negative = action.second.get<bool>("options.negative");
      } else if (type == "status") {
        // TODO:
      }
    }
  }

  catch (const std::exception &e) {
    throw e; // TODO: Handle this.
  }
}
