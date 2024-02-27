#pragma once

#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <vector>

#include <commands/base.hpp>
#include <commands/rotate.hpp>
#include <exception>
#include <memory>

template <CommandCreatorConcept Motor> class CommandBuilder {
public:
  CommandBuilder(Motor &m) : motor_(m) {}

  std::shared_ptr<std::vector<cf_motors::bridges::CanMsg>>
  BuildCommands(const boost::program_options::variables_map &vm,
                const uint32_t can_id);

  std::shared_ptr<std::vector<cf_motors::bridges::CanMsg>>
  BuildCommands(const boost::property_tree::ptree pt, const uint32_t can_id);

private:
  Motor &motor_;
};

template <CommandCreatorConcept Motor>
std::shared_ptr<std::vector<cf_motors::bridges::CanMsg>>
CommandBuilder<Motor>::BuildCommands(
    const boost::program_options::variables_map &vm, const uint32_t can_id) {

  std::vector<cf_motors::bridges::CanMsg> commands;
  if (vm.count("radian")) {
    if (!vm.count("side")) {
      throw std::invalid_argument("side not provided");
    }
    int32_t radian = vm.at("radian").as<int32_t>();
    std::string side = vm.at("side").as<std::string>();
    auto rc = rotate_command<Motor>(motor_, can_id, radian, 10,
                                    absolute_position_closed_loop);
    commands.push_back(rc.CanMessage());
  }

  return std::make_shared<std::vector<cf_motors::bridges::CanMsg>>(
      std::move(commands));
}

template <CommandCreatorConcept Motor>
std::shared_ptr<std::vector<cf_motors::bridges::CanMsg>>
CommandBuilder<Motor>::BuildCommands(const boost::property_tree::ptree pt,
                                     const uint32_t can_id) {
  std::vector<cf_motors::bridges::CanMsg> commands;
  try {
    for (const auto &action : pt.get_child("actions")) {
      std::string type = action.second.get<std::string>("type");

      if (type == "rotate") {
        int32_t angle = action.second.get<int32_t>("options.angle");
        int16_t speed = action.second.get<int16_t>("options.speed");
        auto rc = rotate_command<Motor>(motor_, can_id, angle, speed,
                                        absolute_position_closed_loop);
        commands.push_back(rc.CanMessage());
        // TODO: Handle this.
        bool negative = action.second.get<bool>("options.negative");
      } else if (type == "status") {
        // TODO:
      }
    }
  } catch (const std::exception &e) {
    throw e;
  }
  return std::make_shared<std::vector<cf_motors::bridges::CanMsg>>(
      std::move(commands));
}
