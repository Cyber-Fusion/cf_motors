#pragma once

#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <vector>

#include <commands/base.hpp>
#include <commands/rotate.hpp>
#include <commands/status.hpp>
#include <exception>
#include <memory>

template <typename T>
concept CommandCreatorConcept =
    StatusCommandCreatorConcept<T> && RotateCommandCreatorConcept<T>;

template <CommandCreatorConcept Motor> class CommandBuilder {
public:
  CommandBuilder(Motor &m) : motor_(m) {}

  std::shared_ptr<std::vector<UARTProxyCommand>>
  BuildCommands(const boost::program_options::variables_map &vm,
                const uint32_t can_id);

  std::shared_ptr<std::vector<UARTProxyCommand>>
  BuildCommands(const boost::property_tree::ptree pt, const uint32_t can_id);

private:
  Motor &motor_;

private:
};

template <CommandCreatorConcept Motor>
std::shared_ptr<std::vector<UARTProxyCommand>>
CommandBuilder<Motor>::BuildCommands(
    const boost::program_options::variables_map &vm, const uint32_t can_id) {
  std::vector<UARTProxyCommand> commands;

  // First we need to push status reading command.
  commands.push_back(status_command<Motor>(motor_, can_id, status_2).Message());

  if (vm.count("radian")) {
    if (!vm.count("direction") || !vm.count("speed")) {
      throw std::invalid_argument("side not provided");
    }
    std::cout << "building rotation command\n";
    int32_t radian = vm.at("radian").as<int32_t>();
    std::string direction = vm.at("direction").as<std::string>();
    uint8_t speed = vm.at("speed").as<uint16_t>();
    auto rc = rotate_command<Motor>(motor_, can_id, radian, speed,
                                    absolute_position_closed_loop);
    commands.push_back(rc.Message());
  }

  return std::make_shared<std::vector<UARTProxyCommand>>(std::move(commands));
}

template <CommandCreatorConcept Motor>
std::shared_ptr<std::vector<UARTProxyCommand>>
CommandBuilder<Motor>::BuildCommands(const boost::property_tree::ptree pt,
                                     const uint32_t can_id) {
  std::vector<UARTProxyCommand> commands;

  // First we need to push status reading command.
  commands.push_back(status_command<Motor>(motor_, can_id, status_2).Message());

  try {
    for (const auto &action : pt.get_child("actions")) {
      std::string type = action.second.get<std::string>("type");

      if (type == "rotate") {
        std::cout << "building rotation command\n";
        int32_t angle = action.second.get<int32_t>("options.angle");
        int16_t speed = action.second.get<int16_t>("options.speed");
        auto rc = rotate_command<Motor>(motor_, can_id, angle, speed,
                                        absolute_position_closed_loop);
        commands.push_back(rc.Message());
        // TODO: Handle direction.
        bool negative = action.second.get<bool>("options.negative");
      } else if (type == "status") {
        std::cout << "building status reading command\n";
        // TODO:
      }
    }
  } catch (const std::exception &e) {
    throw e;
  }
  return std::make_shared<std::vector<UARTProxyCommand>>(std::move(commands));
}
