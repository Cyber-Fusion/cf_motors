#pragma once
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <protocol/can/rmdx.hpp>

/* TODO:
In future use this variant of implementation.

template <MotorInterface MI, typename CommandT>
struct Command {
  CommandT ExtractCommand(args... any) {
    switch (command_type) {
      case command_1:
        m_.Build(args...);
    }
  }
};

*/

class CommandBuilder {
public:
  CommandBuilder(::cf_motors::rmdx::RMDX &rmdx) : rmdx_(rmdx) {}
  ::cf_motors::bridges::CanMsg
  BuildCommand(const ::boost::program_options::variables_map &vm);
  ::cf_motors::bridges::CanMsg
  BuildCommand(const ::boost::property_tree::ptree pt);

private:
  ::cf_motors::rmdx::RMDX &rmdx_;
};
