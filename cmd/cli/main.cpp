
#include <options.hpp>

#include <bridges/can/types.hpp>
#include <bridges/serial/serial.hpp>

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace po = boost::program_options;

int main(int argc, char **argv) {

  auto opts = provide_cl_arguments(argc, argv);
  if (opts.vm.count("help") || argc < 2) {
    opts.main_opts.print(std::cout);
    return 0;
  }

  std::string command = std::string(argv[1]);

  // TODO: Setup all.
  std::shared_ptr<
      cf_motors::bridges::SerialToCanSync<cf_motors::bridges::CanMsg>>
      serial_b;

  if (command == "execute") {
    std::cout << "execute\n";
    // TODO: Execute commands.
  } else if (command == "file") {
    std::cout << "file\n";
    // TODO: Read commands from file.
  } else if (command == "configure") {
    std::cout << "configure\n";
    // TODO: Setup configurations.
  } else {
    std::cout << "invalid command\n";
    opts.main_opts.print(std::cout);
    return 1;
  }

  return 0;
}
