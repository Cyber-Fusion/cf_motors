#include <boost/filesystem.hpp>
#include <iostream>
#include <string>

#include <options.hpp>

namespace po = boost::program_options;

cl_options provide_cl_arguments(int argc, char **argv) {
  po::options_description rotate_opts(
      "Execute commands: cf_motor execute [options]");

  rotate_opts.add_options()("radian,r", po::value<int32_t>(),
                            "Radian to rotate")(
      "direction,d", po::value<std::string>(),
      "neg/pos")("speed,s", po::value<uint16_t>(), "Speed of rotations");

  po::options_description from_file("cf_motor file");
  from_file.add_options()("path,p", po::value<boost::filesystem::path>(),
                          "File with commands");

  po::options_description setup_configurations("cf_motor configure");
  setup_configurations.add_options()("serial-port,s", po::value<std::string>(),
                                     "Port to connect with serial port")(
      "baud-rate,b", po::value<uint32_t>(),
      "Baud rate for connection with motor.")(
      "can-id,c", po::value<uint32_t>(),
      "Can id for communication with motor.");

  po::options_description main_opts("Usage: cf_motor [command] [options]");
  main_opts.add(rotate_opts).add(from_file).add(setup_configurations);
  main_opts.add_options()("help,h", "Produce usage instructions.");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(main_opts).run(), vm);
  po::notify(vm);
  return cl_options{.vm = vm, .main_opts = main_opts};
}
