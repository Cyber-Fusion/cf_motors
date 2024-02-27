
#include <options.hpp>

#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <bridges/can/types.hpp>
#include <bridges/serial/serial.hpp>
#include <builder.hpp>
#include <iostream>
#include <memory>
#include <protocol/can/rmdx.hpp>
#include <string>

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace pt = boost::property_tree;

bool fileExists(const std::string &filePath);

int main(int argc, char **argv) {
  try {
    auto opts = provide_cl_arguments(argc, argv);
    if (opts.vm.count("help") || argc < 2) {
      opts.main_opts.print(std::cout);
      return 0;
    }

    std::string command = std::string(argv[1]);

    cf_motors::protocol::RMDX rmdx_motor;
    CommandBuilder<cf_motors::protocol::RMDX> rmdx_command_builder(rmdx_motor);

    std::shared_ptr<std::vector<cf_motors::bridges::CanMsg>> commands;

    fs::path home_dir(std::getenv("HOME"));
    fs::path config_file_name(".cf_motors.json");
    fs::path basic_config_json_file_path = home_dir / config_file_name;

    pt::ptree basic_config_json;
    bool configured_from__file = false;
    if (fileExists(basic_config_json_file_path.string()) &&
        command != "configure") {
      pt::read_json(basic_config_json_file_path.string(), basic_config_json);
      configured_from__file = true;
    }

    if (command == "execute" && configured_from__file) {
      // Parsing command for simple execution.
      auto can_id = basic_config_json.get<uint32_t>("CAN_ID");
      commands = rmdx_command_builder.BuildCommands(opts.vm, can_id);
    } else if (command == "file" && configured_from__file) {
      // Reading commands from file for execution of commands.
      fs::path commands_file_path = opts.vm.at("path").as<fs::path>();
      pt::ptree commands_json;
      pt::read_json(commands_file_path.string(), commands_json);
      commands = rmdx_command_builder.BuildCommands(
          commands_json, commands_json.get<uint32_t>("CAN_ID"));
    } else if (command == "configure") {
      // Configuring cli for further work.
      if (!opts.vm.count("serial-port") || !opts.vm.count("baud-rate") ||
          !opts.vm.count("can-id")) {
        std::cout << "Please configure fully with serial-port and baud-rate.\n";
        return 1;
      }

      // Setting configurations.
      pt::ptree new_config_json;
      new_config_json.put("CAN_ID", opts.vm.at("can-id").as<uint32_t>());
      new_config_json.put("serial-port",
                          opts.vm.at("serial-port").as<std::string>());
      new_config_json.put("baud-rate", opts.vm.at("baud-rate").as<uint32_t>());
      pt::write_json(basic_config_json_file_path.string(), new_config_json);

      std::cout << "The motor was successfully configured.\n";
      return 0;
    } else {
      std::cout << "invalid command\n";
      opts.main_opts.print(std::cout);
      return 1;
    }

    boost::asio::io_service io_service;
    std::shared_ptr<
        cf_motors::bridges::SerialToCanSync<cf_motors::bridges::CanMsg>>
        serial_b = std::make_shared<
            cf_motors::bridges::SerialToCanSync<cf_motors::bridges::CanMsg>>(
            io_service, basic_config_json.get<std::string>("serial-port"),
            basic_config_json.get<uint32_t>("baud-rate"));

    for (auto it = commands->begin(); it != commands->end(); ++it) {
      serial_b->SendCommand(*it);
      auto response = serial_b->ReceiveCommand();
      std::cout << "--- Response ---\n";
      std::cout << "id: " << response.id << ", dlc: " << response.dlc
                << ", err: " << response.err << ", rtr: " << response.rtr
                << ", e(ff: " << response.eff << "\n";
      for (auto &b : response.data) {
        std::cout << " " << b;
      }
      std::cout << "\n------\n";
    }
  } catch (const std::exception &ex) {
    std::cout << ex.what() << std::endl;
    return 1;
  }

  std::cout << "Execution finished\n";
  return 0;
}

bool fileExists(const std::string &filePath) {
  fs::path pathObj(filePath);
  return fs::exists(pathObj) && fs::is_regular_file(pathObj);
}
