#pragma once

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace cf_motors {
namespace bridges {

template <Serializable CommandType> class SerialToCanSync {
public:
  SerialToCanSync(boost::asio::io_service &io_service, const std::string &port,
                  const uint32_t baud_rate)
      : serial_port_(io_service, port) {
    serial_port_.set_option(
        boost::asio::serial_port_base::baud_rate(baud_rate));
  }
  SerialToCanSync() = delete;

  void SendCommand(const CommandType &command) {
    auto buf = command.Serialize();
    boost::asio::write(serial_port_, boost::asio::buffer(buf));
  }

  CommandType ReceiveCommand() {
    boost::asio::streambuf buffer;
    boost::asio::read_until(serial_port_, buffer, "\n");
    std::istream is(&buffer);
    std::string serialized_data;
    std::getline(is, serialized_data);
    CommandType command;
    command.Deserialize(serialized_data);
    return command;
  }

private:
  boost::asio::serial_port serial_port_;

  std::string serialize_with_archive(const CommandType &command) {
    std::ostringstream ss;
    boost::archive::text_oarchive oa(ss);
    oa << command;
    return ss.str();
  }
};

} // namespace bridges
} // namespace cf_motors
