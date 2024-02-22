#pragma once

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <sstream>
#include <string>

namespace cf_motors {
namespace bridges {

// Forward declaration of CanMsg
struct CanMsg;

// SerialToCanSync class
// TODO: Use concept Serializable.
template <typename CommandType> class SerialToCanSync {
public:
  SerialToCanSync(boost::asio::io_service &io_service, const std::string &port,
                  const uint32_t baud_rate)
      : serial_port_(io_service, port) {
    serial_port_.set_option(
        boost::asio::serial_port_base::baud_rate(baud_rate));
  }
  SerialToCanSync() = delete;

  void SendCommand(const CommandType &command) {
    std::ostringstream ss;
    boost::archive::text_oarchive oa(ss);
    oa << command;
    std::string serialized_data = ss.str();
    boost::asio::write(serial_port_, boost::asio::buffer(serialized_data));
  }

  CommandType ReceiveCommand() {
    boost::asio::streambuf buffer;
    boost::asio::read_until(serial_port_, buffer, "\n");
    std::istream is(&buffer);
    std::string serialized_data;
    std::getline(is, serialized_data);
    std::istringstream ss(serialized_data);
    boost::archive::text_iarchive ia(ss);
    CommandType command;
    ia >> command;
    return command;
  }

private:
  boost::asio::serial_port serial_port_;
};

} // namespace bridges
} // namespace cf_motors
