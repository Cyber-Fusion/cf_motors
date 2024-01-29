#pragma once

#include <boost/asio.hpp>
#include <iostream>
#include <string>

class SerialPortCommunication {
public:
  SerialPortCommunication(boost::asio::io_service &io_service,
                          const std::string &port)
      : io_service_(io_service), serial_port_(io_service, port) {}

  void StartReading() { async_read(); }

  void SendCommand(const std::string &command) {
    // Ensure that the command is terminated with a newline character
    std::string command_with_newline = command + "\n";

    boost::asio::async_write(
        serial_port_, boost::asio::buffer(command_with_newline),
        [this, command](const boost::system::error_code &error,
                        std::size_t /*bytes_transferred*/) {
          if (!error) {
            std::cout << "Command sent: " << command << std::endl;
          } else {
            std::cerr << "Error sending command: " << error.message()
                      << std::endl;
          }
        });
  }

private:
  void async_read() {
    boost::asio::async_read_until(
        serial_port_, response_buffer_, '\n',
        [this](const boost::system::error_code &error,
               std::size_t bytes_transferred) {
          if (!error) {
            // Process the received data
            std::istream response_stream(&response_buffer_);
            std::string response;
            std::getline(response_stream, response);

            std::cout << "Received response: " << response << std::endl;

            // Continue reading
            async_read();
          } else {
            std::cerr << "Error reading from serial port: " << error.message()
                      << std::endl;
          }
        });
  }

private:
  boost::asio::io_service &io_service_;
  boost::asio::serial_port serial_port_;
  boost::asio::streambuf response_buffer_;
};
