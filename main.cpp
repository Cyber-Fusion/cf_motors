#include <boost/asio.hpp>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

uint8_t GenPos = 0;
uint8_t GenVal = 200;

using namespace std;
using namespace boost;
std::vector<uint8_t> data2{0x01, 0x41, 0x9C, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00};

// Function to handle asynchronous data reception
void handleReceive(const boost::system::error_code &error,
                   std::size_t bytes_transferred, std::vector<uint8_t> &data,
                   boost::asio::serial_port &serial) {
  if (!error) {
    if (bytes_transferred > 0) {
      // std::cout << "Received data: ";
      for (std::size_t i = 0; i < bytes_transferred; ++i) {
        // std::cout <<hex << static_cast<int>(data[i]) ;
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(data[i]) << dec << ' ';
      }

      std::cout << dec << endl;

      std::cout << bytes_transferred << endl;

      data.clear();
    }
    // Continue listening for more data
    boost::asio::async_read_until(
        serial, boost::asio::dynamic_buffer(data), '\n',
        [&data, &serial](const boost::system::error_code &e, std::size_t b) {
          handleReceive(e, b, data, serial);
        });
  } else {
    std::cerr << "Error receiving data: " << error.message() << std::endl;
  }
}

int main() {

  // Configure the serial port and other parameters
  // You may need to adjust the serial port name and baud rate
  const std::string serialPort =
      "/dev/ttyCH341USB0"; // Replace with your serial port
  const int baudRate = 115200;

  boost::asio::io_context io;
  boost::asio::serial_port serial(io, serialPort);

  // Open the serial port

  serial.set_option(boost::asio::serial_port_base::baud_rate(baudRate));

  // Prepare the ID and data to be sent
  try {
    // Send the buffer to the serial port
    boost::asio::write(serial, boost::asio::buffer(data2));

    // Buffer to store received data
    std::vector<uint8_t> receiveData;
    receiveData.clear();
    // Receive data
    size_t bytesRead = boost::asio::read_until(
        serial, asio::dynamic_buffer(receiveData), '\n');

    for (uint8_t a : receiveData) {
      cout << hex << (int)a << ' ';
    }

    cout << endl;
    cout << endl;
    cout << endl;

    long buf8 = int(receiveData[8]);
    long buf9 = int(receiveData[9]);
    GenPos = (int)(buf9 << 8) | buf8;
    GenPos = GenPos * 100;
    GenPos = GenPos + 200;

    uint8_t dataSend[10];
    dataSend[0] = 0x01;
    dataSend[1] = 0x41;
    dataSend[2] = 0xA4;
    dataSend[3] = 0x00;
    dataSend[4] = GenVal;
    dataSend[5] = GenVal >> 8;
    dataSend[6] = (uint8_t)GenPos;
    dataSend[7] = (uint8_t)(GenPos >> 8);
    dataSend[8] = (uint8_t)(GenPos >> 16);
    dataSend[9] = (uint8_t)(GenPos >> 24);

    try {
      // Send the buffer to the serial port
      boost::asio::write(serial, boost::asio::buffer(dataSend));
    } catch (std::exception &e) {
      std::cerr << "Error: " << e.what() << std::endl;
    }

    boost::asio::write(serial, boost::asio::buffer(data2));

    // Buffer to store received data
    std::vector<uint8_t> receiveData2;

    // Receive data
    boost::asio::read_until(serial, asio::dynamic_buffer(receiveData2), '\n');

    for (uint8_t a : receiveData2) {
      cout << hex << (int)a << ' ';
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  } catch (std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  // // Buffer to store received data
  std::vector<uint8_t> receiveBuffer;

  // // Start asynchronous data reception
  boost::asio::async_read_until(
      serial, boost::asio::dynamic_buffer(receiveBuffer), '\n',
      [&receiveBuffer, &serial](const boost::system::error_code &e,
                                std::size_t bytes_transferred) {
        handleReceive(e, bytes_transferred, receiveBuffer, serial);
      });

  // // Run the IO service
  io.run();

  return 0;
}