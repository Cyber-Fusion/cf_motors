#pragma once

#ifdef __linux__
#include <bridges/can/types.hpp>

#include <linux/can/raw.h>

#include <boost/asio.hpp>
#include <sstream>

namespace cf_motors {
namespace bridges {

template <CanMessageHandler HandlerT> class CanBridge {
public:
  CanBridge(std::shared_ptr<HandlerT> node, boost::asio::io_service &ios,
            std::string can_socket = "can0")
      : node_(node), ios_(ios), stream_(ios), signals_(ios, SIGTERM, SIGINT),
        can_socket_(can_socket) {}

  void Run();
  void SendCommand(const CanMsg msg);

private:
  // Callbacks for asio.
  void on_message(struct can_frame &rec_frame,
                  boost::asio::posix::basic_stream_descriptor<> &stream);

  void on_send();
  void stop();

private:
  // Delegator interface.
  std::shared_ptr<HandlerT> node_;

  // Socket connection interfaces.
  boost::asio::io_service &ios_;
  boost::asio::posix::basic_stream_descriptor<> stream_;
  boost::asio::signal_set signals_;
  std::string can_socket_;
  struct sockaddr_can addr_;
  struct ifreq ifr_;
  int nat_sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  uint32_t last_frame_id_;
  struct can_frame rec_frame_;
};

/*
  @brief: Binding callbacks.
*/
template <CanMessageHandler HandlerT> void CanBridge<HandlerT>::Run() {
  strcpy(ifr_.ifr_name, can_socket_.c_str());
  ioctl(nat_sock_, SIOCGIFINDEX, &ifr_);

  addr_.can_family = AF_CAN;
  addr_.can_ifindex = ifr_.ifr_ifindex;

  if (bind(nat_sock_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) {
    throw std::runtime_error("Error in socket bind");
  }

  stream_.assign(nat_sock_);

  stream_.async_read_some(boost::asio::buffer(&rec_frame_, sizeof(rec_frame_)),
                          std::bind(&CanBridge::on_message, this,
                                    std::ref(rec_frame_), std::ref(stream_)));

  signals_.async_wait(std::bind(&CanBridge::stop, this));
}

/*
  @brief: Sending message to CAN.
*/
template <CanMessageHandler HandlerT>
void CanBridge<HandlerT>::SendCommand(const CanMsg msg) {
  struct can_frame frame_to_send;

  frame_to_send.can_id = msg.id;
  last_frame_id_ = msg.id;

  if (msg.eff == 1) {
    frame_to_send.can_id = frame_to_send.can_id + CAN_EFF_FLAG;
  }

  if (msg.err == 1) {
    frame_to_send.can_id = frame_to_send.can_id + CAN_ERR_FLAG;
  }

  if (msg.rtr == 1) {
    frame_to_send.can_id = frame_to_send.can_id + CAN_RTR_FLAG;
  }

  frame_to_send.can_dlc = msg.dlc;

  for (int i = 0; i < (int)frame_to_send.can_dlc; i++) {
    frame_to_send.data[i] = msg.data[i];
  }

  stream_.async_write_some(
      boost::asio::buffer(&frame_to_send, sizeof(frame_to_send)),
      std::bind(&CanBridge::on_send, this));
}

/*
  @brief: Handling  message from CAN.
*/
template <CanMessageHandler HandlerT>
void CanBridge<HandlerT>::on_message(
    struct ::can_frame &rec_frame,
    ::boost::asio::posix::basic_stream_descriptor<> &stream) {

  CanMsg frame_to_read;

  frame_to_read.id = rec_frame.can_id;
  frame_to_read.dlc = int(rec_frame.can_dlc);

  for (int i = 0; i < rec_frame.can_dlc; i++) {
    frame_to_read.data[i] = rec_frame.data[i];
  }

  node_->Delegate(frame_to_read);

  stream.async_read_some(boost::asio::buffer(&rec_frame_, sizeof(rec_frame_)),
                         std::bind(&CanBridge::on_message, this,
                                   std::ref(rec_frame_), std::ref(stream_)));
}

/*
  @brief: Invokes after sending message to CAN.
*/
template <CanMessageHandler HandlerT> void CanBridge<HandlerT>::on_send() {}

/*
  @brief : Stopping io_service.
*/
template <CanMessageHandler HandlerT> void CanBridge<HandlerT>::stop() {
  ios_.stop();
  signals_.clear();
}

// template class CanBridge<FakeCanMessageHandler>;

void some_function();

} // namespace bridges
} // namespace cf_motors

#endif