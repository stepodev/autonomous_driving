//
// Created by stepo on 1/21/18.
//


#include "UdpServer.hpp"

UdpServer::UdpServer(boost::asio::io_service &io_service, ros::Publisher pub)
    : socket_( io_service, udp::endpoint(boost::asio::ip::address_v4::broadcast(), 10000)) {

  //publisher of stuff
  pub_platoonProtocolIn_ = pub;

  start_receive();

  std::cout << "ret" << std::endl;
}

void UdpServer::start_receive() {

  socket_.async_receive_from(
      boost::asio::buffer(recv_buffer_), remote_endpoint_,
      boost::bind(&UdpServer::handle_receive, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

/**
 **  @throws throws exception if msg to send is larger than max_recv_bytes
 **/

void UdpServer::start_send(platooning::platoonProtocolOut msg) {
  start_send(msg.payload, msg.message_type);
}

void UdpServer::start_send(std::string message, int32_t message_type) {

  std::cout << "start send check len" << std::endl;

  if (message.length() + sizeof(message_type) + 1 > MAX_RECV_BYTES) {
    throw;
  }

  std::cout << "start send" << std::endl;

  memcpy(&message_type, send_buffer_.data(), sizeof(int32_t));
  message.copy(send_buffer_.data(), message.length(), 4);
  send_buffer_[sizeof(int32_t) + message.size()] = '\0';

  socket_.async_send_to(
      boost::asio::buffer(send_buffer_), remote_endpoint_,
      boost::bind(&UdpServer::handle_send, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));

}

void UdpServer::handle_receive(const boost::system::error_code &error,
                                      std::size_t /*bytes_transferred*/) {
  std::cout << "handl recv" << std::endl;

  if (!error || error == boost::asio::error::message_size) {

    std::cout << "handling receive" << std::endl;

    char buf[MAX_RECV_BYTES];

    socket_.receive(boost::asio::buffer(buf));

    char message_type_array[sizeof(int32_t)];
    memcpy(message_type_array, buf, sizeof(int32_t));

    int32_t message_type;
    sscanf(message_type_array, "%i", &message_type);

    char* ptr = &buf[sizeof(int32_t)-1];

    std::string str = ptr;

    platooning::platoonProtocolIn msg;

    switch (message_type) {
      case LV_BROADCAST:
      case FV_HEARTBEAT:
      case LV_REQUEST:
      case FV_REQUEST:
      case ACCEPT_RESPONSE:
      case REJECT_RESPONSE:
      case LEAVE_PLATOON:
        msg.payload = str;
        pub_platoonProtocolIn_.publish(msg);
        break;

      default:
        break;

    }

    start_receive();
  }
}

void UdpServer::handle_send(const boost::system::error_code &error,
                            std::size_t /*bytes_transferred*/) {
}
