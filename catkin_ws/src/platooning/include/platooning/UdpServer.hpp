//
// Created by stepo on 1/21/18.
//

#ifndef PLATOONING_UDP_SERVER_HPP
#define PLATOONING_UDP_SERVER_HPP

#include <boost/asio.hpp>
#include <ros/ros.h>

#include "platooning/platoonProtocolOut.h"
#include <platooning/platoonProtocolIn.h>
#include "MessageTypes.hpp"

const int32_t MAX_RECV_BYTES = 1024;

using boost::asio::ip::udp;

class UdpServer {
public:

  UdpServer(boost::asio::io_service &io_service
      , boost::function<void(std::shared_ptr<std::vector<char>>)>
      , udp::endpoint bind_endpoint
      , udp::endpoint remote_endpoint);

  void start_send(platooning::platoonProtocolOut);

private:
  void start_receive();

  void start_send(std::string, int32_t message_type);

  void handle_receive(const boost::system::error_code &error,
                      std::size_t /*bytes_transferred*/);
  void handle_send(const boost::system::error_code &error
      , size_t);

  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  boost::array<char, MAX_RECV_BYTES> recv_buffer_;
  boost::array<char, MAX_RECV_BYTES> send_buffer_;

  boost::function<void(std::shared_ptr<std::vector<char>>)> callback;

};


#endif //PLATOONING_UDP_SERVER_HPP
