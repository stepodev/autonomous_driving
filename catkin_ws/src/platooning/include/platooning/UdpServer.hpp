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

  UdpServer(boost::asio::io_service &io_service, ros::Publisher pub);

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
  ros::Publisher pub_platoonProtocolIn_; /* publishes received platooning messages */
  ros::Publisher pub_communicationMessageIn_; /* publishes received communication messages */


};


#endif //PLATOONING_UDP_SERVER_HPP
