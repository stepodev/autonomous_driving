//
// Created by stepo on 1/21/18.
//

#ifndef PLATOONING_UDP_SERVER_HPP
#define PLATOONING_UDP_SERVER_HPP

#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/array.hpp>
#include <utility>

#include "MessageTypes.hpp"

const uint32_t MAX_RECV_BYTES = 1024;

using boost::asio::ip::udp;

class UdpServer {
public:
  UdpServer() = delete;

  UdpServer(boost::function<void(std::pair<std::string, uint32_t>)> receive_callback, udp::endpoint bind_endpoint,
              udp::endpoint remote_endpoint);
  ~UdpServer();

  void start_send(std::string, uint32_t message_type);
  void set_filter_own_broadcasts(bool filter);

  void shutdown();

private:
  void start_receive();

  void handle_receive(const boost::system::error_code &error,
                      std::size_t /*bytes_transferred*/);
  void handle_send(const boost::system::error_code &error
      , size_t);

  size_t write_to_sendbuffer(const std::string &message, const uint32_t &message_type);
  std::pair<std::string, uint32_t> read_from_recvbuffer(size_t bytes_transferred);

  std::unique_ptr<udp::socket> socket_ptr_;
  boost::asio::io_service io_service_;
  boost::thread_group thread_pool_;

  udp::endpoint send_endpoint_;
  udp::endpoint msg_src_endpoint_;
  boost::array<char, MAX_RECV_BYTES> recv_buffer_;
  boost::array<char, MAX_RECV_BYTES> send_buffer_;
  boost::asio::ip::address myaddress_;
  bool filter_own_broadcasts_ = true;

  boost::function<void(std::pair<std::string, uint32_t>)> callback_;
  /***
   * @brief broadcast message to ourselves, so we can get our ip from the recvd datagram
   */
  void find_own_ip();
};


#endif //PLATOONING_UDP_SERVER_HPP
