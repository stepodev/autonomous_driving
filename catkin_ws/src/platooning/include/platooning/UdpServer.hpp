//
// Created by stepo on 1/21/18.
//

#ifndef PLATOONING_UDPSERVER_HPP
#define PLATOONING_UDPSERVER_HPP

#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/array.hpp>

#include <memory>
#include <chrono>
#include <utility>

#include "UdpPackageSet.hpp"
#include "MessageTypes.hpp"

using boost::asio::ip::udp;

class UdpServer {
  public:
	UdpServer() = delete;

	UdpServer(boost::function<void(boost::shared_ptr<std::pair<std::string, uint32_t>>)> receive_callback,
	          const udp::endpoint &bind_endpoint,
	          const udp::endpoint &remote_endpoint);
	~UdpServer();

	void start_send(std::string, uint32_t message_type);
	void handle_send(const boost::system::error_code &error, std::size_t size, std::shared_ptr<UdpPackage> package);
	void set_filter_own_broadcasts(bool filter);

	void shutdown();

  private:
	void start_receive();
	void handle_receive(const boost::system::error_code &error, std::size_t size, std::shared_ptr<UdpPackage>);

	size_t write_to_sendbuffer(boost::array<char, 1024> &target_buf,
	                           const std::string &message,
	                           const uint32_t &message_type);

	boost::shared_ptr<std::pair<std::string, uint32_t>> read_from_recvbuffer(const boost::array<char,
	                                                                                            MAX_RECV_BYTES> &buf,
	                                                                         size_t bytes_transferred);

	std::unique_ptr<udp::socket> socket_ptr_;
	boost::asio::io_service io_service_;
	boost::asio::io_service::work io_work_;
	boost::thread_group thread_pool_;

	udp::endpoint send_endpoint_;

	boost::asio::ip::address myaddress_;
	unsigned short myport_;
	bool filter_own_broadcasts_ = true;
	UdpPackageSet pending_packages_;

	boost::function<void(boost::shared_ptr<std::pair<std::string, uint32_t>>)> callback_;
	/***
	 * @brief broadcast message to ourselves, so we can get our ip from the recvd datagram
	 */
	void find_own_ip();
};

#endif //PLATOONING_UDPSERVER_HPP
