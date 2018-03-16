//
// Created by stepo on 1/21/18.
//

#ifndef PLATOONING_UDPSERVER_HPP
#define PLATOONING_UDPSERVER_HPP

#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/array.hpp>
#include <boost/thread/mutex.hpp>
#include <unordered_set>
#include <memory>
#include <chrono>
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

	class UdpPackage {
	  public:
		udp::endpoint endpoint_;
		boost::array<char, MAX_RECV_BYTES> buffer_;
		boost::function<void(const boost::system::error_code &error, std::size_t, std::shared_ptr<UdpPackage>)> receive_callback;
		long timestamp = std::chrono::system_clock::now().time_since_epoch().count();
		void handle_recv_done(const boost::system::error_code &error, size_t);
		void handle_send_done(const boost::system::error_code &error, size_t);
	};

	class PackageSet {
	  private:
		static boost::mutex send_set_mtx_;
		static boost::mutex recv_set_mtx_;
		static std::unordered_set<std::shared_ptr<UdpPackage>> set_;

	  public:
		static std::shared_ptr<UdpPackage> get_sendpackage(const udp::endpoint& to);
		static std::shared_ptr<UdpPackage> get_recvpackage(boost::function<void(const boost::system::error_code &error, std::size_t bytes_recvd, std::shared_ptr<UdpPackage>)> receive_callback);
		static std::shared_ptr<UdpPackage> get_package(UdpPackage* );

		static void remove_package(UdpPackage *p);

	};

	void start_receive();
	void handle_receive(const boost::system::error_code &error, std::size_t, std::shared_ptr<UdpPackage> package);


	size_t write_to_sendbuffer(boost::array<char, MAX_RECV_BYTES> &target_buf,
	                           const std::string &message,
	                           const uint32_t &message_type);

	std::pair<std::string, uint32_t> read_from_recvbuffer(const boost::array<char, MAX_RECV_BYTES> &buf,
		                                                      size_t bytes_transferred);

	std::unique_ptr<udp::socket> socket_ptr_;
	boost::asio::io_service io_service_;
	boost::asio::io_service::work io_work_;
	boost::thread_group thread_pool_;

	udp::endpoint send_endpoint_;

	boost::asio::ip::address myaddress_;
	unsigned short myport_;
	bool filter_own_broadcasts_ = true;

	boost::function<void(std::pair<std::string, uint32_t>)> callback_;
	/***
	 * @brief broadcast message to ourselves, so we can get our ip from the recvd datagram
	 */
	void find_own_ip();
};

#endif //PLATOONING_UDPSERVER_HPP
