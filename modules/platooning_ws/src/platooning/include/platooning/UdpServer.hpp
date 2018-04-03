/**
 * @file testing/include/UdpServer.hpp
 * @author stepo
 * @date 23.03.2018
 * @brief Header file UdpServer class
 *
 */


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

/**
 * @class UdpServer
 * @brief Provides functionality for asyncronous sends and receives.
 *
 * @bugs craps out at sent==recv 9000packets per second, 1.6mBit/s?
 * Maybe preallocated buffers in ring structure?
 * increasing socket recv buffer doest work.
 * wireshark says that sends are limited
 * valgrind callgrind doesnt list udpserver functions as high time consumers
 */
class UdpServer {
  public:
	UdpServer() = delete;

	/**
	 * @brief Constructs UdpServer object with a callback in case of a received message,
	 *        an endpoint the server listens to and an endpoint the server always sends to,
	 * @param receive_callback function object for callback function
	 * @param bind_endpoint endpoint the server listens to
	 * @param remote_endpoint endpoint the server sends to
	 */
	UdpServer(const boost::function<void(boost::shared_ptr<std::pair<std::string, uint32_t>>)>& receive_callback,
	          const udp::endpoint &bind_endpoint,
	          const udp::endpoint &remote_endpoint);

	/**
	 * @brief Deconstructor calls shutdown.
	 */
	~UdpServer();

	/**
	 * @brief Prepends the given uint32_t as a header to the string message and sends it as a udp package to the
	 * given endpoint and starts an async send
	 * @param message a string with the message
	 * @param message_type a unsigned 32bit int with the header type
	 */
	void start_send(const std::string &message, const uint32_t &message_type);

	/**
	 * @brief Handler which is called when the async send is done.
	 * @param error errorcode of the send process
	 * @param size number of bytes sent
	 * @param package pointer to udppackage object as it was sent
	 */
	void handle_send(const boost::system::error_code &error, std::size_t size, std::shared_ptr<UdpPackage> package);

	/**
	 * @brief Determines whether broadcasts from same ip and port should be filtered out or handed up to the receive handler
	 * @param filter
	 */
	void set_filter_own_broadcasts(bool filter);

	/**
	 * @brief Stops threads, pending transactions, io_services, etc.
	 */
	void shutdown();

  private:

	/**
	 * @brief starts listener on the given endpoint
	 */
	void start_receive();

	/**
	 * @brief Handles receives on the listened to endpoint
	 * @param error errorcode for the transaction
	 * @param size number of bytes received
	 */
	void handle_receive(const boost::system::error_code &error, std::size_t size, std::shared_ptr<UdpPackage>);

	/**
	 * @brief Prepares buffer with data that is to be sent. Writes the header bytes and appends the message string
	 * @param target_buf address of the buffer to write to
	 * @param message message to be sent
	 * @param message_type header type to be prepended
	 * @return number of bytes written
	 */
	size_t write_to_sendbuffer(boost::array<char, 1024> &target_buf,
	                           const std::string &message,
	                           const uint32_t &message_type);

	/**
	 * @brief Reads message and messagetype from a given buffer. Interprets the first sizeof(uint32_t) bytes as the header,
	 * the rest as a string
	 * @param buf address to the buffer with the data to read
	 * @param bytes_transferred number of bytes received
	 * @return pointer to a pair with the message and the message type
	 */
	boost::shared_ptr<std::pair<std::string, uint32_t>> read_from_recvbuffer(const boost::array<char,
	                                                                                            MAX_RECV_BYTES> &buf,
	                                                                         size_t bytes_transferred);

	std::unique_ptr<udp::socket> socket_ptr_; /**< pointer to the udpserver */

	boost::asio::io_service io_service_;    /**< waits for async work to finish */
	boost::asio::io_service::work io_work_; /**< work object for io_service to ensure io_service doesn't return */
	boost::thread_group thread_pool_; /**< pool of threads, usually just the io_service worker thread */

	udp::endpoint send_endpoint_; /**< endpoint that is being sent to */

	boost::asio::ip::address myaddress_; /**< own ip address so we can filter our own broadcasts */
	unsigned short myport_; /**< own port so we can filter our own broadcasts */
	bool filter_own_broadcasts_ = true; /**< filter own broadcast flag */
	bool is_up = false; /**< connection status of udpserver */
	UdpPackageSet pending_packages_; /**< set of pending packages awaiting async receipt or send to finish*/

	boost::function<void(boost::shared_ptr<std::pair<std::string, uint32_t>>)>
		callback_; /**< callback to execute in case of message receipt */

	/***
	 * @brief broadcast message to ourselves, so we can get our ip from the recvd datagram
	 */
	void find_own_ip();
};

#endif //PLATOONING_UDPSERVER_HPP
