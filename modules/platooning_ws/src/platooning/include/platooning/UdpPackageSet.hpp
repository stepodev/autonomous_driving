/**
 * @file include/UdpPackageSet.hpp
 * @author stepo
 * @date 23.03.2018
 * @brief Header file UdpPackage and UdpPackageSet classes
 *
 */

#ifndef PLATOONING_UDPPACKAGESET_HPP
#define PLATOONING_UDPPACKAGESET_HPP

#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <unordered_set>
#include <boost/array.hpp>

#define MAX_RECV_BYTES 1024

/**
 * @class UdpPackage
 * @brief Storage class for data related to one udp datagram
 *
 * Stores endpoint either sent to or received from,
 * callback of storage time handling class to be called after transmission is done,
 * callback original initiator of the transmission to be called by storage time handling class
 */
class UdpPackage {
  public:

	explicit UdpPackage(const boost::function<void(const boost::system::error_code &error,
		                                         std::size_t,
		                                         UdpPackage *)>& transmission_callback,
		                    const boost::function<void(const boost::system::error_code &error,
		                                std::size_t,
		                                std::shared_ptr<UdpPackage>)>& callee_callback);

	explicit UdpPackage(const boost::function<void(const boost::system::error_code &error,
		                                         std::size_t,
		                                         UdpPackage *)>& transmission_callback,
		                    const boost::function<void(const boost::system::error_code &error,
		                                std::size_t,
		                                std::shared_ptr<UdpPackage>)>& callee_callback,
		                    const boost::asio::ip::udp::endpoint &endpoint);

	boost::asio::ip::udp::endpoint endpoint_; /**< either endpoint package was received from or sent from */
	boost::array<char, MAX_RECV_BYTES> buffer_; /**< buffer with data of the updpackage */
	boost::function<void(const boost::system::error_code &error, std::size_t, UdpPackage *)> package_set_callback_; /**< callback to be called after transmission is done */
	boost::function<void(const boost::system::error_code &error, std::size_t, std::shared_ptr<UdpPackage>)> callee_callback_; /**< callback of original transmission initiator */
	void handle_transmission_done(const boost::system::error_code &error, size_t byte_count); /**< calls the storage manager class callback */
};

/**
 * @class UdpPackageSet
 * @brief Creates and manages lifetimes of UdpPackages. Provides a handler each UdpPackage calls to get removed from
 * set and free that memory. After that, calls the handler of the transmission initiator.
 */
class UdpPackageSet {

  public:
	/**
	 * Constructor for a package with data to be send. Requires endpoint to send to.
	 * @param send_callback callback once send is done
	 * @param to endpoint to send to
	 * @return pointer to UdpPackage object with all data sent.
	 */
	std::shared_ptr<UdpPackage> get_sendpackage(boost::function<void(const boost::system::error_code &error,
	                                                                 std::size_t,
	                                                                 std::shared_ptr<UdpPackage>)> send_callback,
	                                            const boost::asio::ip::udp::endpoint &to);
	/**
	 * Constructor for a package awaiting receipt of data.
	 * @param receive_callback callback once data is received
	 * @return pointer to UdpPackage object with all data received and endpoint it was received from.
	 */
	std::shared_ptr<UdpPackage> get_recvpackage(boost::function<void(const boost::system::error_code &error,
	                                                                 std::size_t bytes_recvd,
	                                                                 std::shared_ptr<UdpPackage>)> receive_callback);

  private:
	boost::mutex set_mtx_; /**< syncronizes access to the unordered set */
	std::unordered_set<std::shared_ptr<UdpPackage>> set_{};

	void handle_transmission_done(const boost::system::error_code &error,
	                              std::size_t size,
	                              UdpPackage *udp_package_ptr);
};

#endif //PLATOONING_UDPPACKAGESET_HPP
