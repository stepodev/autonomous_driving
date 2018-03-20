//
// Created by stepo on 3/20/18.
//

#ifndef PLATOONING_UDPPACKAGESET_HPP
#define PLATOONING_UDPPACKAGESET_HPP

#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <unordered_set>
#include <boost/array.hpp>

#define MAX_RECV_BYTES 1024

class UdpPackage {
  public:

	explicit UdpPackage(boost::function<void(const boost::system::error_code &error,
	                                         std::size_t,
	                                         UdpPackage *)> transmission_callback);

	explicit UdpPackage(boost::function<void(const boost::system::error_code &error,
	                                         std::size_t,
	                                         UdpPackage *)> transmission_callback,
	                    const boost::asio::ip::udp::endpoint &endpoint);

	boost::asio::ip::udp::endpoint endpoint_;
	boost::array<char, MAX_RECV_BYTES> buffer_;
	boost::function<void(const boost::system::error_code &error, std::size_t, UdpPackage *)> package_set_callback_;
	boost::function<void(const boost::system::error_code &error, std::size_t, std::shared_ptr<UdpPackage>)> callee_callback_;
	void handle_transmission_done(const boost::system::error_code &error, size_t byte_count);
};

class UdpPackageSet {

  public:
	std::shared_ptr<UdpPackage> get_sendpackage(boost::function<void(const boost::system::error_code &error,
	                                                                 std::size_t,
	                                                                 std::shared_ptr<UdpPackage>)> send_callback,
	                                            const boost::asio::ip::udp::endpoint &to);
	std::shared_ptr<UdpPackage> get_recvpackage(boost::function<void(const boost::system::error_code &error,
	                                                                 std::size_t bytes_recvd,
	                                                                 std::shared_ptr<UdpPackage>)> receive_callback);

  private:
	boost::mutex set_mtx_;
	std::unordered_set<std::shared_ptr<UdpPackage>> set_{};

	void handle_transmission_done(const boost::system::error_code &error,
	                              std::size_t size,
	                              UdpPackage *udp_package_ptr);
};

#endif //PLATOONING_UDPPACKAGESET_HPP
