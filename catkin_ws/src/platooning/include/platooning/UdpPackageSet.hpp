//
// Created by stepo on 3/20/18.
//

#ifndef PLATOONING_UDPPACKAGESET_HPP
#define PLATOONING_UDPPACKAGESET_HPP

#include "platooning/UdpPackage.hpp
#include <boost/thread/mutex.hpp>"
#include <unordered_set>

class UdpPackageSet {

	  private:
		boost::mutex send_set_mtx_;
		boost::mutex recv_set_mtx_;
		std::unordered_set<std::shared_ptr<UdpPackage>> set_{};

	  public:
		std::shared_ptr<UdpPackage> get_sendpackage(const boost::asio::ip::udp::endpoint& to);
		std::shared_ptr<UdpPackage> get_recvpackage(boost::function<void(const boost::system::error_code &error, std::size_t bytes_recvd, std::shared_ptr<UdpPackage>)> receive_callback);
		std::shared_ptr<UdpPackage> get_package(UdpPackage* );

		void remove_package(UdpPackage *p);
};

#endif //PLATOONING_UDPPACKAGESET_HPP
