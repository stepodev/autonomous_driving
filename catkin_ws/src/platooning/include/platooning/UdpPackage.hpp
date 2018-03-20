//
// Created by stepo on 3/20/18.
//

#ifndef PLATOONING_UDPPACKAGE_HPP
#define PLATOONING_UDPPACKAGE_HPP

#include <boost/array.hpp>
#include <boost/function.hpp>
#include <boost/asio.hpp>
#include <iostream>

#include "platooning/PackageSet.hpp"

class UdpPackage {
  public:
	boost::asio::ip::udp::endpoint endpoint_;
	boost::array<char, 1024> buffer_;
	boost::function<void(const boost::system::error_code &error, std::size_t, std::shared_ptr<UdpPackage>)> receive_callback;
	void handle_recv_done(const boost::system::error_code &error, size_t);
	void handle_send_done(const boost::system::error_code &error, size_t);
	PackageSet* call_to_delete_;
};

#endif //PLATOONING_UDPPACKAGE_HPP
