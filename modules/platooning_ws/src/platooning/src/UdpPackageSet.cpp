//
// Created by stepo on 3/20/18.
//

#include "platooning/UdpPackageSet.hpp"

/**
 * @brief Remove the object from the pending packages set and calls it's callback
 */
void UdpPackageSet::handle_transmission_done(const boost::system::error_code &error,
                                             std::size_t size,
                                             UdpPackage *udp_package_ptr) {

	boost::mutex::scoped_lock l(set_mtx_);

	std::shared_ptr<UdpPackage> ptr;

	for (auto &mem : set_) {
		if (mem.get() == udp_package_ptr) {
			ptr = mem;
			set_.erase(mem);
			break;
		}
	}

	l.unlock();

	udp_package_ptr->callee_callback_(error, size, ptr);
}


std::shared_ptr<UdpPackage> UdpPackageSet::get_sendpackage(boost::function<void(const boost::system::error_code &error,
                                                                                std::size_t,
                                                                                std::shared_ptr<UdpPackage>)> send_callback,
                                                           const boost::asio::ip::udp::endpoint &to) {

	boost::function<void(const boost::system::error_code &, size_t, UdpPackage *)> package_set_callback
		= boost::bind(boost::mem_fn(&UdpPackageSet::handle_transmission_done), this, _1, _2, _3);

	auto psend = std::shared_ptr<UdpPackage>(new UdpPackage(package_set_callback,
	                                                        send_callback,
	                                                        to));
	return psend;
}

std::shared_ptr<UdpPackage> UdpPackageSet::get_recvpackage(boost::function<void(const boost::system::error_code &error,
                                                                                std::size_t,
                                                                                std::shared_ptr<UdpPackage>)> receive_callback) {

	boost::function<void(const boost::system::error_code &, size_t, UdpPackage *)> package_set_callback
		= boost::bind(boost::mem_fn(&UdpPackageSet::handle_transmission_done), this, _1, _2, _3);

	auto psend = std::shared_ptr<UdpPackage>(new UdpPackage(package_set_callback,
	                                                        receive_callback ));

	boost::mutex::scoped_lock l(set_mtx_);
	set_.insert(psend);
	l.unlock();

	return psend;
}

UdpPackage::UdpPackage(const boost::function<void(const boost::system::error_code &error,
                                                  std::size_t,
                                                  UdpPackage *)> &transmission_callback,
                       const boost::function<void(const boost::system::error_code &error,
                                                  std::size_t,
                                                  std::shared_ptr<UdpPackage>)> &callee_callback) {

	package_set_callback_ = transmission_callback;
	callee_callback_ = callee_callback;
}

void UdpPackage::handle_transmission_done(const boost::system::error_code &error, size_t size) {
	package_set_callback_(error, size, this);
}

UdpPackage::UdpPackage(const boost::function<void(const boost::system::error_code &error,
                                                  std::size_t,
                                                  UdpPackage *)> &transmission_callback,
                       const boost::function<void(const boost::system::error_code &error,
                                                  std::size_t,
                                                  std::shared_ptr<UdpPackage>)> &callee_callback,
                       const boost::asio::ip::udp::endpoint &endpoint) {

	package_set_callback_ = transmission_callback;
	callee_callback_ = callee_callback;
	endpoint_ = endpoint;
}