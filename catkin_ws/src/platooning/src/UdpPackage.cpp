//
// Created by stepo on 3/20/18.
//
#include "platooning/UdpPackage.hpp"

void UdpPackage::handle_send_done(const boost::system::error_code &error,
                                             std::size_t bytes /*bytes_transferred*/) {
	if (error) {
		std::cerr << "server send error " << error.message() << std::endl;
	} else {
		//std::cout << "server sent " << bytes << " bytes" << std::endl;
	}

	call_to_delete_->remove_package(this);
}

void UdpPackage::handle_recv_done(const boost::system::error_code &error, size_t size) {
	auto p = call_to_delete_->get_package(this);

	try {
		p->receive_callback( error, size, p );
	}catch( std::exception& e ) {
		std::cerr << "receive callback error " << e.what() << std::endl;
	}

	call_to_delete_->remove_package(this);
}