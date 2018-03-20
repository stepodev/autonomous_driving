//
// Created by stepo on 1/21/18.
//


#include "platooning/UdpServer.hpp"

UdpServer::UdpServer(boost::function<void(std::pair<std::string, uint32_t>)> receive_callback,
                     udp::endpoint bind_endpoint,
                     udp::endpoint remote_endpoint) : io_work_(io_service_) {

	try {

		socket_ptr_ = std::unique_ptr<udp::socket>(new udp::socket(io_service_, udp::v4()));

		//reuse port
		typedef boost::asio::detail::socket_option::boolean<SOL_SOCKET, SO_REUSEPORT> reuse_port;
		socket_ptr_->set_option(reuse_port(true));

		//enable broadcast
		boost::asio::socket_base::broadcast enable_broadcast(true);
		socket_ptr_->set_option(enable_broadcast);

		//reuse port
		boost::asio::socket_base::reuse_address reuse_address(true);
		socket_ptr_->set_option(reuse_address);

		socket_ptr_->bind(bind_endpoint);

		send_endpoint_ = std::move(remote_endpoint);

		myport_ = bind_endpoint.port();
		find_own_ip();

		callback_ = std::move(receive_callback);

		start_receive();

		thread_pool_.create_thread([this] {
				io_service_.run();
				std::cout << "[UdpServer] IOSERVICE STOPPED" << std::endl;});
	} catch (std::exception &e) {
		std::cerr << "[UdpServer][constructor] threw " << e.what() << std::endl;
	}
}

UdpServer::~UdpServer() {
	shutdown();
}

void UdpServer::shutdown() {
	std::cout << "[UdpServer] shutdown called" << std::endl;
try { io_service_.stop(); } catch (std::exception &ex) {
		std::cerr << "[UdpServer] shutdown io_service_ threw " << ex.what() << std::endl;
	};
	try { socket_ptr_->close(); } catch (std::exception &ex) {
		std::cerr << "[UdpServer] shutdown socket threw " << ex.what() << std::endl;
	};
	try {
		thread_pool_.interrupt_all();
		thread_pool_.join_all();
	} catch (std::exception &ex) {
		std::cerr << "[UdpServer] shutdown threadpool threw " << ex.what() << std::endl;
	};
}

void UdpServer::find_own_ip() {

	udp::endpoint broadcast(boost::asio::ip::address_v4::broadcast(), 54566);

	boost::array<char, 5> buf = {'0', '1', '2'};
	udp::endpoint myself(udp::v4(), 54566);
	udp::socket mesock(io_service_, myself);

	thread_pool_.create_thread([&mesock, &myself, &buf] {
		mesock.receive_from(boost::asio::buffer(buf), myself);
	});

	socket_ptr_->send_to(boost::asio::buffer(buf), broadcast);

	while (myself.address() == udp::endpoint(udp::v4(), 54566).address()
		|| buf[0] != '0' || buf[1] != '1' || buf[2] != '2') {
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	}
	mesock.close();
	myaddress_ = myself.address();

	std::cout << "[UdpServer] bound to ip " << myaddress_.to_string() << ":" << myport_ << std::endl;

}

size_t UdpServer::write_to_sendbuffer(boost::array<char, MAX_RECV_BYTES> &target_buf,
                                      const std::string &message,
                                      const uint32_t &message_type) {

	try {
		memcpy(target_buf.data(), &message_type, sizeof(message_type));

		memcpy(target_buf.data() + sizeof(message_type), message.c_str(), message.length());

		target_buf[sizeof(uint32_t) + message.length()] = '\0';

		return sizeof(message_type) + message.length();
	} catch (std::exception &e) {
		std::cerr << "[UdpServer][write_to_sendbuffer] threw " << e.what() << std::endl;
	}
}

std::pair<std::string, uint32_t> UdpServer::read_from_recvbuffer(const boost::array<char, MAX_RECV_BYTES> &buf,
                                                                 size_t bytes_transferred) {
	try {
		uint32_t message_type;
		memcpy(&message_type, buf.data(), sizeof(uint32_t));

		//hopefully the whole string without the message
		std::string str(buf.begin() + sizeof(uint32_t), bytes_transferred - sizeof(uint32_t));

		//std::cout << "[UdpServer] recvd message type " << message_type << "\nmessage:" << str << std::endl;

		return std::pair<std::string, uint32_t>(str.data(), message_type);
	} catch (std::exception &e) {
		std::cerr << "[UdpServer][read_from_buffer] threw " << e.what() << std::endl;
		return std::pair<std::string, uint32_t>();
	}
}

void UdpServer::start_receive() {

	boost::function<void(const boost::system::error_code &, size_t, std::shared_ptr<UdpPackage>)> cbfun
		= boost::bind(boost::mem_fn(&UdpServer::handle_receive), this, _1, _2, _3);

	auto p = pending_packages_.get_recvpackage(cbfun);

	try {
		socket_ptr_->async_receive_from(
			boost::asio::buffer(p->buffer_), p->endpoint_,
			boost::bind(&UdpPackage::handle_transmission_done, p,
			            boost::asio::placeholders::error,
			            boost::asio::placeholders::bytes_transferred));
	} catch (std::exception &e) {
		std::cerr << "[UdpServer][start_receive] threw " << e.what() << std::endl;
	}
}

/**
 **  @throws throws exception if msg to send is larger than max_recv_bytes
 **/

void UdpServer::start_send(std::string message, uint32_t message_type) {

	//std::cout << "sending " << message << std::endl;

	if (message.length() + sizeof(message_type) + 1 > MAX_RECV_BYTES) {
		std::cerr << "[UdpServer] message max length exceeded" << std::endl;
	}

	boost::function<void(const boost::system::error_code &, size_t, std::shared_ptr<UdpPackage>)> cbfun
		= boost::bind(boost::mem_fn(&UdpServer::handle_send), this, _1, _2, _3);

	std::shared_ptr<UdpPackage> p = pending_packages_.get_sendpackage(cbfun, send_endpoint_);

	size_t bytes_written = 0;
	try {
		bytes_written = write_to_sendbuffer(p->buffer_, message, message_type);

		//std::cout << "srv sending " << p->buffer_.data() << std::endl;
	} catch (std::exception &ex) {
		std::cerr << "udpserver error stuffing sendbuffer " << ex.what() << std::endl;
		return;
	}

	try {
		socket_ptr_->async_send_to(
			boost::asio::buffer(p->buffer_, bytes_written), p->endpoint_,
			boost::bind(&UdpPackage::handle_transmission_done, p,
			            boost::asio::placeholders::error,
			            boost::asio::placeholders::bytes_transferred));
	}
	catch (std::exception &e) {
		std::cerr << "[UdpServer][start_send] threw " << e.what() << std::endl;

	}
	//std::cout << "srv send done" << std::endl;

}

void UdpServer::handle_receive(const boost::system::error_code &error,
                               std::size_t size,
                               std::shared_ptr<UdpPackage> package) {

	//wait for next, regardless of what happens
	start_receive();

	if (error) {
		std::cerr << "[udpserver] error during receive:" << error.message() << std::endl;
		return;
	}

	//ignore my own broadcasts except from test port
	if (filter_own_broadcasts_
		&& package->endpoint_.address() == myaddress_
		&& package->endpoint_.port() == myport_) { //testport
		//std::cout << "FILTERED" << msg_src_endpoint_.address() << ":" << msg_src_endpoint_.port() << std::endl;
		return;
	}

	try {
		//std::cout << "udpserv handl recv " << size << " bytes"
		//          <<  "\n" << msg_src_endpoint_.address() << "==>" << socket_ptr_->local_endpoint().address() << std::endl;

		std::pair<std::string, uint32_t> msgpair = read_from_recvbuffer(package->buffer_, size);
		callback_(msgpair);

		//std::cout << "srv recv done" << std::endl;

	} catch (std::exception &e) {
		std::cerr << "[UdpServer][handle_receive] threw " << e.what() << std::endl;
	}
}

void UdpServer::handle_send(const boost::system::error_code &error,
                            std::size_t size,
                            std::shared_ptr<UdpPackage> package) {

	if (error) {
		std::cerr << "[udpserver] error during sending:" << error.message() << std::endl;
		return;
	}

	//std::cout << "sent " << package->buffer_.data() << std::endl;
}

void UdpServer::set_filter_own_broadcasts(bool flag) {
	filter_own_broadcasts_ = flag;
}



