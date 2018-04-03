/**
 * @file src/UdpServer.cpp
 * @author stepo
 * @date 23.03.2018
 * @brief Implementation of UdpServer class
 *
 */

#include "platooning/UdpServer.hpp"

UdpServer::UdpServer(const boost::function<void(boost::shared_ptr<std::pair<std::string, uint32_t>>)>& receive_callback,
                     const udp::endpoint &bind_endpoint,
                     const udp::endpoint &remote_endpoint) : io_work_(io_service_) {

	try {
		//create socket, set it to reuse so several udpservers can listen to one port concurrently, enable broadcasts
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

		//set endpoints and determine own ip
		socket_ptr_->bind(bind_endpoint);

		send_endpoint_ = remote_endpoint;

		myport_ = bind_endpoint.port();

		find_own_ip();

		//save the callback to call in case of receives and start listening
		callback_ = receive_callback;

		start_receive();

		thread_pool_.create_thread([this] {
			io_service_.run();
			std::cout << "[UdpServer] IOSERVICE STOPPED" << std::endl;
		});

		is_up = true;
	} catch (std::exception &e) {
		std::cerr << "[UdpServer][constructor] threw " << e.what() << std::endl;
	}
}

UdpServer::~UdpServer() {
	shutdown();
}

void UdpServer::shutdown() {
	std::cout << "[UdpServer] shutdown called" << std::endl;

	if( !is_up ) {
		return;
	}

	try { io_service_.stop(); } catch (std::exception &ex) {
		std::cerr << "[UdpServer] shutdown io_service_ threw " << ex.what() << std::endl;
	};
	try {
		thread_pool_.interrupt_all();
		thread_pool_.join_all();
	} catch (std::exception &ex) {
		std::cerr << "[UdpServer] shutdown threadpool threw " << ex.what() << std::endl;
	};
	try { socket_ptr_->close(); } catch (std::exception &ex) {
		std::cerr << "[UdpServer] shutdown socket threw " << ex.what() << std::endl;
	};

	is_up = false;
}

/**
 * @brief sends a specially crafted package on a specially crafted port to and listens to that port to determine our ip
 */
void UdpServer::find_own_ip() {

	try {
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
	} catch (std::exception &ex) {
		std::cerr << "[UdpServer] find_own_ip threadpool threw " << ex.what() << std::endl;
	}

}

size_t UdpServer::write_to_sendbuffer(boost::array<char, 1024> &target_buf,
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

	return sizeof(message_type) + message.length();
}

boost::shared_ptr<std::pair<std::string, uint32_t>> UdpServer::read_from_recvbuffer(const boost::array<char,
                                                                                                       MAX_RECV_BYTES> &buf,
                                                                                    size_t bytes_transferred) {

	uint32_t message_type;
	memcpy(&message_type, buf.data(), sizeof(uint32_t));

	//hopefully parses the whole string without the message
	std::string str(buf.begin() + sizeof(uint32_t), bytes_transferred - sizeof(uint32_t));


	auto msgpair = boost::shared_ptr<std::pair<std::string, uint32_t>>(new std::pair<std::string, uint32_t>());
	msgpair->first = str;
	msgpair->second = message_type;

	return msgpair;
}

/**
 * @brief Hands over a callback function to the UdpPackageSet object and gets a UdpPackage with a buffer the data
 * is going to be in.
 */
void UdpServer::start_receive() {
	try {
		boost::function<void(const boost::system::error_code &, size_t, std::shared_ptr<UdpPackage>)> cbfun
			= boost::bind(boost::mem_fn(&UdpServer::handle_receive), this, _1, _2, _3);

		auto p = pending_packages_.get_recvpackage(cbfun);

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
 *  @brief Hands a callback function to UdpPackageSet that gets called on when send is done, gets a UdpPackage pointer
 *  with the buffer the data to send must be written to.
 *  @throws throws exception if msg to send is larger than max_recv_bytes
 */
void UdpServer::start_send(const std::string &message, const uint32_t &message_type) {

	if (message.length() + sizeof(message_type) + 1 > MAX_RECV_BYTES) {
		std::cerr << "[UdpServer] message max length exceeded" << std::endl;
	}

	try {
		boost::function<void(const boost::system::error_code &, size_t, std::shared_ptr<UdpPackage>)> cbfun
			= boost::bind(boost::mem_fn(&UdpServer::handle_send), this, _1, _2, _3);

		std::shared_ptr<UdpPackage> p = pending_packages_.get_sendpackage(cbfun, send_endpoint_);

		size_t bytes_written = 0;

		bytes_written = write_to_sendbuffer(p->buffer_, message, message_type);

		socket_ptr_->async_send_to(
			boost::asio::buffer(p->buffer_, bytes_written), p->endpoint_,
			boost::bind(&UdpPackage::handle_transmission_done, p,
			            boost::asio::placeholders::error,
			            boost::asio::placeholders::bytes_transferred));
	}
	catch (std::exception &e) {
		std::cerr << "[UdpServer][start_send] threw " << e.what() << std::endl;

	}
}

/**
 * @brief Immediately starts another receive after being called, filters the package if necessary and calls the
 * receive callback of the owner
 */
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
		&& package->endpoint_.port() == myport_) {
		return;
	}

	try {
		boost::shared_ptr<std::pair<std::string, uint32_t>> msgpair = read_from_recvbuffer(package->buffer_, size);
		callback_(msgpair);

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
}

void UdpServer::set_filter_own_broadcasts(bool flag) {
	filter_own_broadcasts_ = flag;
}


