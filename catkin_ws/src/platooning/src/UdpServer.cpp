//
// Created by stepo on 1/21/18.
//


#include <chrono>
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

		thread_pool_.create_thread([this] { this->io_service_.run(); });
	} catch (std::exception &e) {
		std::cerr << "[UdpServer][constructor] threw " << e.what() << std::endl;
	}
}

UdpServer::~UdpServer() {
	shutdown();
}

void UdpServer::shutdown() {
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

	auto p = PackageSet::get_recvpackage(cbfun);

	try {
		socket_ptr_->async_receive_from(
			boost::asio::buffer(p->buffer_), p->endpoint_,
			boost::bind(&UdpServer::UdpPackage::handle_recv_done, p,
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

	//std::cout << "start send check len of msg \"" << message << "\" type " << message_type << std::endl;

	if (message.length() + sizeof(message_type) + 1 > MAX_RECV_BYTES) {
		throw;
	}

	std::shared_ptr<UdpPackage> p = PackageSet::get_sendpackage(send_endpoint_);

	size_t bytes_written = 0;
	try {
		bytes_written = write_to_sendbuffer(p->buffer_, message, message_type);

		//std::cout << "srv sending " << buffer_.data() << std::endl;
	} catch (std::exception &ex) {
		std::cerr << "udpserver error stuffing sendbuffer " << ex.what() << std::endl;
		return;
	}

	try {
		socket_ptr_->async_send_to(
			boost::asio::buffer(p->buffer_, bytes_written), p->endpoint_,
			boost::bind(&UdpServer::UdpPackage::handle_send_done, p,
			            boost::asio::placeholders::error,
			            boost::asio::placeholders::bytes_transferred));
	}
	catch (std::exception &e) {
		std::cerr << "[UdpServer][start_send] threw " << e.what() << std::endl;

	}
	//std::cout << "srv send done" << std::endl;

}

void UdpServer::handle_receive(const boost::system::error_code &error, std::size_t size, std::shared_ptr<UdpPackage> package ) {

	if (error) {
		std::cerr << "[udpserver] error during handling receive:" << error.message() << std::endl;
		return;
	}

	//ignore my own broadcasts except from test port
	if (filter_own_broadcasts_
		&& package->endpoint_.address() == myaddress_
		&& package->endpoint_.port() == myport_) { //testport
		//std::cout << "FILTERED" << msg_src_endpoint_.address() << ":" << msg_src_endpoint_.port() << std::endl;
		start_receive();
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

	start_receive();

}

void UdpServer::UdpPackage::handle_send_done(const boost::system::error_code &error,
                                             std::size_t bytes /*bytes_transferred*/) {
	if (error) {
		std::cerr << "server send error " << error.message() << std::endl;
	} else {
		//std::cout << "server sent " << bytes << " bytes" << std::endl;
	}

	UdpServer::PackageSet::remove_package(this);
}

void UdpServer::UdpPackage::handle_recv_done(const boost::system::error_code &error, size_t size) {
	auto p = UdpServer::PackageSet::get_package(this);

	try {
		p->receive_callback( error, size, p );
	}catch( std::exception& e ) {
		std::cerr << "receive callback error " << e.what() << std::endl;
	}

	UdpServer::PackageSet::remove_package(this);
}

void UdpServer::set_filter_own_broadcasts(bool flag) {
	filter_own_broadcasts_ = flag;
}

void UdpServer::PackageSet::remove_package(UdpServer::UdpPackage *p) {
	boost::mutex::scoped_lock l(send_set_mtx_);

	for (auto &mem : set_) {
		if (mem.get() == p) {
			set_.erase(mem);
			break;
		}
	}
	//std::cout << "[UdpServer][PackageSet][remove_package] size is " << set_.size() << std::endl;
}

std::shared_ptr<UdpServer::UdpPackage> UdpServer::PackageSet::get_sendpackage( const udp::endpoint& to ) {
	auto psend = std::shared_ptr<UdpPackage>(new UdpPackage());
	psend->endpoint_ = to;
	boost::mutex::scoped_lock l(send_set_mtx_);
	set_.insert(psend);
	//std::cout << "[UdpServer][PackageSet][get_sendpackage] size is " << set_.size() << std::endl;
	return psend;
}

std::shared_ptr<UdpServer::UdpPackage> UdpServer::PackageSet::get_recvpackage(boost::function<void(const boost::system::error_code &error, std::size_t, std::shared_ptr<UdpPackage>)> receive_callback ) {

	auto psend = std::shared_ptr<UdpPackage>(new UdpPackage());
	psend->receive_callback = std::move(receive_callback);

	boost::mutex::scoped_lock l(recv_set_mtx_);
	set_.insert(psend);
	//std::cout << "[UdpServer][PackageSet][get_sendpackage] size is " << set_.size() << std::endl;
	return psend;
}

std::shared_ptr<UdpServer::UdpPackage> UdpServer::PackageSet::get_package(UdpServer::UdpPackage * p) {

	std::shared_ptr<UdpPackage> pret;

	for (auto &mem : set_) {
		if (mem.get() == p) {
			pret = mem;
			break;
		}
	}

	return pret;
}

boost::mutex UdpServer::PackageSet::send_set_mtx_;
boost::mutex UdpServer::PackageSet::recv_set_mtx_;
std::unordered_set<std::shared_ptr<UdpServer::UdpPackage>> UdpServer::PackageSet::set_;


