//
// Created by stepo on 1/21/18.
//


#include "UdpServer.hpp"

UdpServer::UdpServer(boost::function<void(std::pair<std::string, int32_t>)> receive_callback,
                     udp::endpoint bind_endpoint,
                     udp::endpoint remote_endpoint) {

  try {

    socket_ptr_ = std::unique_ptr<udp::socket>(new udp::socket(io_service_,udp::v4()));

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

    find_own_ip();
    std::cout << "i am " << myaddress_.to_string() << std::endl;

    callback_ = std::move(receive_callback);

    start_receive();

    io_thread_ = boost::thread([this] { this->io_service_.run(); });
  }catch (std::exception &e) {
    std::cerr << "[UdpServer][constructor] threw " << e.what() << std::endl;
  }
}


UdpServer::~UdpServer() {
  socket_ptr_->close();
  io_service_.stop();
  io_thread_.interrupt();
}

void UdpServer::find_own_ip() {

  udp::endpoint broadcast(boost::asio::ip::address_v4::broadcast(),54566);

  boost::array<char, 5> buf = { '0','1','2'};
  udp::endpoint myself( udp::v4(),54566);
  udp::socket mesock( io_service_, myself);

  boost::thread t = boost::thread( [&mesock, &myself, &buf] {
    mesock.receive_from(boost::asio::buffer(buf), myself);
  });

  socket_ptr_->send_to(boost::asio::buffer(buf),broadcast);

  while( myself.address() == udp::endpoint(udp::v4(),54566).address()
      || buf[0] != '0' || buf[1] != '1' || buf[2] != '2' ) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }
  mesock.close();
  myaddress_ = myself.address();

}

size_t UdpServer::write_to_sendbuffer(const std::string &message, const int32_t &message_type) {

  try {
    memcpy(send_buffer_.data(), &message_type, sizeof(message_type));

    memcpy(send_buffer_.data() + sizeof(message_type), message.c_str(), message.length());

    send_buffer_[sizeof(int32_t) + message.length()] = '\0';

    return sizeof(message_type) + message.length();
  }catch (std::exception &e) {
    std::cerr << "[UdpServer][write_to_sendbuffer] threw " << e.what() << std::endl;
  }
}

std::pair<std::string, int32_t> UdpServer::read_from_recvbuffer( size_t bytes_transferred) {
  try {
    int32_t message_type;
    memcpy(&message_type, recv_buffer_.data(), sizeof(int32_t));

    //hopefully the whole string without the message
    std::string str(recv_buffer_.begin() + sizeof(int32_t), bytes_transferred - sizeof(int32_t));

    std::cout << "[UdpServer] recvd message type " << message_type << "\nmessage:" << str << std::endl;

    return std::pair<std::string, int32_t>(str.data(), message_type);
  } catch (std::exception &e) {
    std::cerr << "[UdpServer][read_from_buffer] threw " << e.what() << std::endl;
    return std::pair<std::string, int32_t>();
  }
}

void UdpServer::start_receive() {
  try {
    socket_ptr_->async_receive_from(
        boost::asio::buffer(recv_buffer_), msg_src_endpoint_,
        boost::bind(&UdpServer::handle_receive, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
    std::cout << "server started async recv" << std::endl;
  }catch (std::exception &e) {
    std::cerr << "[UdpServer][start_receive] threw " << e.what() << std::endl;
  }

}

/**
 **  @throws throws exception if msg to send is larger than max_recv_bytes
 **/

void UdpServer::start_send(std::string message, int32_t message_type) {

  std::cout << "start send check len of msg \"" << message << "\" type " << message_type << std::endl;

  if (message.length() + sizeof(message_type) + 1 > MAX_RECV_BYTES) {
    throw;
  }

  size_t bytes_written = 0;
  try {
    bytes_written = write_to_sendbuffer(message, message_type);

    std::cout << "srv sending " << send_buffer_.data() << std::endl;
  } catch (std::exception &ex) {
    std::cerr << "udpserver error stuffing sendbuffer " << ex.what() << std::endl;
  }

  try {
    socket_ptr_->async_send_to(
        boost::asio::buffer(send_buffer_,bytes_written), send_endpoint_,
        boost::bind(&UdpServer::handle_send, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
  }
  catch (std::exception &e) {
    std::cerr << "[UdpServer][start_send] threw " << e.what() << std::endl;
  }
  std::cout << "srv send done" << std::endl;

}

void UdpServer::handle_receive(const boost::system::error_code &error,
                               std::size_t size /*bytes_transferred*/) {

  //ignore my own broadcasts except from test port
  if( filter_own_broadcasts_
     && msg_src_endpoint_.address() == myaddress_
     && msg_src_endpoint_.port() == 10000 ) { //testport
    std::cout << "FILTERED" << msg_src_endpoint_.address() << ":" << msg_src_endpoint_.port() << std::endl;
    return;
  }

  try {
    std::cout << "udpserv handl recv " << size << " bytes"
              <<  "\n" << msg_src_endpoint_.address() << "==>" << socket_ptr_->local_endpoint().address() << std::endl;

    if (!error || error == boost::asio::error::message_size) {

      callback_(read_from_recvbuffer(size));

      start_receive();

      std::cout << "srv recv done" << std::endl;
    } else {
      std::cerr << "[udpserver] error during handling receive:" << error.message();
    }
  }catch (std::exception &e) {
    std::cerr << "[UdpServer][handle_receive] threw " << e.what() << std::endl;
  }

}

void UdpServer::handle_send(const boost::system::error_code &error,
                            std::size_t bytes /*bytes_transferred*/) {
  if (error) {
    std::cerr << "server handlesend eror " << error.message() << std::endl;
  } else {
    std::cout << "server sent " << bytes << " bytes" << std::endl;
  }
}

void UdpServer::set_filter_own_broadcasts(bool flag) {
  filter_own_broadcasts_ = flag;
}


