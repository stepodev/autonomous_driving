//******************************************************************
// file:       platoon/wifi.cpp
// purpose:    ros-node for access to wireless networking
// license:    ?
// maintainer: Bruno Lemke - lemkebru@informatik.hu-berlin.de
// created:    2017/12/11
//******************************************************************

//******************************************************************
// WIFI
//******************************************************************

#include <algorithm>

#include <pluginlib/class_list_macros.h>

#include <config.hpp>
#include <wifi.hpp>

using namespace platoon::wifi;
namespace asio = boost::asio;
using namespace asio::ip;

//******************************************************************
// NetworkHandler
//******************************************************************

//------ ~NetworkHandler ------
NetworkHandler::~NetworkHandler() {
  DBG("NetworkHandler::~NetworkHandler");

  io_.stop();
  io_worker_.join();

  INFO("Shutdown network service");
}

//------ onInit ------
void NetworkHandler::onInit() {
  DBG("NetworkHandler::onInit");

  port = static_cast<uint16_t>(
      getPrivateNodeHandle().param<int>("/vehicle/port", 10000));
  ip_address = getPrivateNodeHandle().param<std::string>("/vehicle/ip_address",
                                                         "10.42.43.11");

  INFO("Parameters set: port=%hd ip_address=%s", port, ip_address.c_str());

  initSockets();
  INFO("Initialized network sockets");

  installNetworkHandler();
  INFO("Installed network handler");

  startNetworkService();
  INFO("Network service started");

  car_subscriber_ = getPrivateNodeHandle().subscribe(
      "protocol_send", 2, &NetworkHandler::recvFromBus, this);
  INFO("Networh handler has subscribed to car bus");

  network_subscription_service_ = getMTNodeHandle().advertiseService(
      "subscribe_to_network", &NetworkHandler::subscribeToNetwork, this);
  INFO("Advertised network-subscription service");
}

//------ initSockets ------
void NetworkHandler::initSockets() {
  DBG("NetworkHandler::initSockets");

  INFO("Initalizing %s Port %u", ip_address.c_str(), port);

  try {
    endpoint_ = std::unique_ptr<udp::endpoint>{
        new udp::endpoint{asio::ip::address::from_string(ip_address), port}};
    socket_ = std::unique_ptr<udp::socket>{new udp::socket{io_, *endpoint_}};
    socket_->set_option(asio::socket_base::broadcast{true});
  } catch (const boost::system::system_error& e) {
    FATAL("Could not initialize network sockets: %s", e.what());
  }
}

//------ installNetworkHandler ------
void NetworkHandler::installNetworkHandler() {
  DBG("NetworkHandler::installNetworkHandler");

  socket_->async_receive(
      asio::buffer(io_buffer_),
      boost::bind(&NetworkHandler::recvFromNet, this, _1, _2));
}

//------ startNetworkService ------
void NetworkHandler::startNetworkService() {
  DBG("NetworkHandler::startNetworkService");

  // io_service::run will return if no more handler are to be dispatched.
  // We want to keep this service until we manually stop it, so at every time
  // at least one handler has to be installed. This can be accomplished by
  // reinstalling the handler while running.
  io_worker_ = std::thread{[this]() { io_.run(); }};
}

//------ subscribeToNetwork ------
bool NetworkHandler::subscribeToNetwork(
    srv::SubscribeNetwork::Request& request,
    srv::SubscribeNetwork::Response& response) {
  DBG("NetworkHandler::subscribeToNetwork");

  car_subscriptions_.push_back(
      {request.filter,
       getNodeHandle().advertise<msg::NetworkMessage>(request.topic, 2)});

  INFO("Network translation topic %s created for messages of type %08X",
       request.topic.c_str(), request.filter);

  return true;
}

//------ recvFromBus ------
void NetworkHandler::recvFromBus(const msg::NetworkMessage& msg) {
  DBG1("NetworkHandler::recvFromBus (first call)");

  try {
    boost::array<asio::const_buffer, 2> buffer = {
        asio::buffer<uint32_t, 1>(std::array<uint32_t, 1>{msg.type}),
        asio::buffer(msg.data)};

    udp::endpoint to_endpoint{asio::ip::address_v4::broadcast(), port};

    socket_->send_to(buffer, to_endpoint);
  } catch (const boost::system::system_error& e) {
    ERROR("Could not send network message: %s", e.what());
    return;
  }
}

//------ recvFromNet ------
void NetworkHandler::recvFromNet(const boost::system::error_code& error,
                                 size_t bytes_transferred) {
  DBG1("NetworkHandler::recvFromNet (first call)");

  if (bytes_transferred <= sizeof(uint32_t)) {
    ERROR("Invalid message received");
    installNetworkHandler();
    return;
  }

  boost::shared_ptr<msg::NetworkMessage> net_msg =
      boost::make_shared<msg::NetworkMessage>(msg::NetworkMessage());

  net_msg->type = *reinterpret_cast<uint32_t*>(io_buffer_.data());
  net_msg->data =
      std::string(&io_buffer_[4], MSG_BUFFER_SIZE - sizeof(uint32_t));

  std::for_each(car_subscriptions_.begin(), car_subscriptions_.end(),
                [this, &net_msg](const NetworkSubscription& net_sub) {
                  if (net_sub.msg_filter & net_msg->type)
                    net_sub.pub.publish(net_msg);
                });

  installNetworkHandler();
}

PLUGINLIB_DECLARE_CLASS(platoon, wifi, platoon::wifi::NetworkHandler,
                        nodelet::Nodelet);
