//******************************************************************
// file:       platoon/wifi.hpp
// purpose:    ros-node for network access
// license:    ?
// maintainer: Bruno Lemke - lemkebru@informatik.hu-berlin.de
// created:    2017/11/23
//******************************************************************

#ifndef ROS1_PLATOON_WIFI_HPP
#define ROS1_PLATOON_WIFI_HPP

//******************************************************************
// WIFI
//******************************************************************

#include <array>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <platoon_idl/NetworkMessage.h>
#include <platoon_idl/PlatoonConfiguration.h>
#include <platoon_idl/SubscribeNetwork.h>

namespace platoon {
namespace wifi {
constexpr int MSG_BUFFER_SIZE = 1024;

//******************************************************************
// Platoon IDL
//******************************************************************

namespace msg {
using platoon_idl::NetworkMessage;
using platoon_idl::PlatoonConfiguration;
}  // namespace msg

namespace srv {
using platoon_idl::SubscribeNetwork;
}  // namespace srv

//******************************************************************
// NetworkSubscription
//******************************************************************

struct NetworkSubscription {
  uint32_t msg_filter;
  ros::Publisher pub;
};

//******************************************************************
// NetworkHandler
//******************************************************************

class NetworkHandler : public nodelet::Nodelet {
 public:
  NetworkHandler() = default;
  ~NetworkHandler();

  //------ Services ------
  bool subscribeToNetwork(srv::SubscribeNetwork::Request& request,
                          srv::SubscribeNetwork::Response& response);

  //------ Configuration ------
  std::string ip_address;
  uint16_t port;

 private:
  //------ Initialization ------
  virtual void onInit() override final;
  void initSockets();
  void installNetworkHandler();
  void startNetworkService();

  //------ Msg Handler Functions ------
  void recvFromBus(const msg::NetworkMessage& msg);
  void recvFromNet(const boost::system::error_code& error,
                   size_t bytes_transferred);

  //------ Services / Subscriptions ------
  ros::ServiceServer network_subscription_service_;
  ros::Subscriber car_subscriber_;
  std::vector<NetworkSubscription> car_subscriptions_;

  //------ Worker and Network Services ------
  std::thread io_worker_;
  boost::asio::io_service io_;
  std::unique_ptr<boost::asio::ip::udp::socket> socket_;
  std::unique_ptr<boost::asio::ip::udp::endpoint> endpoint_;

  std::array<char, MSG_BUFFER_SIZE> io_buffer_;
};

}  // namespace wifi
}  // namespace platoon

#endif  // ROS1_PLATOON_WIFI_HPP