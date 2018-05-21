//******************************************************************
// file:       platoon/protocol.hpp
// purpose:    ros-node for the car platoon communication protocol
// license:    ?
// maintainer: Bruno Lemke - lemkebru@informatik.hu-berlin.de
// created:    2017/11/23
//******************************************************************

#ifndef ROS1_PLATOON_PROTOCOL_HPP
#define ROS1_PLATOON_PROTOCOL_HPP

//******************************************************************
// PROTOCOL
//******************************************************************

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <tuple>

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <messages.hpp>

#include <platoon_idl/CreatePlatoon.h>
#include <platoon_idl/LeavePlatoon.h>
#include <platoon_idl/NetworkMessage.h>
#include <platoon_idl/PlatoonConfiguration.h>

// TODO
// - dynamic create platoon
// - send request and expect answer
// - timeout for receiving broadcast

namespace platoon {
namespace protocol {
//******************************************************************
// Platoon IDL
//******************************************************************

namespace srv {
using platoon_idl::CreatePlatoon;
using platoon_idl::LeavePlatoon;
}  // namespace srv

namespace msg {
using platoon_idl::NetworkMessage;
using platoon_idl::PlatoonConfiguration;
}  // namespace msg

//******************************************************************
// PlatoonMessage
//******************************************************************

constexpr int LEADING_VEHICLE_FILTER = PlatoonMessage::FV_REQUEST |
                                       PlatoonMessage::FV_HEARTBEAT |
                                       PlatoonMessage::FV_LEAVE;
constexpr int FOLLOWER_VEHICLE_FILTER = PlatoonMessage::LV_ACCEPT |
                                        PlatoonMessage::LV_REJECT |
                                        PlatoonMessage::LV_BROADCAST;

//******************************************************************
// PlatooningProtocol
//******************************************************************

class PlatooningProtocol : public nodelet::Nodelet {
 public:
  PlatooningProtocol() = default;
  ~PlatooningProtocol();

  //------ Services ------
  bool createPlatoon(srv::CreatePlatoon::Request& request,
                     srv::CreatePlatoon::Response& response);
  bool leavePlatoon(srv::LeavePlatoon::Request& request,
                    srv::LeavePlatoon::Response& response);
  bool queryState(srv::LeavePlatoon::Request& request,
                  srv::LeavePlatoon::Response& response);

  //------ Subscriber Callbacks ------
  void busRecvCallback(const msg::PlatoonConfiguration& msg);
  void netRecvCallback(const msg::NetworkMessage& msg);

  //------ Platoon Configuration ------
  float ps{0};
  float ipd{0};
  uint32_t position{1};
  uint32_t platoon_id{0};
  uint32_t vehicle_id{position};
  std::map<uint32_t,
           std::chrono::time_point<std::chrono::high_resolution_clock>>
      platoon_list;

  std::mutex platoon_lock;
  std::atomic<bool> platoon_created{false};
  std::atomic<bool> platoon_running{false};
  std::atomic<bool> platoon_changed{false};
  std::atomic<bool> platoon_requesting{false};
  std::atomic<bool> platoon_leaving{false};
  std::queue<boost::shared_ptr<msg::NetworkMessage>> scheduled_messages;
  std::tuple<uint32_t,
             std::chrono::time_point<std::chrono::high_resolution_clock>>
      expected_answer;

 private:
  //------ Initialization ------
  virtual void onInit() override final;

  //------ Helper Functions ------
  bool startPlatoon();
  void stopPlatoon();
  bool requestNetworkSubscription(uint32_t msg_filter,
                                  const std::string& topic);

  //----- Network Msg Handler ------
  void lvRecvMsgNoPlatoon(const msg::NetworkMessage& msg);
  void lvRecvMsgInPlatoon(const msg::NetworkMessage& msg);
  void fvRecvMsgNoPlatoon(const msg::NetworkMessage& msg);
  void fvRecvMsgInPlatoon(const msg::NetworkMessage& msg);

  std::array<ros::ServiceServer, 2> services_;
  ros::ServiceServer state_query_service_;

  ros::Publisher bus_pub_;
  ros::Subscriber bus_sub_;
  ros::Subscriber net_sub_;

  std::thread worker_;

  friend class ProtocolWorker;

#ifdef PLATOON_TEST_SUITE
  friend class ProtocolTestSuite
#endif
};

//******************************************************************
// ProtocolWorker
//******************************************************************

class ProtocolWorker {
 public:
  ProtocolWorker(PlatooningProtocol* protocol);

  void operator()();

 private:
  void lvWorker();
  void fvWorker();

  void fvRequest();
  void fvLeave();

  PlatooningProtocol* protocol_;
  const std::string& protocol_name_;

  ros::Publisher net_pub_;
};

}  // namespace protocol
}  // namespace platoon

#endif  // ROS1_PLATOON_PROTOCOL_HPP
