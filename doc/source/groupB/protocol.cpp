//******************************************************************
// file:       platoon/protocol.cpp
// purpose:    ros-node for the car platoon communication protocol
// license:    ?
// maintainer: Bruno Lemke - lemkebru@informatik.hu-berlin.de
// created:    2017/11/23
//******************************************************************

//******************************************************************
// PROTOCOL
//******************************************************************

#include <json.hpp>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <config.hpp>
#include <protocol.hpp>

#include <platoon_idl/SubscribeNetwork.h>

using namespace platoon::protocol;

using json = nlohmann::json;
using hr_clock = std::chrono::high_resolution_clock;
using hr_time = std::chrono::time_point<hr_clock>;
using duration = std::chrono::duration<int64_t, std::milli>;

constexpr int REQUEST_TRIES = 10;
constexpr uint32_t LEADING_VEHICLE_POSITION = 0;
constexpr uint32_t INVALID_VEHICLE = 0xFFFFFFFF;
constexpr duration BROADCAST_TIMEOUT = std::chrono::milliseconds(500);
constexpr duration HEARTBEAT_TIMEOUT =
    std::chrono::seconds(1);  // TODO: specification?
constexpr duration BROADCAST_INTERVAL = std::chrono::milliseconds(50);
constexpr duration HEARTBEAT_INTERVAL = std::chrono::milliseconds(200);
constexpr duration REQUEST_TIMEOUT =
    std::chrono::seconds(1);  // TODO: specification

//******************************************************************
// PlatooningProtocol
//******************************************************************

//------ ~PlatooningProtocol ------
PlatooningProtocol::~PlatooningProtocol() {
  DBG("PlatooningProtocol::~PlatooningProtocol");

  bus_pub_.shutdown();
  bus_sub_.shutdown();
  for (auto& srv : services_) srv.shutdown();
  state_query_service_.shutdown();
}

//------ onInit ------
void PlatooningProtocol::onInit() {
  DBG("PlatooningProtocol::onInit");

  position = static_cast<uint32_t>(getPrivateNodeHandle().param<int>(
      "/vehicle/position", LEADING_VEHICLE_POSITION));
  vehicle_id = static_cast<uint32_t>(
      getPrivateNodeHandle().param<int>("/vehicle/vehicle_id", position));
  platoon_id = static_cast<uint32_t>(
      getPrivateNodeHandle().param<int>("/vehicle/platoon_id", 0));
  ipd = getPrivateNodeHandle().param<float>("/vehicle/ipd", 0);
  ps = getPrivateNodeHandle().param<float>("/vehicle/ps", 0);

  INFO("Parameters set: position=%u vehicle_id=%u platoon_id=%u ipd=%f ps=%f",
       position, vehicle_id, platoon_id, ipd, ps);

  bus_pub_ = getMTPrivateNodeHandle().advertise<msg::PlatoonConfiguration>(
      "car_bus", 1);
  bus_sub_ = getMTPrivateNodeHandle().subscribe(
      "car_bus", 2, &PlatooningProtocol::busRecvCallback, this);

  INFO("Bus connection established");

  services_ = {getMTPrivateNodeHandle().advertiseService(
                   "create_platoon", &PlatooningProtocol::createPlatoon, this),
               getMTPrivateNodeHandle().advertiseService(
                   "leave_platoon", &PlatooningProtocol::leavePlatoon, this)};

  state_query_service_ = getMTPrivateNodeHandle().advertiseService(
      "query_state", &PlatooningProtocol::queryState, this);

  INFO("Services advertised");
}

//------ createPlatoon ------
bool PlatooningProtocol::createPlatoon(srv::CreatePlatoon::Request& request,
                                       srv::CreatePlatoon::Response& response) {
  DBG("PlatooningProtocol::createPlatoon");

  if (platoon_created) {
    WARN("WARNING: tried to create already established platoon");
    return false;
  }

  if (!requestNetworkSubscription(position == LEADING_VEHICLE_POSITION
                                      ? LEADING_VEHICLE_FILTER
                                      : FOLLOWER_VEHICLE_FILTER,
                                  "protocol_recv")) {
    ERROR("ERROR: Network connection could not be established");
    return false;
  }

  net_sub_ = getMTPrivateNodeHandle().subscribe(
      "protocol_recv", 2, &PlatooningProtocol::netRecvCallback, this);

  platoon_created = true;

  if (position != LEADING_VEHICLE_POSITION) {
    platoon_requesting = true;
    if (!startPlatoon()) {
      ERROR("ERROR: Could not start platoon request");
      return false;
    }
  }

  response.position = position;

  INFO("Network connection established");
  return true;
}

//------ leavePlatoon ------
bool PlatooningProtocol::leavePlatoon(srv::LeavePlatoon::Request& request,
                                      srv::LeavePlatoon::Response& response) {
  DBG("PlatooningProtocol::leavePlatoon");

  if (!platoon_created) {
    WARN("WARNING: tried to leave not established platoon");
    return false;
  }

  if (platoon_running) stopPlatoon();

  net_sub_.shutdown();

  platoon_created = false;

  return true;
}

//------ queryState ------
bool PlatooningProtocol::queryState(srv::LeavePlatoon::Request& request,
                                    srv::LeavePlatoon::Response& response) {
  INFO("CURRENT PLATOON CONFIGURATION:");
  INFO("  ps:                %f", ps);
  INFO("  ipd:               %f", ipd);
  INFO("  position:          %u", position);
  INFO("  vehicle_id:        %u", vehicle_id);
  INFO("  platoon_id:        %u", platoon_id);
  INFO("  platoon_list:      %u vehicles", platoon_list.size());
  INFO("CURRENT PLATOON STATE:");
  INFO("platoon_created:     %s", platoon_created ? "true" : "false");
  INFO("platoon_running:     %s", platoon_running ? "true" : "false");
  INFO("platoon_changed:     %s", platoon_changed ? "true" : "false");
  INFO("platoon_requesting:  %s", platoon_requesting ? "true" : "false");
  INFO("platoon_leaving:     %s", platoon_leaving ? "true" : "false");
  return true;
}

//------ startPlatoon ------
bool PlatooningProtocol::startPlatoon() {
  DBG("PlatooningProtocol::startPlatoon");

  assert(platoon_created);
  assert(!platoon_running);

  if (worker_.joinable()) {
    ERROR("ERROR: Network worker thread was already running");
    return false;
  }
  worker_ = std::thread{ProtocolWorker{this}};

  platoon_running = true;
  return true;
}

//------ stopPlatoon ------
void PlatooningProtocol::stopPlatoon() {
  DBG("PlatooningProtocol::stopPlatoon");

  assert(platoon_created);
  assert(platoon_running);

  assert(worker_.joinable());

  if (position != LEADING_VEHICLE_POSITION)
    platoon_leaving = true;
  else
    platoon_running = false;

  worker_.join();
}

//------ requestNetworkSubscription ------
bool PlatooningProtocol::requestNetworkSubscription(uint32_t msg_filter,
                                                    const std::string& topic) {
  DBG("PlatooningProtocol::requestNetworkSubscription");

  platoon_idl::SubscribeNetwork sub_net_svc;
  sub_net_svc.request.filter = msg_filter;
  sub_net_svc.request.topic = "protocol_recv";
  return ros::service::call("subscribe_to_network", sub_net_svc);
}

//------ busRecvCallback ------
void PlatooningProtocol::busRecvCallback(const msg::PlatoonConfiguration& msg) {
  DBG1("PlatooningProtocol::busRecvCallback (first call)");

  ATOMIC(platoon_lock) {
    if (position != LEADING_VEHICLE_POSITION) {
      WARN("WARNING: tried to change platoon via bus from FV");
      return;
    }

    if (ps != msg.ps || ipd != msg.ipd) {
      ps = msg.ps;
      ipd = msg.ipd;
      platoon_changed = true;

      DBG("Platoon configuration has changed via bus message (%u, %u)", msg.ps,
          msg.ipd);
    }
  }
}

//------ netRecvCallback ------
void PlatooningProtocol::netRecvCallback(const msg::NetworkMessage& msg) {
  DBG1("PlatooningProtocol::netRecvCallback (first call)");

  ATOMIC(platoon_lock) {
    try {
      // TODO: use a state machine
      if (position == LEADING_VEHICLE_POSITION)
        platoon_running ? lvRecvMsgInPlatoon(msg) : lvRecvMsgNoPlatoon(msg);
      else
        platoon_running ? fvRecvMsgInPlatoon(msg) : fvRecvMsgNoPlatoon(msg);
    } catch (const json::exception&) {
      ERROR("Received invalid json message");
    }
  }
}

//------ lvRecvMsgInPlatoon ------
void PlatooningProtocol::lvRecvMsgInPlatoon(const msg::NetworkMessage& msg) {
  if (msg.type == PlatoonMessage::FV_HEARTBEAT) {
    uint32_t msg_src_vehicle = json{msg.data}.at("src_vehicle").get<uint32_t>();
    uint32_t msg_platoon_id = json{msg.data}.at("platoon_id").get<uint32_t>();
    if (msg_platoon_id != platoon_id) {
      WARN("WARNING: Received msg for different platoon %d", msg_platoon_id);
      return;
    }
    auto iter = platoon_list.find(msg_src_vehicle);
    if (iter == platoon_list.end()) {
      WARN("WARNING: Received msg from not registers vehicle %d",
           msg_src_vehicle);
      return;
    }
    iter->second = hr_clock::now();
  }

  else if (msg.type == PlatoonMessage::FV_REQUEST) {
    uint32_t msg_src_vehicle = json{msg.data}.at("src_vehicle").get<uint32_t>();
    // TODO: specification: ipd?
    // TODO: specification: ps?

    uint32_t answer = 0;
    auto iter = platoon_list.find(msg_src_vehicle);
    if (iter == platoon_list.end()) {
      platoon_list.insert({msg_src_vehicle, hr_clock::now()});
      answer = PlatoonMessage::LV_ACCEPT;
    } else {
      WARN(
          "WARNING: Received request from already registered vehicle "
          "%d",
          msg_src_vehicle);
      answer = PlatoonMessage::LV_REJECT;
    }

    boost::shared_ptr<msg::NetworkMessage> msg =
        boost::make_shared<msg::NetworkMessage>(msg::NetworkMessage());
    msg->type = answer;
    msg->data = json::array({{"src_vehicle", position},
                             {"platoon_id", platoon_id},
                             {"dst_vehicle", msg_src_vehicle}})
                    .dump();
    scheduled_messages.push(std::move(msg));
  }

  else if (msg.type == PlatoonMessage::FV_LEAVE) {
    uint32_t msg_src_vehicle = json{msg.data}.at("src_vehicle");
    uint32_t msg_platoon_id = json{msg.data}.at("platoon_id");
    if (msg_platoon_id != platoon_id) {
      WARN("WARNING: Received msg for different platoon %d", msg_platoon_id);
      return;
    }
    auto iter = platoon_list.find(msg_src_vehicle);
    if (iter == platoon_list.end()) {
      WARN("WARNING: Received leave msg from not registered vehicle %d",
           msg_src_vehicle);
      return;
    }

    platoon_list.erase(iter);
  }

  else
    ERROR("ERROR: Received invalid network msg: 0x%08X", msg.type);
}

//------ lvRecvMsgNoPlatoon ------
void PlatooningProtocol::lvRecvMsgNoPlatoon(const msg::NetworkMessage& msg) {
  if (msg.type == PlatoonMessage::FV_HEARTBEAT) {
    uint32_t msg_src_vehicle = json{msg.data}.at("src_vehicle").get<uint32_t>();

    WARN("WARNING: Received heartbeat without created platoon from %d",
         msg_src_vehicle);
  }

  else if (msg.type == PlatoonMessage::FV_REQUEST) {
    uint32_t msg_src_vehicle = json{msg.data}.at("src_vehicle").get<uint32_t>();

    assert(!platoon_running);
    assert(platoon_list.size() == 0);

    if (!startPlatoon()) {
      ERROR("ERROR: Could not start platoon");
      return;
    }

    platoon_list.insert({msg_src_vehicle, hr_clock::now()});

    boost::shared_ptr<msg::NetworkMessage> msg =
        boost::make_shared<msg::NetworkMessage>(msg::NetworkMessage());
    msg->type = PlatoonMessage::LV_ACCEPT;
    msg->data = json::array({{"src_vehicle", position},
                             {"platoon_id", platoon_id},
                             {"dst_vehicle", msg_src_vehicle}})
                    .dump();
    scheduled_messages.push(std::move(msg));
  }

  else if (msg.type == PlatoonMessage::FV_LEAVE) {
    uint32_t msg_src_vehicle = json{msg.data}.at("src_vehicle").get<uint32_t>();
    WARN("WARNING: Received leave msg without created platoon from %d",
         msg_src_vehicle);
  }

  else
    ERROR("ERROR: Received invalid network msg: 0x%08X", msg.type);
}

//------ fvRecvMsgInPlatoon ------
void PlatooningProtocol::fvRecvMsgInPlatoon(const msg::NetworkMessage& msg) {
  if (msg.type == PlatoonMessage::LV_ACCEPT) {
    uint32_t msg_src_vehicle = json{msg.data}.at("src_vehicle").get<uint32_t>();
    uint32_t msg_platoon_id = json{msg.data}.at("platoon_id").get<uint32_t>();
    uint32_t msg_dst_vehicle = json{msg.data}.at("dst_vehicle").get<uint32_t>();

    if (msg_dst_vehicle != vehicle_id) return;

    WARN("WARNING: Is already in platoon, but received accept msg from %d",
         msg_src_vehicle);
  }

  else if (msg.type == PlatoonMessage::LV_REJECT) {
    uint32_t msg_src_vehicle = json{msg.data}.at("src_vehicle").get<uint32_t>();
    uint32_t msg_platoon_id = json{msg.data}.at("platoon_id").get<uint32_t>();
    uint32_t msg_dst_vehicle = json{msg.data}.at("dst_vehicle").get<uint32_t>();

    if (msg_dst_vehicle != vehicle_id) return;

    WARN("WARNING: Is already in platoon, but received reject msg from %d",
         msg_src_vehicle);
  }

  else if (msg.type == PlatoonMessage::LV_BROADCAST) {
    uint32_t msg_src_vehicle = json{msg.data}.at("src_vehicle").get<uint32_t>();
    uint32_t msg_platoon_id = json{msg.data}.at("platoon_id").get<uint32_t>();
    uint32_t msg_ipd = json{msg.data}.at("ipd").get<uint32_t>();
    uint32_t msg_ps = json{msg.data}.at("ps").get<uint32_t>();
    std::vector<uint32_t> msg_followers;
    for (auto& j : json{msg.data}.at("followers"))
      msg_followers.push_back(j.get<uint32_t>());

    if (msg_platoon_id != platoon_id) {
      WARN("WARNING: Received msg for different platoon %d", msg_platoon_id);
      return;
    }

    bool registered_as_follower = false;
    for (uint32_t id : msg_followers) {
      if (id == vehicle_id) {
        registered_as_follower = true;
        break;
      }
    }
    if (!registered_as_follower) {
      WARN("WARNING: Lost connection to platoon");
      stopPlatoon();
    }
  }

  else
    ERROR("ERROR: Received invalid network msg: 0x%08X", msg.type);
}

//------ fvRecvMsgNoPlatoon ------
void PlatooningProtocol::fvRecvMsgNoPlatoon(const msg::NetworkMessage& msg) {
  if (msg.type == PlatoonMessage::LV_ACCEPT) {
    uint32_t msg_src_vehicle = json{msg.data}.at("src_vehicle").get<uint32_t>();
    uint32_t msg_platoon_id = json{msg.data}.at("platoon_id").get<uint32_t>();
    uint32_t msg_dst_vehicle = json{msg.data}.at("dst_vehicle").get<uint32_t>();

    if (msg_dst_vehicle != vehicle_id) return;

    auto& expected_answer_vehicle = std::get<0>(expected_answer);
    if (expected_answer_vehicle == INVALID_VEHICLE) {
      WARN("WARNING: Received not expected accept msg from %d",
           msg_src_vehicle);
      return;
    }

    INFO("REQUEST ACCEPTED - PLATOON ESTABLISHED");
    platoon_running = true;
    platoon_requesting = false;

    expected_answer_vehicle = INVALID_VEHICLE;
  }

  else if (msg.type == PlatoonMessage::LV_REJECT) {
    uint32_t msg_src_vehicle = json{msg.data}.at("src_vehicle").get<uint32_t>();
    uint32_t msg_platoon_id = json{msg.data}.at("platoon_id").get<uint32_t>();
    uint32_t msg_dst_vehicle = json{msg.data}.at("dst_vehicle").get<uint32_t>();

    if (msg_dst_vehicle != vehicle_id) return;

    auto& expected_answer_vehicle = std::get<0>(expected_answer);
    if (expected_answer_vehicle == INVALID_VEHICLE) {
      WARN("WARNING: Received not expected reject msg from %d",
           msg_src_vehicle);
      return;
    }

    INFO("REQUEST REJECTED");  // TODO: signal master
    platoon_requesting = false;

    expected_answer_vehicle = INVALID_VEHICLE;
  }

  else if (msg.type == PlatoonMessage::LV_BROADCAST) {
  }

  else
    ERROR("ERROR: Received invalid network msg: 0x%08X", msg.type);
}

//******************************************************************
// ProtocolWorker
//******************************************************************

//------ ProtocolWorker ------
ProtocolWorker::ProtocolWorker(PlatooningProtocol* protocol)
    : protocol_(protocol), protocol_name_(protocol->getName()) {
  DBG_OBJ(protocol_->getName(), "ProcotolWorker::ProcotolWorker");
  net_pub_ = protocol_->getMTPrivateNodeHandle().advertise<msg::NetworkMessage>(
      "protocol_send", 1);
}

//------ operator () ------
void ProtocolWorker::operator()() {
  protocol_->position == LEADING_VEHICLE_POSITION ? lvWorker() : fvWorker();
}

//------ lvWorker ------
void ProtocolWorker::lvWorker() {
  DBG_OBJ(protocol_name_, "ProtocolWorker::lvWorker");

  while (protocol_->platoon_running) {
    ATOMIC(protocol_->platoon_lock) {
      while (protocol_->scheduled_messages.size() != 0) {
        net_pub_.publish(protocol_->scheduled_messages.front());
        protocol_->scheduled_messages.pop();

        INFO_OBJ(protocol_name_, "Responded to request");
      }

      hr_time now = hr_clock::now();
      json follower_list = json::array();

      // Equivalent to std::experimental::erase_if
      auto& follower_map = protocol_->platoon_list;
      for (auto iter = follower_map.begin(); iter != follower_map.end();) {
        if ((now - std::get<1>(*iter)) >= HEARTBEAT_TIMEOUT) {
          WARN_OBJ(protocol_name_, "Lost following vehicle");
          iter = follower_map.erase(iter);
        } else {
          follower_list.push_back(std::get<0>(*iter));
          ++iter;
        }
      }

      boost::shared_ptr<msg::NetworkMessage> msg =
          boost::make_shared<msg::NetworkMessage>(msg::NetworkMessage());
      msg->type = PlatoonMessage::LV_BROADCAST;
      msg->data = json::array({{"src_vehicle", protocol_->position},
                               {"platoon_id", protocol_->platoon_id},
                               {"ps", protocol_->ps},
                               {"ipd", protocol_->ipd},
                               {"followers", follower_list}})
                      .dump();
      net_pub_.publish(msg);
    }

    INFO_OBJ1(protocol_name_, "published first broadcast to network");
    std::this_thread::sleep_for(BROADCAST_INTERVAL);
  }
}

//------ fvWorker ------
void ProtocolWorker::fvWorker() {
  DBG_OBJ(protocol_name_, "ProtocolWorker::fvWorker");

  fvRequest();

  while (protocol_->platoon_running) {
    if (protocol_->platoon_leaving) {
      fvLeave();
      break;
    }

    ATOMIC(protocol_->platoon_lock) {
      if (std::get<0>(protocol_->expected_answer) != INVALID_VEHICLE &&
          (hr_clock::now() - std::get<1>(protocol_->expected_answer)) >=
              REQUEST_TIMEOUT) {
        WARN_OBJ(protocol_name_, "Request timed out");
        std::get<0>(protocol_->expected_answer) = INVALID_VEHICLE;
      }

      boost::shared_ptr<msg::NetworkMessage> msg =
          boost::make_shared<msg::NetworkMessage>(msg::NetworkMessage());
      msg->type = PlatoonMessage::FV_HEARTBEAT;
      msg->data = json::array({{"src_vehicle", protocol_->position},
                               {"platoon_id", protocol_->platoon_id}})
                      .dump();
      net_pub_.publish(msg);
    }

    INFO_OBJ1(protocol_name_, "published first heartbeat to network");
    std::this_thread::sleep_for(HEARTBEAT_INTERVAL);
  }
}

//------ fvRequest ------
void ProtocolWorker::fvRequest() {
  DBG_OBJ(protocol_name_, "ProtocolWorker::fvRequest");

  int tries = 0;

  while (protocol_->platoon_requesting) {
    ATOMIC(protocol_->platoon_lock) {
      boost::shared_ptr<msg::NetworkMessage> msg =
          boost::make_shared<msg::NetworkMessage>(msg::NetworkMessage());
      msg->type = PlatoonMessage::FV_REQUEST;
      msg->data = json::array({{"src_vehicle", protocol_->vehicle_id}}).dump();
      // ps?
      // ipd?
      net_pub_.publish(msg);
    }

    if (++tries >= REQUEST_TRIES) {
      WARN_OBJ(protocol_name_, "Request was not answered");
      protocol_->platoon_running = false;
      break;
    }

    std::this_thread::sleep_for(HEARTBEAT_INTERVAL);
  }

  protocol_->platoon_requesting = false;
}

//------ fvLeave ------
void ProtocolWorker::fvLeave() {
  DBG_OBJ(protocol_name_, "ProtocolWorker:fvLeave");

  ATOMIC(protocol_->platoon_lock) {
    boost::shared_ptr<msg::NetworkMessage> msg =
        boost::make_shared<msg::NetworkMessage>(msg::NetworkMessage());
    msg->type = PlatoonMessage::FV_LEAVE;
    msg->data = json::array({{"src_vehicle", protocol_->vehicle_id},
                             {"platoon_id", protocol_->platoon_id}})
                    .dump();
    net_pub_.publish(msg);

    protocol_->platoon_running = false;
    protocol_->platoon_leaving = false;
  }
}

PLUGINLIB_DECLARE_CLASS(platoon, protocol,
                        platoon::protocol::PlatooningProtocol,
                        nodelet::Nodelet);
