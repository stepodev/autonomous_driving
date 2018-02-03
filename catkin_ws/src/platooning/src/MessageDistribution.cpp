//
// Created by stepo on 12/16/17.
//


/**
 * @file /platooning/src/platooning.cpp
 *
 * @brief Nodelet implementation of RemoteContol
 *
 * @author stepo
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include "MessageDistribution.hpp"

namespace pt = boost::property_tree;

namespace platooning {


/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

  MessageDistribution::MessageDistribution() = default;


/*****************************************************************************
** Destructors
*****************************************************************************/

  MessageDistribution::~MessageDistribution() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/

  /**
  * Set-up necessary publishers/subscribers
  * @return true, if successful
  */
  void MessageDistribution::onInit() {

    //subscribers of protocolmsg to decode and publish
    sub_platooningIn = nh_.subscribe(topics::IN_PLATOONING_MSG, 1,
                                     &MessageDistribution::hndl_platooningIn, this);

    sub_ui = nh_.subscribe(topics::USERINTERFACE, 1,
                           &MessageDistribution::hndl_ui, this);

    sub_lv_broadcast = nh_.subscribe(topics::OUT_LV_BROADCAST, 100,
                                     &MessageDistribution::hndl_lv_broadcast, this);
    sub_lv_accept = nh_.subscribe(topics::OUT_LV_ACCEPT, 100,
                                  &MessageDistribution::hndl_lv_accept, this);
    sub_lv_reject = nh_.subscribe(topics::OUT_LV_REJECT, 100,
                                  &MessageDistribution::hndl_lv_reject, this);
    sub_fv_heartbeat = nh_.subscribe(topics::OUT_FV_HEARTBEAT, 100,
                                     &MessageDistribution::hndl_fv_heartbeat, this);
    sub_fv_leave = nh_.subscribe(topics::OUT_FV_LEAVE, 100,
                                 &MessageDistribution::hndl_fv_leave, this);
    sub_fv_request = nh_.subscribe(topics::OUT_FV_REQUEST, 100,
                                   &MessageDistribution::hndl_fv_request, this);

    //publisher of messages to send out
    pub_platooningOut = nh_.advertise<platooning::platoonProtocol>(topics::OUT_PLATOONING_MSG, 100);

    //publisher of decoded platooning messages
    pub_lv_broadcast = nh_.advertise<platooning::lv_broadcast>(topics::IN_LV_BROADCAST,
                                                               1); //queue size 1 to overwrite stale data
    pub_lv_accept = nh_.advertise<platooning::lv_accept>(topics::IN_LV_ACCEPT, 100);
    pub_lv_reject = nh_.advertise<platooning::lv_reject>(topics::IN_LV_REJECT, 100);
    pub_fv_heartbeat = nh_.advertise<platooning::fv_heartbeat>(topics::IN_FV_HEARTBEAT, 100);
    pub_fv_leave = nh_.advertise<platooning::fv_leave>(topics::IN_FV_LEAVE, 100);
    pub_fv_request = nh_.advertise<platooning::fv_request>(topics::IN_FV_REQUEST, 100);

    NODELET_INFO("MessageDistribution init done");

  };


/*****************************************************************************
** Handlers
*****************************************************************************/

  /**
  * @brief decodes json payload of incoming message and publishes data on the appropriate topic
  * @return true, if successful
  */
  void MessageDistribution::hndl_platooningIn(platooning::platoonProtocol inmsg) {

    NODELET_DEBUG("json payload received");

    try {
      switch (inmsg.message_type) {
        case LV_BROADCAST: {
          boost::shared_ptr<lv_broadcast> outmsg = boost::shared_ptr<lv_broadcast>(new lv_broadcast);
          decode_json(inmsg.payload, *outmsg);
          pub_lv_broadcast.publish(outmsg);
        }
          break;
        case LV_REJECT: {
          boost::shared_ptr<lv_reject> outmsg = boost::shared_ptr<lv_reject>(new lv_reject);
          decode_json(inmsg.payload, *outmsg);
          pub_lv_reject.publish(outmsg);
        }
          break;
        case LV_ACCEPT: {
          boost::shared_ptr<lv_accept> outmsg = boost::shared_ptr<lv_accept>(new lv_accept);
          decode_json(inmsg.payload, *outmsg);
          pub_lv_accept.publish(outmsg);
        }
          break;
        case FV_LEAVE: {
          boost::shared_ptr<fv_leave> outmsg = boost::shared_ptr<fv_leave>(new fv_leave);
          decode_json(inmsg.payload, *outmsg);
          pub_fv_leave.publish(outmsg);
        }
          break;
        case FV_REQUEST: {
          boost::shared_ptr<fv_request> outmsg = boost::shared_ptr<fv_request>(new fv_request);
          decode_json(inmsg.payload, *outmsg);
          pub_fv_request.publish(outmsg);
        }
          break;
        case FV_HEARTBEAT: {
          boost::shared_ptr<fv_heartbeat> outmsg = boost::shared_ptr<fv_heartbeat>(new fv_heartbeat);
          decode_json(inmsg.payload, *outmsg);
          pub_fv_heartbeat.publish(outmsg);
        }
        case REMOTE_CONTROLINPUT: {
          boost::shared_ptr<remotecontrolInput> outmsg = boost::shared_ptr<remotecontrolInput>(new remotecontrolInput);
          decode_json(inmsg.payload, *outmsg);
          pub_fv_heartbeat.publish(outmsg);
        }
        case REMOTE_CONTROLTOGGLE: {
          boost::shared_ptr<remotecontrolToggle> outmsg = boost::shared_ptr<remotecontrolToggle>(new remotecontrolToggle);
          decode_json(inmsg.payload, *outmsg);
          pub_fv_heartbeat.publish(outmsg);
        }
        case REMOTE_PLATOONINGTOGGLE: {
          boost::shared_ptr<platooningToggle> outmsg = boost::shared_ptr<platooningToggle>(new platooningToggle);
          decode_json(inmsg.payload, *outmsg);
          pub_fv_heartbeat.publish(outmsg);
        }
          break;
        default:
          NODELET_ERROR("[MessageDistribution] unknown message type");
          break;
      }

    } catch (pt::json_parser_error &ex) {
      std::stringstream ss;

      ss << "[MessageDistribution] incoming json parse error, json malformed\n"
         << ex.what() << " " << ex.line() << " " << ex.message();

      NODELET_ERROR(ss.str().c_str());
    } catch (std::exception &ex) {
      NODELET_ERROR((std::string("[" + name_ + "] error in/platoonProtocolHandler\n") + ex.what()).c_str());
    }
  }


/*****************************************************************************
** Helper Methods
*****************************************************************************/

  template<typename T>
  std::vector<T> json_as_vector(boost::property_tree::ptree const &pt, boost::property_tree::ptree::key_type const &key) {
    std::vector<T> r;
    for (auto &item : pt.get_child(key))
      r.push_back(item.second.get_value<T>());
    return r;
  }

  template<typename T>
  boost::property_tree::ptree vector_as_json(std::vector<T> const &v) {
    boost::property_tree::ptree root;

    for( auto &i : v) {
      boost::property_tree::ptree leaf;
      leaf.put("",i);
      root.push_back(std::make_pair("",leaf));
    }

    return root;
  }

  void MessageDistribution::decode_json(const std::string &json, lv_broadcast &message) {
    std::stringstream ss(json);

    pt::ptree root;
    pt::read_json(ss, root);

    message.src_vehicle = root.get<uint32_t>("src_vehicle");
    message.platoon_id = root.get<uint32_t>("platoon_id");
    message.ipd = root.get<float>("ipd");
    message.pd = root.get<float>("pd");
  }

  void MessageDistribution::decode_json(const std::string &json, lv_accept &message) {
    std::stringstream ss(json);

    pt::ptree root;
    pt::read_json(ss, root);

    message.src_vehicle = root.get<uint32_t>("src_vehicle");
    message.platoon_id = root.get<uint32_t>("platoon_id");
    message.dst_vehicle = root.get<uint32_t>("dst_vehicle");

  }

  void MessageDistribution::decode_json(const std::string &json, lv_reject &message) {
    std::stringstream ss(json);

    pt::ptree root;
    pt::read_json(ss, root);

    message.src_vehicle = root.get<uint32_t>("src_vehicle");
    message.platoon_id = root.get<uint32_t>("platoon_id");
    message.dst_vehicle = root.get<uint32_t>("dst_vehicle");

  }

  void MessageDistribution::decode_json(const std::string &json, fv_request &message) {
    std::stringstream ss(json);

    pt::ptree root;
    pt::read_json(ss, root);

    message.src_vehicle = root.get<uint32_t>("src_vehicle");
    //msgfields.ipd = root.get<float>("ipd");
    //msgfields.ipd = root.get<float>("pd");

  }

  void MessageDistribution::decode_json(const std::string &json, fv_heartbeat &message) {
    std::stringstream ss(json);

    pt::ptree root;
    pt::read_json(ss, root);

    message.src_vehicle = root.get<uint32_t>("src_vehicle");
    message.platoon_id = root.get<uint32_t>("platoon_id");

  }

  void MessageDistribution::decode_json(const std::string &json, fv_leave &message) {
    std::stringstream ss(json);

    pt::ptree root;
    pt::read_json(ss, root);

    message.src_vehicle = root.get<uint32_t>("src_vehicle");
    message.platoon_id = root.get<uint32_t>("platoon_id");

  }

  void MessageDistribution::decode_json(const std::string &json, remotecontrolInput &message) {
    std::stringstream ss(json);

    pt::ptree root;
    pt::read_json(ss, root);

    message.emergency_stop = root.get<bool>("emergency_stop");
    message.remote_angle = root.get<bool>("remote_angle");
    message.remote_speed = root.get<bool>("remote_speed");
  }

  void MessageDistribution::decode_json(const std::string &json, remotecontrolToggle &message) {
    std::stringstream ss(json);

    pt::ptree root;
    pt::read_json(ss, root);

    message.enable_remotecontrol = root.get<bool>("enable_remotecontrol");

  }

  void MessageDistribution::decode_json(const std::string &json, platooningToggle &message) {

    std::stringstream ss(json);

    pt::ptree root;
    pt::read_json(ss, root);

    message.enable_platooning = root.get<bool>("enable_platooning");

  }

  void MessageDistribution::decode_json(const std::string &json, userInterface &message) {

    std::stringstream ss(json);

    pt::ptree root;
    pt::read_json(ss, root);

    message.enable_remotecontrol = root.get<bool>("enable_remotecontrol");
    message.leading_vehicle = root.get<bool>("leading_vehicle");
    message.following_vehicle = root.get<bool>("following_vehicle");
    message.platooning_state = root.get<std::string>("platooning_state");
    message.src_vehicle = root.get<uint32_t>("src_vehicle");
    message.platoon_members = json_as_vector<uint32_t>(root, "platoon_members");
  }

  std::string MessageDistribution::encode_message(const lv_broadcast &message) {

    std::string json;
    pt::ptree root;

    root.put( "src_vehicle", message.src_vehicle);
    root.add_child( "followers", vector_as_json<uint32_t>(message.followers));
    root.put( "pd", message.pd);
    root.put( "ipd", message.ipd);
    root.put( "platoon_id", message.platoon_id);

    std::stringstream ss;
    boost::property_tree::write_json(ss,root, false);

    return ss.str();
  }

  std::string MessageDistribution::encode_message(const lv_accept &message) {
    std::string json;
    pt::ptree root;

    root.put( "platoon_id", message.platoon_id);
    root.put( "src_vehicle", message.src_vehicle);
    root.put( "dst_vehicle", message.dst_vehicle);

    std::stringstream ss;
    boost::property_tree::write_json(ss,root, false);

    return ss.str();
  }

  std::string MessageDistribution::encode_message(const lv_reject &message) {
    std::string json;
    pt::ptree root;

    root.put( "platoon_id", message.platoon_id);
    root.put( "src_vehicle", message.src_vehicle);
    root.put( "dst_vehicle", message.dst_vehicle);

    std::stringstream ss;
    boost::property_tree::write_json(ss,root, false);

    return ss.str();
  }

  std::string MessageDistribution::encode_message(const fv_heartbeat &message) {
    std::string json;
    pt::ptree root;

    root.put( "platoon_id", message.platoon_id);
    root.put( "src_vehicle", message.src_vehicle);

    std::stringstream ss;
    boost::property_tree::write_json(ss,root, false);

    return ss.str();
  }

  std::string MessageDistribution::encode_message(const fv_leave &message) {
    std::string json;
    pt::ptree root;

    root.put( "platoon_id", message.platoon_id);
    root.put( "src_vehicle", message.src_vehicle);

    std::stringstream ss;
    boost::property_tree::write_json(ss,root, false);

    return ss.str();
  }

  std::string MessageDistribution::encode_message(const fv_request &message) {
    std::string json;
    pt::ptree root;

    root.put( "src_vehicle", message.src_vehicle);

    std::stringstream ss;
    boost::property_tree::write_json(ss,root, false);

    return ss.str();
  }

  std::string MessageDistribution::encode_message(const remotecontrolInput &message) {
    std::string json;
    pt::ptree root;

    root.put( "remote_angle", message.remote_angle);
    root.put( "remote_speed", message.remote_speed);
    root.put( "emergency_stop", message.emergency_stop);

    std::stringstream ss;
    boost::property_tree::write_json(ss,root, false);

    return ss.str();
  }

  std::string MessageDistribution::encode_message(const remotecontrolToggle &message) {
    std::string json;
    pt::ptree root;

    root.put( "enable_remotecontrol", message.enable_remotecontrol);

    std::stringstream ss;
    boost::property_tree::write_json(ss,root, false);

    return ss.str();
  }


  std::string MessageDistribution::encode_message(const platooningToggle &message) {
    std::string json;
    pt::ptree root;

    root.put( "enable_platooning", message.enable_platooning);

    std::stringstream ss;
    boost::property_tree::write_json(ss,root, false);

    return ss.str();
  }

  std::string MessageDistribution::encode_message(const userInterface &message) {
    std::string json;
    pt::ptree root;

    root.put( "src_vehicle", message.src_vehicle);
    root.put( "enable_remotecontrol", message.enable_remotecontrol);
    root.add_child( "platoon_members", vector_as_json<uint32_t >(message.platoon_members));
    root.put( "platooning_state", message.platooning_state);
    root.put( "following_vehicle", message.following_vehicle);
    root.put( "leading_vehicle", message.leading_vehicle);
    root.put( "speed", message.speed);
    root.put( "actual_distance", message.actual_distance);
    root.put( "platoon_size", message.platoon_size);
    root.put( "platoon_speed", message.platoon_speed);
    root.put( "inner_platoon_distance", message.inner_platoon_distance);
    root.put( "potential_following_vehicle", message.potential_following_vehicle);

    std::stringstream ss;
    boost::property_tree::write_json(ss,root, false);

    return ss.str();
  }

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::MessageDistribution, nodelet::Nodelet);
// %EndTag(FULLTEXT)%