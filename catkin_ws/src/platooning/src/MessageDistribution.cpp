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
#include "platooning/MessageDistribution.hpp"

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

    pub_remotecontrol_input = nh_.advertise<remotecontrolInput>(topics::REMOTECONTROL_INPUT, 100);
    pub_remotecontrol_toggle = nh_.advertise<remotecontrolToggle>(topics::TOGGLE_REMOTECONTROL, 100);
    pub_platooning_toggle = nh_.advertise<platooningToggle>(topics::TOGGLE_PLATOONING, 100);

    NODELET_INFO("MessageDistribution init done");

  };


/*****************************************************************************
** Handlers
*****************************************************************************/

  /**
  * @brief decodes json payload of incoming message and publishes data on the appropriate topic
  * @return true, if successful
  */
  void MessageDistribution::hndl_platooningIn(const platooning::platoonProtocol &inmsg) {

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
          pub_remotecontrol_input.publish(outmsg);
        }
        case REMOTE_CONTROLTOGGLE: {
          boost::shared_ptr<remotecontrolToggle> outmsg = boost::shared_ptr<remotecontrolToggle>(
              new remotecontrolToggle);
          decode_json(inmsg.payload, *outmsg);
          pub_remotecontrol_toggle.publish(outmsg);
        }
        case REMOTE_PLATOONINGTOGGLE: {
          boost::shared_ptr<platooningToggle> outmsg = boost::shared_ptr<platooningToggle>(new platooningToggle);
          decode_json(inmsg.payload, *outmsg);
          pub_platooning_toggle.publish(outmsg);
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

  void MessageDistribution::hndl_lv_broadcast(const lv_broadcast &msg) {
    auto p = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
    p->message_type = LV_BROADCAST;
    p->payload = encode_message(msg);

    pub_platooningOut.publish(p);
  }

  void MessageDistribution::hndl_lv_accept(const platooning::lv_accept &msg) {
    auto p = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
    p->message_type = LV_ACCEPT;
    p->payload = encode_message(msg);

    pub_platooningOut.publish(p);
  }

  void MessageDistribution::hndl_lv_reject(const platooning::lv_reject &msg) {
    auto p = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
    p->message_type = LV_REJECT;
    p->payload = encode_message(msg);

    pub_platooningOut.publish(p);
  }

  void MessageDistribution::hndl_fv_heartbeat(const platooning::fv_heartbeat &msg) {
    auto p = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
    p->message_type = FV_HEARTBEAT;
    p->payload = encode_message(msg);

    pub_platooningOut.publish(p);
  }

  void MessageDistribution::hndl_fv_leave(const platooning::fv_leave &msg) {
    auto p = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
    p->message_type = FV_LEAVE;
    p->payload = encode_message(msg);

    pub_platooningOut.publish(p);
  }

  void MessageDistribution::hndl_fv_request(const platooning::fv_request &msg) {
    auto p = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
    p->message_type = FV_REQUEST;
    p->payload = encode_message(msg);

    pub_platooningOut.publish(p);
  }

  void MessageDistribution::hndl_ui(const platooning::userInterface &msg) {
    auto p = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
    p->message_type = REMOTE_USERINTERFACE;
    p->payload = encode_message(msg);

    pub_platooningOut.publish(p);
  }

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::MessageDistribution, nodelet::Nodelet);
// %EndTag(FULLTEXT)%