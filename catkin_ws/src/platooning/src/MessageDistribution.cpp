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
#include "../include/MessageDistribution.hpp"

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
    sub_platooningIn = nh_.subscribe("in/platoonProtocol", 1,
                                             &MessageDistribution::hndl_platooningIn, this);

    sub_ui = nh_.subscribe("userinterface", 1,
                                     &MessageDistribution::hndl_ui, this);

    sub_lv_broadcast = nh_.subscribe("out/lv_broadcast", 100,
                           &MessageDistribution::hndl_lv_broadcast, this);
    sub_lv_accept = nh_.subscribe("out/lv_accept", 100,
                                     &MessageDistribution::hndl_lv_accept, this);
    sub_lv_reject = nh_.subscribe("out/lv_reject", 100,
                                     &MessageDistribution::hndl_lv_reject, this);
    sub_fv_heartbeat = nh_.subscribe("out/fv_heartbeat", 100,
                                     &MessageDistribution::hndl_fv_heartbeat, this);
    sub_fv_leave = nh_.subscribe("out/fv_leave", 100,
                                     &MessageDistribution::hndl_fv_leave, this);
    sub_fv_request = nh_.subscribe("out/fv_request", 100,
                                     &MessageDistribution::hndl_fv_request, this);

    //publisher of messages to send out
    pub_platooningOut = nh_.advertise< platooning::platoonProtocol >("out/platoonProtocol", 100);

    //publisher of decoded platooning messages
    pub_lv_broadcast = nh_.advertise< platooning::lv_broadcast >("in/lv_broadcast", 1); //queue size 1 to overwrite stale data
    pub_lv_accept = nh_.advertise< platooning::lv_accept >("in/lv_accept", 100);
    pub_lv_reject = nh_.advertise< platooning::lv_reject >("in/lv_reject", 100);
    pub_fv_heartbeat = nh_.advertise< platooning::fv_heartbeat >("in/fv_heartbeat", 100);
    pub_fv_leave = nh_.advertise< platooning::fv_leave >("in/fv_leave", 100);
    pub_fv_request = nh_.advertise< platooning::fv_request >("in/fv_request", 100);

    NODELET_INFO("PROTOCOL init done");

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
      switch ( inmsg.message_type ) {
        case LV_BROADCAST:
        {
          boost::shared_ptr<lv_broadcast> outmsg = boost::shared_ptr<lv_broadcast>( new lv_broadcast);
          DecodeIncomingJson( inmsg.payload, *outmsg );
          pub_lv_broadcast.publish(outmsg);
        }
          break;
        case LV_REJECT:
        {
          boost::shared_ptr<lv_reject> outmsg = boost::shared_ptr<lv_reject>( new lv_reject);
          DecodeIncomingJson( inmsg.payload, *outmsg );
          pub_lv_reject.publish(outmsg);
        }
          break;
        case LV_ACCEPT:
        {
          boost::shared_ptr<lv_accept> outmsg = boost::shared_ptr<lv_accept>( new lv_accept);
          DecodeIncomingJson( inmsg.payload, *outmsg );
          pub_lv_accept.publish(outmsg);
        }
          break;
        case FV_LEAVE:
        {
          boost::shared_ptr<fv_leave> outmsg = boost::shared_ptr<fv_leave>( new fv_leave);
          DecodeIncomingJson( inmsg.payload, *outmsg );
          pub_fv_leave.publish(outmsg);
        }
          break;
        case FV_REQUEST:
        {
          boost::shared_ptr<fv_request> outmsg = boost::shared_ptr<fv_request>( new fv_request);
          DecodeIncomingJson( inmsg.payload, *outmsg );
          pub_fv_request.publish(outmsg);
        }
          break;
        case FV_HEARTBEAT:
        {
          boost::shared_ptr<fv_heartbeat> outmsg = boost::shared_ptr<fv_heartbeat>( new fv_heartbeat);
          DecodeIncomingJson( inmsg.payload, *outmsg );
          pub_fv_heartbeat.publish(outmsg);
        }
          break;
        default:
          NODELET_ERROR("[protocol] unknown message type");
          break;
      }

    } catch(pt::json_parser_error& ex) {
      std::stringstream ss;

      ss << "[PROTOCOL] incoming json parse error, json malformed\n"
         << ex.what() << " " << ex.line() << " " << ex.message();

      NODELET_ERROR(ss.str().c_str());
    } catch( std::exception& ex ) {

      std::stringstream ss;

      ss << "[PROTOCOL] error in/platoonProtocolHandler\n"
          << ex.what();

      NODELET_ERROR( ss.str().c_str() );
    }
  }


/*****************************************************************************
** Helper Methods
*****************************************************************************/

  void MessageDistribution::DecodeIncomingJson( std::string& json, lv_broadcast& message ) {

    std::stringstream ss(json);

    pt::ptree root;
    pt::read_json(ss, root);

    message.src_vehicle = root.get<uint32_t>("src_vehicle");
    message.platoon_id = root.get<uint32_t>("platoon_id");
    message.ipd = root.get<float>("ipd");
    message.pd = root.get<float>("pd");

  }

  void MessageDistribution::DecodeIncomingJson( std::string& json, lv_accept& message ) {
    std::stringstream ss(json);

    pt::ptree root;
    pt::read_json( ss, root );

    message.src_vehicle = root.get<uint32_t >("src_vehicle");
    message.platoon_id = root.get<uint32_t >("platoon_id");
    message.dst_vehicle = root.get<uint32_t >("dst_vehicle");

  }

  void MessageDistribution::DecodeIncomingJson( std::string& json, lv_reject& message ) {
    std::stringstream ss(json);

    pt::ptree root;
    pt::read_json( ss, root );

    message.src_vehicle = root.get<uint32_t >("src_vehicle");
    message.platoon_id = root.get<uint32_t >("platoon_id");
    message.dst_vehicle = root.get<uint32_t >("dst_vehicle");

  }

  void MessageDistribution::DecodeIncomingJson( std::string& json, fv_request& message ) {
    std::stringstream ss(json);

    pt::ptree root;
    pt::read_json( ss, root );

    message.src_vehicle = root.get<uint32_t >("src_vehicle");
    //msgfields.ipd = root.get<float>("ipd");
    //msgfields.ipd = root.get<float>("pd");

  }

  void MessageDistribution::DecodeIncomingJson( std::string& json, fv_heartbeat& message ) {
    std::stringstream ss(json);

    pt::ptree root;
    pt::read_json( ss, root );

    message.src_vehicle = root.get<uint32_t >("src_vehicle");
    message.platoon_id = root.get<uint32_t >("platoon_id");

  }

  void MessageDistribution::DecodeIncomingJson( std::string& json, fv_leave& message ) {
    std::stringstream ss(json);

    pt::ptree root;
    pt::read_json( ss, root );

    message.src_vehicle = root.get<uint32_t >("src_vehicle");
    message.platoon_id = root.get<uint32_t >("platoon_id");

  }
} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::MessageDistribution, nodelet::Nodelet);
// %EndTag(FULLTEXT)%