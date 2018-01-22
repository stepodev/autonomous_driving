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
#include "Protocol.hpp"

namespace pt = boost::property_tree;

namespace platooning {


/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

  Protocol::Protocol() = default;



/*****************************************************************************
** Destructors
*****************************************************************************/

  Protocol::~Protocol() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/

  /**
  * Set-up necessary publishers/subscribers
  * @return true, if successful
  */
  void Protocol::onInit() {

    //subscribers of protocol nodelet
    sub_platooningIn = nh_.subscribe("platoonProtocolIn", 10,
                                             &Protocol::platoonProtocolInHandler, this);

    //publisher of messages to send out
    pub_platooningOut = nh_.advertise< platooning::platoonProtocolOut >("platoonProtocolOut", 10);

    //publisher of decoded platooning messages
    pub_platooningAction = nh_.advertise< platooning::platooningAction >("platooningAction", 10);

    NODELET_INFO("PROTOCOL init done");

  };


/*****************************************************************************
** Handlers
*****************************************************************************/

  /*
   * handling an event and publishing something
   */
  void Protocol::platoonProtocolInHandler(platooning::platoonProtocolIn msg) {

    NODELET_DEBUG("json payload received");

    try {
      platooningAction decodedMsg = DecodeIncomingJson( msg.payload );

      switch ( decodedMsg.actionType ) {
        case LV_REQUEST:
          pub_platooningAction.publish(decodedMsg);
        default:
          break;
      }

    } catch(pt::json_parser_error& ex) {
      std::stringstream ss;

      ss << "[PROTOCOL] incoming json parse error, json malformed\n"
         << ex.what() << " " << ex.line() << " " << ex.message();

      NODELET_ERROR(ss.str().c_str());
    } catch( std::exception& ex ) {

      std::stringstream ss;

      ss << "[PROTOCOL] error platoonProtocolInHandler\n"
          << ex.what();

      NODELET_ERROR( ss.str().c_str() );
    }
  }


/*****************************************************************************
** Helper functions
*****************************************************************************/

  platooningAction Protocol::DecodeIncomingJson( std::string& json ) {

    std::cout << "i recvd a thing" << std::endl;

    std::stringstream ss(json);

    pt::ptree root;
    pt::read_json( ss, root );

    platooningAction action;

    action.actionType = root.get<short>("MessageType");

    switch ( action.actionType ) {
      case LV_REQUEST :
        action.vehicleId = root.get<unsigned int>("vehicle_id");
        action.platoonId = root.get<unsigned int>("platoon_id");
        break;
      case FV_REQUEST:
        break;
      case ACCEPT_RESPONSE:
        break;
      case REJECT_RESPONSE:
        break;
      case FV_HEARTBEAT:
        break;
      case LV_BROADCAST:
        action.vehicleId = root.get<unsigned int>("vehicle_id");
        action.platoonId = root.get<unsigned int>("platoon_id");
        action.innerPlatoonDistance = root.get<unsigned int>("ipd");
        action.platoonSpeed = root.get<unsigned int>("ps");
        break;
      case LEAVE_PLATOON:
        break;
      default:
        NODELET_ERROR("unrecognized message type");
    }

    return action;

  }

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Protocol, nodelet::Nodelet);
// %EndTag(FULLTEXT)%