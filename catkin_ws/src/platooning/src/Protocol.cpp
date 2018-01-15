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
    pub_platooningOut = nh_.advertise< platooning::platoonProtocolOut >("commands/platoonProtocolOut", 10);

    //publisher of decoded platooning messages
    pub_platooningAction = nh_.advertise< platooning::platoonProtocolOut >("commands/platooningAction", 10);

  };


/*****************************************************************************
** Handlers
*****************************************************************************/

  /*
   * handling an event and publishing something
   */
  void Protocol::platoonProtocolInHandler(platooning::platoonProtocolIn msg) {

    NODELET_DEBUG("json payload received");

    platooningAction decodedMsg = DecodeIncomingJson( msg.payload );

    switch ( decodedMsg.actionType ) {
      case LEADER_REQUEST:
        pub_platooningAction.publish(decodedMsg);
      default:
        break;
    }

  }


/*****************************************************************************
** Helper functions
*****************************************************************************/

  platooningAction Protocol::DecodeIncomingJson( std::string& json ) {

    pt::ptree root;
    try {
      pt::read_json( json, root );
    } catch ( const pt::json_parser_error& ex) {
      NODELET_ERROR("incoming json parse error");
    }

    platooningAction action;

    action.actionType = root.get<short>("MessageType");

    switch ( action.actionType ) {
      case LEADER_REQUEST :
        action.vehicleId = root.get<unsigned int>("vehicle_id");
        action.platoonId = root.get<unsigned int>("platoon_id");
        break;
      case FOLLOWER_REQUEST:
        break;
      case ACCEPT_RESPONSE:
        break;
      case REJECT_RESPONSE:
        break;
      case HEARTBEAT:
        break;
      case BROADCAST:
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