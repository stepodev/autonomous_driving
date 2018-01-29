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

  /**
  * @brief decodes json payload of incoming message and publishes data on the appropriate topic
  * @return true, if successful
  */
  void Protocol::platoonProtocolInHandler(platooning::platoonProtocolIn msg) {

    NODELET_DEBUG("json payload received");

    try {
      MessageFields msgfields = DecodeIncomingJson( msg.payload, msg.message_type );

      switch ( msg.message_type ) {
        case LV_BROADCAST:

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

  template <class T>
  void Protocol::DecodeIncomingJson<T>( std::string& json, T message ) {

    std::cout << "i recvd a thing" << std::endl;

    std::stringstream ss(json);

    pt::ptree root;
    pt::read_json( ss, root );

    switch ( message ) {
      case FV_REQUEST :
        msgfields.src_vehicle = root.get<uint32_t >("src_vehicle");
        //msgfields.ipd = root.get<float>("ipd");
        //msgfields.ipd = root.get<float>("pd");
        break;
      case LV_ACCEPT:
        msgfields.src_vehicle = root.get<uint32_t >("src_vehicle");
        msgfields.platoon_id = root.get<uint32_t >("platoon_id");
        msgfields.dst_vehicle = root.get<uint32_t >("dst_vehicle");
        break;
      case LV_REJECT:
        msgfields.src_vehicle = root.get<uint32_t >("src_vehicle");
        msgfields.platoon_id = root.get<uint32_t >("platoon_id");
        msgfields.dst_vehicle = root.get<uint32_t >("dst_vehicle");
        break;
      case FV_HEARTBEAT:
        msgfields.src_vehicle = root.get<uint32_t >("src_vehicle");
        msgfields.platoon_id = root.get<uint32_t >("platoon_id");
        break;
      case LV_BROADCAST:
        msgfields.src_vehicle = root.get<uint32_t >("src_vehicle");
        msgfields.platoon_id = root.get<uint32_t >("platoon_id");
        msgfields.ipd = root.get<float>("ipd");
        msgfields.ipd = root.get<float>("pd");
        break;
      case FV_LEAVE:
        msgfields.src_vehicle = root.get<uint32_t >("src_vehicle");
        msgfields.platoon_id = root.get<uint32_t >("platoon_id");
        break;
      default:
        NODELET_ERROR("unrecognized message type");
    }

    return msgfields;

  }

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Protocol, nodelet::Nodelet);
// %EndTag(FULLTEXT)%