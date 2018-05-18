/**
 * @file include/platooning/MessageTypes.hpp
 * @author stepo
 * @date 22.03.2018
 *
 */


/**
 * @class MessageTypes
 *
 * @brief Includes generated messages, defines their header bytes, provides decode and encode functions.
 *
 * Each ros generated message is included here. Their header bytes for encoding to udp packets are defined and functions
 * decoding and encoding are defined here also.
 *
 * Each encode_json takes the message to be encoded and returns a json string.
 * Each decode_json takes a json and an instance of the message that json should be decoded to.
 *
 * @bugs No known
 *
 */
#ifndef PLATOONING_MESSAGETYPES_HPP
#define PLATOONING_MESSAGETYPES_HPP

#define LV_BROADCAST 0x00000001
#define FV_HEARTBEAT 0x00000002
#define FV_REQUEST 0x00000008
#define LV_ACCEPT 0x00000010
#define LV_REJECT 0x00000020
#define FV_LEAVE 0x00000040
#define REMOTE_CUSTOM 0x01000000
#define REMOTE_CMD 0x02000000
#define REMOTE_LOG 0x04000000

#define REMOTE_PLATOONINGTOGGLE 0x00000100
#define REMOTE_CONTROLTOGGLE 0x00000200
#define REMOTE_CONTROLINPUT 0x00000300
#define REMOTE_USERINTERFACE 0x00000400

#define STMSIM_UPDATE 0x00001000
#define GAZ_UPDATE 0x00002000

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp> //json parsing and generating

#include <platooning/json.hpp>

#include "platooning/remotecontrolToggle.h"
#include "platooning/userInterface.h"
#include "platooning/platoonProtocol.h"
#include "platooning/lv_broadcast.h"
#include "platooning/lv_accept.h"
#include "platooning/lv_reject.h"
#include "platooning/fv_request.h"
#include "platooning/fv_heartbeat.h"
#include "platooning/fv_leave.h"
#include "platooning/platooningToggle.h"
#include "platooning/platooningState.h"
#include "platooning/remotecontrolInput.h"
#include "platooning/acceleration.h"
#include "platooning/steeringAngle.h"
#include "platooning/gazupdate.h"
#include "platooning/stmupdate.h"
#include "platooning/targetSpeed.h"
#include "platooning/targetAngle.h"
#include "platooning/targetDistance.h"
#include "platooning/speed.h"
#include "platooning/distance.h"
#include "platooning/vehicleControl.h"
#include "platooning/templateMsg.h"

namespace platooning {

class MessageTypes {

  public:
	template< typename T >
	static T decode_json(const std::string &json);

	template <typename T>
	static std::string encode_message(const T &message);

};

}

#endif //PLATOONING_MESSAGETYPES_HPP
