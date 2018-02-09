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

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp> //json parsing and generating

#include <platooning/userInterface.h>
#include <platooning/remotecontrolInput.h>
#include <platooning/remotecontrolToggle.h>

#include "platooning/platoonProtocol.h"
#include "platooning/lv_broadcast.h"
#include "platooning/lv_accept.h"
#include "platooning/lv_reject.h"
#include "platooning/fv_request.h"
#include "platooning/fv_heartbeat.h"
#include "platooning/fv_leave.h"
#include "platooning/platooningToggle.h"
#include "platooning/platooningState.h"

#include "platooning/templateMsg.h"

namespace platooning {

/**
 * @brief takes json and decodes it
 * @param msg protocol data
 */
  void decode_json(const std::string &json, lv_broadcast &message);

  void decode_json(const std::string &json, lv_accept &message);

  void decode_json(const std::string &json, lv_reject &message);

  void decode_json(const std::string &json, fv_heartbeat &message);

  void decode_json(const std::string &json, fv_leave &message);

  void decode_json(const std::string &json, fv_request &message);

  void decode_json(const std::string &json, remotecontrolInput &message);

  void decode_json(const std::string &json, remotecontrolToggle &message);

  void decode_json(const std::string &json, platooningToggle &message);

  void decode_json(const std::string &json, userInterface &message);


  std::string encode_message(const lv_broadcast &message);

  std::string encode_message(const lv_accept &message);

  std::string encode_message(const lv_reject &message);

  std::string encode_message(const fv_heartbeat &message);

  std::string encode_message(const fv_leave &message);

  std::string encode_message(const fv_request &message);

  std::string encode_message(const remotecontrolInput &message);

  std::string encode_message(const remotecontrolToggle &message);

  std::string encode_message(const platooningToggle &message);

  std::string encode_message(const userInterface &message);

  template<typename T>
  std::vector<T>
  json_as_vector(boost::property_tree::ptree const &pt, boost::property_tree::ptree::key_type const &key);

  template<typename T>
  boost::property_tree::ptree vector_as_json(std::vector<T> const &v);



}

#endif //PLATOONING_MESSAGETYPES_HPP
