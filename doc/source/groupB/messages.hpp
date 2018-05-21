//******************************************************************
// file:       platoon/messages.hpp
// purpose:    platoon network messages
// license:    ?
// maintainer: Bruno Lemke - lemkebru@informatik.hu-berlin.de
// created:    2017/02/03
//******************************************************************

#ifndef ROS1_PLATOON_MESSAGES_HPP
#define ROS1_PLATOON_MESSAGES_HPP

//******************************************************************
// MESSAGES
//******************************************************************

namespace platoon {

namespace PlatoonMessage {

constexpr uint32_t LV_BROADCAST = 0x01;
constexpr uint32_t FV_HEARTBEAT = 0x02;
constexpr uint32_t FV_REQUEST = 0x08;
constexpr uint32_t LV_ACCEPT = 0x10;
constexpr uint32_t LV_REJECT = 0x20;
constexpr uint32_t FV_LEAVE = 0x40;

}  // namespace PlatoonMessage

}  // namespace platoon

#endif  // ROS1_PLATOON_MESSAGES_HPP
