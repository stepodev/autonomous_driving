/**
 * @file include/platooning/RadioInterface.hpp
 * @author stepo
 * @date 22.03.2018
 * @brief Radiointerface header file
 */

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PLATOONING_RADIOINTERFACE_HPP
#define PLATOONING_RADIOINTERFACE_HPP

/*****************************************************************************
** Includes
*****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <string>
#include <boost/asio.hpp>
#include <functional>
#include <utility>
#include <iomanip>

#include "Topics.hpp"
#include "MessageTypes.hpp"
#include "UdpServer.hpp"

using namespace boost::asio;

namespace platooning {

/**
 * @class RadioInterface
 *
 * @brief Manages a communication class to translate ros graph messages to sendable packets and back.
 *
 * Spins up two communication class objects, currently UdpServer instances. One to receive and broadcast udp packages
 * from and to the platoon network, the other to receive controller messages.
 *
 * On receipt of UDP package, the header byte is read, the message converted into a @refitem platoonProtocol message
 * and sent on @ref platooning::IN_PLATOONING_MSG.
 * On receipt of a message on topic @refitem platooning::OUT_PLATOON_MSG, the message gets converted into a packet
 * an byte of header describing the message type followed by a char array of json.
 *
 * @bugs No known
 *
 */

class RadioInterface : public nodelet::Nodelet {
  public:

	/**
	 * @brief Called by nodeletmanager
	 */
	virtual void onInit();

	RadioInterface();

	~RadioInterface();

  private:
	ros::NodeHandle nh_;
	std::string name_= "radiointerface";
	std::unique_ptr<UdpServer> platooning_server_ptr_;
	std::unique_ptr<UdpServer> controller_server_ptr_;

	ros::Subscriber sub_platoonProtocolOut_;

	ros::Publisher pub_platoonProtocolIn_; /**< hands to udp_server to publish received messages */
	ros::Publisher pub_communicationMessageIn_; /**< publishes received communication messages */

	/**
	 * @brief called by subscriber to @ref OUT_PLATOON_MSG and sends it out per communication class
	 * @param msg platoonProtocol message that needs to be sent out
	 */
	void hndl_platoonProtocolOut(platooning::platoonProtocol msg);

	/**
	 * @brief callback for the communication class to call with received Message. Publishes that on @ref OUT_PLATOON_MSG
	 * @param message_tuple a tuple of payload and messagetype
	 */
	void hndl_radio_receive(boost::shared_ptr<std::pair<std::string, uint32_t>> message_tuple);

};

} // namespace platooning

#endif //PLATOONING_RADIOINTERFACE_HPP
