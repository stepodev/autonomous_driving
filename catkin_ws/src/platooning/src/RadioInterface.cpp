//
// Created by stepo on 12/16/17.
//


/**
 * @file /platooning/src/wifi.cpp
 *
 * @brief Implements Listeners and Senders for traffic towards the network
 *
 * @author stepo
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include "platooning/RadioInterface.hpp"

namespace platooning {


/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

RadioInterface::RadioInterface() = default;

/*****************************************************************************
** Destructors
*****************************************************************************/

RadioInterface::~RadioInterface() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/

/**
* Set-up necessary publishers/subscribers
*/
void RadioInterface::onInit() {

	name_ = "radiointerface";

	//subscribers of protocol nodelet
	sub_platoonProtocolOut_ = nh_.subscribe(topics::OUT_PLATOONING_MSG, 100,
	                                        &RadioInterface::hndl_platoonProtocolOut, this);

	pub_platoonProtocolIn_ = nh_.advertise<platoonProtocol>(topics::IN_PLATOONING_MSG, 100);

	try {
		//bind to local 10000 port, broadcast to 10000 port
		boost::function<void(std::pair<std::string, uint32_t>)> cbfun(boost::bind(boost::mem_fn(
			&RadioInterface::hndl_radio_receive), this, _1));

		server_ptr_ = std::unique_ptr<UdpServer>(new UdpServer(cbfun,
		                                                       udp::endpoint(udp::v4(), 10000),
		                                                       udp::endpoint(ip::address_v4::broadcast(), 10000)));
	} catch (std::exception &e) {
		NODELET_FATAL("[%s] udpserver init failed\n %s ", name_.c_str(),e.what());
	}

	NODELET_INFO("[%s] init done", name_.c_str());
};


/*****************************************************************************
** Handlers
*****************************************************************************/

/**
 * @brief Sends messages into the network
 * @param platoonProtocol msg to be sent
 */
void RadioInterface::hndl_platoonProtocolOut(platooning::platoonProtocol msg) {

	switch ( msg.message_type ) {
		case FV_REQUEST:
			NODELET_INFO("[%s] Sending FV_REQUEST via upd", name_.c_str());
			break;
		default:
			break;
	}

	server_ptr_->start_send(msg.payload, msg.message_type);
}

/**
 * @brief handles received messages from the network
 * @param message_pair with message_type and payload
 */
void RadioInterface::hndl_radio_receive(std::pair<std::string, uint32_t> message_pair) {
	//std::cout << "handling radiointerface receive" << std::endl;

	boost::shared_ptr<platooning::platoonProtocol> outmsg
		= boost::shared_ptr<platooning::platoonProtocol>(new platooning::platoonProtocol);

	switch (message_pair.second) {
		case FV_REQUEST:
		case LV_ACCEPT:
		case LV_REJECT:
		case FV_LEAVE:
		case REMOTE_PLATOONINGTOGGLE:
			NODELET_INFO("[%s] received upd command", name_.c_str());
			outmsg->payload = message_pair.first;
			outmsg->message_type = message_pair.second;
			pub_platoonProtocolIn_.publish(outmsg);
			break;
		case LV_BROADCAST:
		case FV_HEARTBEAT:
			NODELET_INFO("[%s] received broadcast", name_.c_str());
			outmsg->payload = message_pair.first;
			outmsg->message_type = message_pair.second;
			pub_platoonProtocolIn_.publish(outmsg);
			break;
		case REMOTE_USERINTERFACE:
			//not for us
			break;
		default:NODELET_ERROR("[%s] messagetype not recognized. Type: %i", name_.c_str(), (uint8_t)message_pair.second);
			break;

	}
}


/*****************************************************************************
** HELPER CLASS
*****************************************************************************/


} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::RadioInterface, nodelet::Nodelet);
// %EndTag(FULLTEXT)%