//
// Created by stepo on 12/16/17.
//


/**
 * @file /platooning/src/RadioInterface.cpp
 *
 * @brief RadioInterface class
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

RadioInterface::RadioInterface() = default;

/*****************************************************************************
** Destructors
*****************************************************************************/

RadioInterface::~RadioInterface() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/
void RadioInterface::onInit() {

	sub_platoonProtocolOut_ = nh_.subscribe(topics::OUT_PLATOONING_MSG, 100,
	                                        &RadioInterface::hndl_platoonProtocolOut, this);

	pub_platoonProtocolIn_ = nh_.advertise<platoonProtocol>(topics::IN_PLATOONING_MSG, 100);



	try {
		/**< create function object to callback method */
		boost::function<void(boost::shared_ptr<std::pair<std::string, uint32_t>>)> cbfun(boost::bind(boost::mem_fn(
			&RadioInterface::hndl_radio_receive), this, _1));

		/**< instantiate communication class listening to udp port 10000 and broadcasting to udp port 10000 */
		platooning_server_ptr_ = std::unique_ptr<UdpServer>(new UdpServer(cbfun,
		                                                       udp::endpoint(udp::v4(), 10000),
		                                                       udp::endpoint(ip::address_v4::broadcast(), 10000)));

		/**< instantiate communication class listening to udp port 13500 and broadcasting to udp port 13500 */
		controller_server_ptr_ = std::unique_ptr<UdpServer>(new UdpServer(cbfun,
		                                                                  udp::endpoint(udp::v4(), 13500),
		                                                                  udp::endpoint(ip::address_v4::broadcast(), 13500)));
	} catch (std::exception &e) {
		NODELET_FATAL("[%s] udpserver init failed\n %s ", name_.c_str(),e.what());
	}

	NODELET_INFO("[%s] init done", name_.c_str());
};


/*****************************************************************************
** Handlers
*****************************************************************************/

void RadioInterface::hndl_platoonProtocolOut(platooning::platoonProtocol msg) {

	/**
	 * switch on messagetype to determine on which communication channel to send out
	 */

	switch ( msg.message_type ) {
		case FV_REQUEST:
		case LV_ACCEPT:
		case LV_REJECT:
		case FV_LEAVE:
		case LV_BROADCAST:
		case FV_HEARTBEAT:
			platooning_server_ptr_->start_send(msg.payload, msg.message_type);
			break;
		case REMOTE_USERINTERFACE:
			controller_server_ptr_->start_send(msg.payload, msg.message_type);
			break;
		default:
			NODELET_ERROR("[%s] trying to send unknown messagetype %#010x paypload %s", name_.c_str(), msg.message_type, msg.payload.c_str());
			break;
	}


}

void RadioInterface::hndl_radio_receive(boost::shared_ptr<std::pair<std::string, uint32_t>> message_pair) {

	/**
	 * Filter by message type to determine what to do with the payload. If relevant to us, forward on IN_PLATOON_MSG
	 */

	auto outmsg = boost::shared_ptr<platooning::platoonProtocol>(new platooning::platoonProtocol);

	switch (message_pair->second) {
		case FV_REQUEST:
		case LV_ACCEPT:
		case LV_REJECT:
		case FV_LEAVE:
		case REMOTE_PLATOONINGTOGGLE:
			NODELET_INFO("[%s] received upd command %#010x : %s", name_.c_str(), message_pair->second, message_pair->first.c_str());
			outmsg->payload = message_pair->first;
			outmsg->message_type = message_pair->second;
			pub_platoonProtocolIn_.publish(outmsg);
			break;
		case LV_BROADCAST:
		case FV_HEARTBEAT:
			outmsg->payload = message_pair->first;
			outmsg->message_type = message_pair->second;
			pub_platoonProtocolIn_.publish(outmsg);
			break;
		case REMOTE_USERINTERFACE:
			//not for us
			break;
		default:NODELET_ERROR("[%s] messagetype not recognized. Type: %#010x : %s", name_.c_str(), message_pair->second, message_pair->first.c_str());
			break;

	}
}


/*****************************************************************************
** HELPER CLASS
*****************************************************************************/


} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::RadioInterface, nodelet::Nodelet);
// %EndTag(FULLTEXT)%