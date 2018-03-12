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
#include "platooning/MessageDistribution.hpp"

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
	sub_platooningIn = nh_.subscribe(topics::IN_PLATOONING_MSG, 1,
	                                 &MessageDistribution::hndl_platooningIn, this);

	sub_ui = nh_.subscribe(topics::USERINTERFACE, 1,
	                       &MessageDistribution::hndl_ui, this);

	sub_lv_broadcast = nh_.subscribe(topics::OUT_LV_BROADCAST, 100,
	                                 &MessageDistribution::hndl_lv_broadcast, this);
	sub_lv_accept = nh_.subscribe(topics::OUT_LV_ACCEPT, 100,
	                              &MessageDistribution::hndl_lv_accept, this);
	sub_lv_reject = nh_.subscribe(topics::OUT_LV_REJECT, 100,
	                              &MessageDistribution::hndl_lv_reject, this);
	sub_fv_heartbeat = nh_.subscribe(topics::OUT_FV_HEARTBEAT, 100,
	                                 &MessageDistribution::hndl_fv_heartbeat, this);
	sub_fv_leave = nh_.subscribe(topics::OUT_FV_LEAVE, 100,
	                             &MessageDistribution::hndl_fv_leave, this);
	sub_fv_request = nh_.subscribe(topics::OUT_FV_REQUEST, 100,
	                               &MessageDistribution::hndl_fv_request, this);

	//publisher of messages to send out
	pub_platooningOut = nh_.advertise<platooning::platoonProtocol>(topics::OUT_PLATOONING_MSG, 100);

	//publisher of decoded platooning messages
	pub_lv_broadcast = nh_.advertise<platooning::lv_broadcast>(topics::IN_LV_BROADCAST,
	                                                           1); //queue size 1 to overwrite stale data
	pub_lv_accept = nh_.advertise<platooning::lv_accept>(topics::IN_LV_ACCEPT, 100);
	pub_lv_reject = nh_.advertise<platooning::lv_reject>(topics::IN_LV_REJECT, 100);
	pub_fv_heartbeat = nh_.advertise<platooning::fv_heartbeat>(topics::IN_FV_HEARTBEAT, 100);
	pub_fv_leave = nh_.advertise<platooning::fv_leave>(topics::IN_FV_LEAVE, 100);
	pub_fv_request = nh_.advertise<platooning::fv_request>(topics::IN_FV_REQUEST, 100);

	pub_remotecontrol_input = nh_.advertise<remotecontrolInput>(topics::REMOTECONTROL_INPUT, 100);
	pub_remotecontrol_toggle = nh_.advertise<remotecontrolToggle>(topics::TOGGLE_REMOTECONTROL, 100);
	pub_platooning_toggle = nh_.advertise<platooningToggle>(topics::TOGGLE_PLATOONING, 100);

	NODELET_INFO("[MessageDistribution] init done");

};


/*****************************************************************************
** Handlers
*****************************************************************************/

/**
* @brief decodes json payload of incoming message and publishes data on the appropriate topic
* @return true, if successful
*/
void MessageDistribution::hndl_platooningIn(const platooning::platoonProtocol &inmsg) {
	try {
		switch (inmsg.message_type) {
			case LV_BROADCAST: {
				boost::shared_ptr<lv_broadcast> outmsg = boost::shared_ptr<lv_broadcast>(new lv_broadcast);
				MessageTypes::decode_json(inmsg.payload, *outmsg);
				pub_lv_broadcast.publish(outmsg);
			}
				return;
			case LV_REJECT: {
				NODELET_INFO("[%s] received upd message LV_REJECT", name_.c_str());
				boost::shared_ptr<lv_reject> outmsg = boost::shared_ptr<lv_reject>(new lv_reject);
				MessageTypes::decode_json(inmsg.payload, *outmsg);
				pub_lv_reject.publish(outmsg);
			}
				return;
			case LV_ACCEPT: {
				NODELET_INFO("[%s] received upd message LV_ACCEPT", name_.c_str());
				boost::shared_ptr<lv_accept> outmsg = boost::shared_ptr<lv_accept>(new lv_accept);
				MessageTypes::decode_json(inmsg.payload, *outmsg);
				pub_lv_accept.publish(outmsg);
			}
				return;
			case FV_LEAVE: {
				NODELET_INFO("[%s] received upd message FV_LEAVE", name_.c_str());
				boost::shared_ptr<fv_leave> outmsg = boost::shared_ptr<fv_leave>(new fv_leave);
				MessageTypes::decode_json(inmsg.payload, *outmsg);
				pub_fv_leave.publish(outmsg);
			}
				return;
			case FV_REQUEST: {
				NODELET_INFO("[%s] received upd message FV_REQUEST", name_.c_str());
				boost::shared_ptr<fv_request> outmsg = boost::shared_ptr<fv_request>(new fv_request);
				MessageTypes::decode_json(inmsg.payload, *outmsg);
				pub_fv_request.publish(outmsg);
			}
				return;
			case FV_HEARTBEAT: {
				boost::shared_ptr<fv_heartbeat> outmsg = boost::shared_ptr<fv_heartbeat>(new fv_heartbeat);
				MessageTypes::decode_json(inmsg.payload, *outmsg);
				pub_fv_heartbeat.publish(outmsg);
			}
			case REMOTE_CONTROLINPUT: {
				NODELET_INFO("[%s] received upd message REMOTE_CONTROLINPUT", name_.c_str());
				boost::shared_ptr<remotecontrolInput>
					outmsg = boost::shared_ptr<remotecontrolInput>(new remotecontrolInput);
				MessageTypes::decode_json(inmsg.payload, *outmsg);
				pub_remotecontrol_input.publish(outmsg);
				return;
			}
			case REMOTE_CONTROLTOGGLE: {
				NODELET_INFO("[%s] received upd message REMOTE_CONTROLTOGGLE", name_.c_str());
				boost::shared_ptr<remotecontrolToggle> outmsg = boost::shared_ptr<remotecontrolToggle>(
					new remotecontrolToggle);
				MessageTypes::decode_json(inmsg.payload, *outmsg);
				pub_remotecontrol_toggle.publish(outmsg);
				return;
			}
			case REMOTE_PLATOONINGTOGGLE: {
				NODELET_INFO("[%s] received upd message REMOTE_PLATOONINGTOGGLE", name_.c_str());
				boost::shared_ptr<platooningToggle> outmsg = boost::shared_ptr<platooningToggle>(new platooningToggle);
				MessageTypes::decode_json(inmsg.payload, *outmsg);
				pub_platooning_toggle.publish(outmsg);
				return;
			}
			default:NODELET_ERROR("[MessageDistribution] unknown message type");
				break;
		}

	} catch (pt::json_parser_error &ex) {
		NODELET_ERROR("[%s]incoming json parse error, json malformed\n%s %lu %s",
		              name_.c_str(), ex.what(), ex.line(), ex.message().c_str());
	} catch (std::exception &ex) {
		NODELET_ERROR("[%s] error in/platoonProtocolHandler\n %s", name_.c_str(), ex.what());
	}

	NODELET_ERROR("[%s] we received udp. we shouldnt be here", name_.c_str());
}

void MessageDistribution::hndl_lv_broadcast(const lv_broadcast &msg) {
	auto p = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	p->message_type = LV_BROADCAST;
	p->payload = MessageTypes::encode_message(msg);

	pub_platooningOut.publish(p);
}

void MessageDistribution::hndl_lv_accept(const platooning::lv_accept &msg) {
	NODELET_INFO("[%s] Sending upd message LV_ACCEPT", name_.c_str());
	auto p = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	p->message_type = LV_ACCEPT;
	p->payload = MessageTypes::encode_message(msg);

	pub_platooningOut.publish(p);
}

void MessageDistribution::hndl_lv_reject(const platooning::lv_reject &msg) {
	NODELET_INFO("[%s] Sending upd message LV_REJECT", name_.c_str());
	auto p = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	p->message_type = LV_REJECT;
	p->payload = MessageTypes::encode_message(msg);

	pub_platooningOut.publish(p);
}

void MessageDistribution::hndl_fv_heartbeat(const platooning::fv_heartbeat &msg) {
	auto p = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	p->message_type = FV_HEARTBEAT;
	p->payload = MessageTypes::encode_message(msg);

	pub_platooningOut.publish(p);
}

void MessageDistribution::hndl_fv_leave(const platooning::fv_leave &msg) {
	NODELET_INFO("[%s] Sending upd message FV_LEAVE", name_.c_str());
	auto p = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	p->message_type = FV_LEAVE;
	p->payload = MessageTypes::encode_message(msg);

	pub_platooningOut.publish(p);
}

void MessageDistribution::hndl_fv_request(const platooning::fv_request &msg) {
	NODELET_INFO("[%s] Sending upd message FV_REQUEST", name_.c_str());
	auto p = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	p->message_type = FV_REQUEST;
	p->payload = MessageTypes::encode_message(msg);

	pub_platooningOut.publish(p);
}

void MessageDistribution::hndl_ui(const platooning::userInterface &msg) {
	auto p = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	p->message_type = REMOTE_USERINTERFACE;
	p->payload = MessageTypes::encode_message(msg);

	pub_platooningOut.publish(p);
}

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::MessageDistribution, nodelet::Nodelet);
// %EndTag(FULLTEXT)%