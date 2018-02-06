#include "PlatooningNode.hpp"

namespace platooning
{
	PlatooningNode::PlatooningNode() {}
	PlatooningNode::~PlatooningNode() {}

	void PlatooningNode::onInit() {

		// subscribers nh_.subscribe("Topic", Rate, &Callback, this)
		sub_fv_leave = nh_.subscribe(topics::IN_FV_LEAVE, 100, &PlatooningNode::hndl_fv_leave, this);
		sub_lv_accept = nh_.subscribe(topics::IN_LV_ACCEPT, 100, &PlatooningNode::hndl_lv_accept, this);
		sub_lv_reject = nh_.subscribe(topics::IN_LV_REJECT, 100, &PlatooningNode::hndl_lv_reject, this);
		sub_fv_request = nh_.subscribe(topics::IN_FV_REQUEST, 100, &PlatooningNode::hndl_fv_request, this);
		sub_lv_broadcast = nh_.subscribe(topics::IN_LV_BROADCAST, 100, &PlatooningNode::hndl_lv_broadcast, this);
		sub_fv_heartbeat = nh_.subscribe(topics::IN_FV_HEARTBEAT, 100, &PlatooningNode::hndl_fv_heartbeat, this);
		sub_platooning_toggle = nh_.subscribe(topics::TOGGLE_PLATOONING, 100, &PlatooningNode::hndl_platooning_toggle, this);

		//publishers nh_.advertise<MsgType>("Topic")
		pub_fv_leave = nh_.advertise<platooning::fv_leave>(topics::OUT_FV_LEAVE, 100);
		pub_lv_accept = nh_.advertise<platooning::lv_accept>(topics::OUT_LV_ACCEPT, 100);
		pub_lv_reject = nh_.advertise<platooning::lv_reject>(topics::OUT_LV_REJECT, 100);
		pub_fv_request = nh_.advertise<platooning::fv_request>(topics::OUT_FV_REQUEST, 100);
		pub_lv_broadcast = nh_.advertise<platooning::lv_broadcast>(topics::OUT_LV_BROADCAST, 1);
		pub_fv_heartbeat = nh_.advertise<platooning::fv_heartbeat>(topics::OUT_FV_HEARTBEAT, 100);
		pub_platooning_state = nh_.advertise<platooning::platooningState>(topics::PLATOONINGSTATE, 100);
	};

	void PlatooningNode::hndl_fv_leave(const platooning::fv_leave& msg) {
		NODELET_DEBUG("handling fv_leave");
	}

	void PlatooningNode::hndl_lv_accept(const platooning::lv_accept& msg) {
		NODELET_DEBUG("handling lv_accept");
	}

	void PlatooningNode::hndl_lv_reject(const platooning::lv_reject& msg) {
		NODELET_DEBUG("handling lv_reject");
	}

	void PlatooningNode::hndl_fv_request(const platooning::fv_request& msg) {
		NODELET_DEBUG("handling fv_request");
	}

	void PlatooningNode::hndl_lv_broadcast(const platooning::lv_broadcast& msg) {
		NODELET_DEBUG("handling lv_broadcast");
	}

	void PlatooningNode::hndl_fv_heartbeat(const platooning::fv_heartbeat& msg) {
		NODELET_DEBUG("handling fv_heartbeat");
	}

	void PlatooningNode::hndl_platooning_toggle(const platooning::platooningToggle& msg) {
		NODELET_DEBUG("handling platooning_toggle");
	}
}

PLUGINLIB_EXPORT_CLASS(platooning::PlatooningNode, nodelet::Nodelet);

