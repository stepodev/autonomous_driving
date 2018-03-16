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

#include "platooning/Moduletest_messagedistribution.hpp"

// %Tag(FULLTEXT)%
namespace platooning {

/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

Moduletest_messagedistribution::Moduletest_messagedistribution() = default;

/*****************************************************************************
** Destructors
*****************************************************************************/

Moduletest_messagedistribution::~Moduletest_messagedistribution() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/

/**
* Set-up necessary publishers/subscribers
* @return true, if successful
*/
void Moduletest_messagedistribution::onInit() {

	name_ = "Moduletest_messagedistribution";

	register_testcases(boost::bind(&Moduletest_messagedistribution::pub_in_platoonMsg_recv_fv_request, this));
	register_testcases(boost::bind(&Moduletest_messagedistribution::pub_in_platoonMsg_recv_lv_accept, this));

	NODELET_INFO("[%s] init done", name_.c_str());

	start_tests();
};

/*****************************************************************************
** Testcases
*****************************************************************************/

void Moduletest_messagedistribution::pub_in_platoonMsg_recv_fv_request() {

	set_current_test("pub_in_platoonMsg_recv_fv_request");
	NODELET_INFO("[%s] started testcase %s", name_.c_str(), get_current_test().c_str());

	//mockup publishers
	pub_map_.clear();
	pub_map_.emplace(topics::IN_PLATOONING_MSG, ros::Publisher());
	pub_map_[topics::IN_PLATOONING_MSG] = nh_.advertise<platooning::platoonProtocol>(topics::IN_PLATOONING_MSG, 1);

	//mockup subscribers
	sub_map_.clear();
	sub_map_.emplace(topics::IN_FV_REQUEST, ros::Subscriber());
	sub_map_[topics::IN_FV_LEAVE] = nh_.subscribe(topics::IN_FV_REQUEST, 1,
	                                              &Moduletest_messagedistribution::hndl_pub_in_platoonMsg_recv_fv_request,
	                                              this);

	boost::shared_ptr<platoonProtocol> inmsg = boost::shared_ptr<platoonProtocol>(new platoonProtocol);

	fv_request msg;
	msg.src_vehicle = 3;

	inmsg->message_type = FV_REQUEST;
	inmsg->payload = MessageTypes::encode_message(msg);

	pub_map_[topics::IN_PLATOONING_MSG].publish(inmsg);

}

void Moduletest_messagedistribution::hndl_pub_in_platoonMsg_recv_fv_request(platooning::fv_request msg) {

	TestResult res;
	res.success = true;

	if (msg.src_vehicle != 3) {
		res.success = false;
		res.comment = std::string("pub_in_platoonMsg_recv_fv_request:") + "src_vehicle:3 == "
			+ std::to_string(msg.src_vehicle)
			+ "=" + std::to_string(msg.src_vehicle == 3);
	}

	if (msg.src_vehicle == 3) {
		res.success = true;
	}

	finalize_test(res);

}
void Moduletest_messagedistribution::pub_in_platoonMsg_recv_lv_accept() {
	set_current_test("pub_in_platoonMsg_recv_lv_accept");
	NODELET_INFO("[%s] started testcase %s", name_.c_str(), get_current_test().c_str());

	//mockup publishers
	pub_map_.clear();
	pub_map_.emplace(topics::IN_PLATOONING_MSG, ros::Publisher());
	pub_map_[topics::IN_PLATOONING_MSG] = nh_.advertise<platooning::platoonProtocol>(topics::IN_PLATOONING_MSG, 1);

	//mockup subscribers
	sub_map_.clear();
	sub_map_.emplace(topics::IN_LV_ACCEPT, ros::Subscriber());
	sub_map_[topics::IN_LV_ACCEPT] = nh_.subscribe(topics::IN_LV_ACCEPT, 1,
	                                              &Moduletest_messagedistribution::hndl_pub_in_platoonMsg_recv_lv_accept,
	                                              this);

	boost::shared_ptr<platoonProtocol> inmsg = boost::shared_ptr<platoonProtocol>(new platoonProtocol);

	lv_accept msg;
	msg.src_vehicle = 3;
	msg.platoon_id = 4;
	msg.dst_vehicle = 5;

	inmsg->message_type = LV_ACCEPT;
	inmsg->payload = MessageTypes::encode_message(msg);

	pub_map_[topics::IN_PLATOONING_MSG].publish(inmsg);
}

void Moduletest_messagedistribution::hndl_pub_in_platoonMsg_recv_lv_accept(platooning::lv_accept msg) {
	TestResult res;
	res.success = true;

	if (msg.src_vehicle != 3 || msg.dst_vehicle != 5 || msg.platoon_id != 4 ) {
		res.success = false;
		res.comment = get_current_test()
			+ " src_vehicle:3 == " + std::to_string(msg.src_vehicle) + "=" + std::to_string(msg.src_vehicle == 3) + "\n"
		+ " dst_vehicle:5 == " + std::to_string(msg.dst_vehicle) + "=" + std::to_string(msg.dst_vehicle == 5) + "\n"
		+ " platoon_id:4 == " + std::to_string(msg.platoon_id) + "=" + std::to_string(msg.platoon_id == 4) + "\n";
	}

	if (msg.src_vehicle == 3 || msg.dst_vehicle == 5 || msg.platoon_id == 4 ) {
		res.success = true;
	}

	finalize_test(res);
}

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Moduletest_messagedistribution, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
