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

#include "platooning/Moduletest_platooning.hpp"
#include "platooning/Platooning.hpp"

// %Tag(FULLTEXT)%
namespace platooning {

/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

Moduletest_platooning::Moduletest_platooning() = default;

/*****************************************************************************
** Destructors
*****************************************************************************/

Moduletest_platooning::~Moduletest_platooning() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/

/**
* Set-up necessary publishers/subscribers
* @return true, if successful
*/
void Moduletest_platooning::onInit() {

	name_ = "Moduletest_platooning";

	register_testcases(boost::bind(&Moduletest_platooning::send_platoontoggle_recv_platoonstate_creating, this));
	register_testcases(boost::bind(&Moduletest_platooning::send_fv_request_recv_lv_accept, this));
	register_testcases(boost::bind(&Moduletest_platooning::send_platoontoggle_recv_error_lv, this));

	NODELET_INFO("[%s] init done", name_.c_str());

	start_tests();
}

/*****************************************************************************
** Testcases
*****************************************************************************/

void Moduletest_platooning::send_platoontoggle_recv_platoonstate_creating() {

	set_current_test("send_platoontoggle_recv_platoonstate_creating");

	pub_map_.emplace(topics::TOGGLE_PLATOONING, ros::Publisher());
	pub_map_[topics::TOGGLE_PLATOONING] = nh_.advertise<platooningToggle>(topics::TOGGLE_PLATOONING, 1);

	sub_map_.emplace(topics::PLATOONINGSTATE, ros::Subscriber());
	sub_map_[topics::PLATOONINGSTATE] = nh_.subscribe(topics::PLATOONINGSTATE, 1,
	                                                &Moduletest_platooning::hndl_testcase_send_platoontoggle_recv_platoonstate_creating,
	                                                this);


	//wait for platooning nodelet to subscribe
	while(pub_map_[topics::TOGGLE_PLATOONING].getNumSubscribers() < 1 ) {
		boost::this_thread::sleep_for( boost::chrono::milliseconds(200));
	}

	auto msg = boost::shared_ptr<platooningToggle>( new platooningToggle);
	msg->enable_platooning = true;
	msg->platoon_speed = 3;
	msg->inner_platoon_distance = 4;

	pub_map_[topics::TOGGLE_PLATOONING].publish(msg);

}

void Moduletest_platooning::hndl_testcase_send_platoontoggle_recv_platoonstate_creating(const platooningState &msg) {

	TestResult res;
	res.success = true;

	if (!msg.platoon_members.empty()) {
		res.success = false;
		res.comment = "platoon not empty. was " + std::to_string(msg.platoon_members.size());
	}

	if( msg.ipd != 4 ) {
		res.success = false;
		res.comment += "\nidp should be 4 was " + std::to_string( msg.ipd);
	}

	if( msg.ps != 3) {
		res.success = false;
		res.comment += "\nps should be 3 was " + std::to_string( msg.ps);
	}

	if( msg.platooning_state != to_string(PlatooningModeEnum::CREATING) ) {
		res.success = false;
		res.comment += "\nps should be CREATING, was " + to_string(PlatooningModeEnum::CREATING);
	}

	if (!res.success) {
		NODELET_ERROR("[%s] error with %s", name_.c_str(), res.comment.c_str());
	}

	//cleanup
	auto msg_to_send = boost::shared_ptr<platooningToggle>( new platooningToggle);
	msg_to_send->enable_platooning = false;
	pub_map_[topics::TOGGLE_PLATOONING].publish(msg_to_send);

	finalize_test(res);

}

void Moduletest_platooning::send_platoontoggle_recv_error_lv() {

	set_current_test("send_platoontoggle_recv_error_lv");

	pub_map_.emplace(topics::TOGGLE_PLATOONING, ros::Publisher());
	pub_map_[topics::TOGGLE_PLATOONING] = nh_.advertise<platooningToggle>(topics::TOGGLE_PLATOONING, 1);

	sub_map_.emplace(topics::PLATOONINGSTATE, ros::Subscriber());
	sub_map_[topics::PLATOONINGSTATE] = nh_.subscribe(topics::PLATOONINGSTATE, 1,
													  &Moduletest_platooning::hndl_testcase_send_platoontoggle_recv_error_lv,
													  this);

	pub_map_.emplace(topics::IN_FV_REQUEST, ros::Publisher());
	pub_map_[topics::IN_FV_REQUEST] = nh_.advertise<fv_request>(topics::IN_FV_REQUEST, 1);

	//TODO: du brauchst hier noch einen handler, der das lv_accept nimmt.
//	sub_map_.emplace(topics::OUT_LV_ACCEPT, ros::Subscriber());
//	sub_map_[topics::OUT_LV_ACCEPT] = nh_.subscribe(topics::OUT_LV_ACCEPT, 1);


	//wait for platooning nodelet to subscribe
	while(pub_map_[topics::TOGGLE_PLATOONING].getNumSubscribers() < 1 ) {
		boost::this_thread::sleep_for( boost::chrono::milliseconds(200));
	}

	//wait for fv_request nodelet to subscribe
	while(pub_map_[topics::IN_FV_REQUEST].getNumSubscribers() < 1 ) {
		boost::this_thread::sleep_for( boost::chrono::milliseconds(200));
	}

	//wait for lv_accept nodelet to subscribe
	while(sub_map_[topics::OUT_LV_ACCEPT].getNumPublishers() < 1 ) {
		boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
	}

	// start platooning
	auto toggle_msg_on = boost::shared_ptr<platooningToggle>( new platooningToggle);
	toggle_msg_on->enable_platooning = true;

	pub_map_[topics::TOGGLE_PLATOONING].publish(toggle_msg_on);

	// send fv request
	auto req_msg = boost::shared_ptr<fv_request>( new fv_request);
	req_msg->src_vehicle = 5;
	pub_map_[topics::IN_FV_REQUEST].publish(req_msg);

	//receive lv accept??

	// actual test starts: lv turns off platoonign while platoon has fv
	auto toggle_msg_off = boost::shared_ptr<platooningToggle>( new platooningToggle);
	toggle_msg_off->enable_platooning = false;
	toggle_msg_off->lvfv = "LV";

	pub_map_[topics::TOGGLE_PLATOONING].publish(toggle_msg_off);
}

//TODO: ich glaube es wird keine platooningState message geschickt wenn ein platooningToggle nicht angenommen wird. das wird nur geschickt wenn sich am platoon was ändert.
	// ich würde auf die platooningState warten und erwarten, dass das timeout kommt.
void Moduletest_platooning::hndl_testcase_send_platoontoggle_recv_error_lv(const platooningState &msg) {

	TestResult res;
	res.success = true;

	if( msg.platooning_state != to_string(PlatooningModeEnum::RUNNING) ||
										  msg.platooning_state != to_string( PlatooningModeEnum::CREATING) ) {
		res.success = false;
		res.comment += "\nps should be CREATING or RUNNING, was " + to_string(PlatooningModeEnum::CREATING);
	}

	//TODO: warum schickst du die toggle message?
	//msg_to_send->enable_platooning = false;
	//pub_map_[topics::TOGGLE_PLATOONING].publish(msg_to_send);

	//TODO: wieder die platooningNode in IDLE setzen bevor wir finalizen, damit der nächste test eine node im startzustand bekommt. siehe cleanup in dem ersten test
	finalize_test(res);
}

void Moduletest_platooning::send_fv_request_recv_lv_accept() {

	set_current_test("send_fv_request_recv_lv_accept");

	pub_map_.emplace(topics::TOGGLE_PLATOONING, ros::Publisher());
	pub_map_[topics::TOGGLE_PLATOONING] = nh_.advertise<platooningToggle>(topics::TOGGLE_PLATOONING, 1);

	pub_map_.emplace(topics::IN_FV_REQUEST, ros::Publisher());
	pub_map_[topics::IN_FV_REQUEST] = nh_.advertise<fv_request>(topics::IN_FV_REQUEST, 1);

	sub_map_.emplace(topics::OUT_LV_ACCEPT, ros::Subscriber());
	sub_map_[topics::OUT_LV_ACCEPT] = nh_.subscribe(topics::OUT_LV_ACCEPT, 1,
	                                                  &Moduletest_platooning::hndl_tc_send_fv_request_recv_lv_accept,
	                                                  this);

	//wait for platooning nodelet to subscribe
	while(pub_map_[topics::TOGGLE_PLATOONING].getNumSubscribers() < 1 ) {
		boost::this_thread::sleep_for( boost::chrono::milliseconds(200));
	}

	//wait for platooning nodelet to subscribe
	while(pub_map_[topics::IN_FV_REQUEST].getNumSubscribers() < 1 ) {
		boost::this_thread::sleep_for( boost::chrono::milliseconds(200));
	}

	//wait for platooning nodelet to subscribe
	while(sub_map_[topics::OUT_LV_ACCEPT].getNumPublishers() < 1 ) {
		boost::this_thread::sleep_for( boost::chrono::milliseconds(200));
	}

	auto toggle_msg = boost::shared_ptr<platooningToggle>( new platooningToggle);
	toggle_msg->enable_platooning = true;
	toggle_msg->platoon_speed = 3;
	toggle_msg->inner_platoon_distance = 4;

	pub_map_[topics::TOGGLE_PLATOONING].publish(toggle_msg);

	auto req_msg = boost::shared_ptr<fv_request>( new fv_request);
	req_msg->src_vehicle = 5;
	pub_map_[topics::IN_FV_REQUEST].publish(req_msg);


}
void Moduletest_platooning::hndl_tc_send_fv_request_recv_lv_accept( lv_accept msg ) {

	TestResult res;
	res.success = true;

	if( msg.dst_vehicle != 5 ) {
		res.success = false;
		res.comment = "wrong destination vehicle. shoudlve been 5, was " + std::to_string(msg.dst_vehicle);
	}

	finalize_test(res);

}



} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Moduletest_platooning, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
