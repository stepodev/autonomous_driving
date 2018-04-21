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

Moduletest_platooning::~Moduletest_platooning() {

};


/*****************************************************************************
** Initializers
*****************************************************************************/

/**
* Set-up necessary publishers/subscribers
* @return true, if successful
*/
void Moduletest_platooning::onInit() {

	name_ = "Moduletest_platooning";

	register_testcases(boost::bind(&Moduletest_platooning::test_send_platoontoggle_recv_platoonstate_creating, this));
	register_testcases(boost::bind(&Moduletest_platooning::test_send_fv_request_recv_lv_accept, this));

	thread_pool_.create_thread( [this] {
		try {
			//Services
			ros::ServiceClient
				srv_client_ = nh_.serviceClient<platooning::getVehicleId>(platooning_services::VEHICLE_ID);

			ros::Duration sec;
			sec.sec = 20;
			if (srv_client_.waitForExistence(ros::Duration(sec))) {

				platooning::getVehicleId::Request req;
				platooning::getVehicleId::Response res;

				if (srv_client_.call(req, res)) {
					this->vehicle_id_ = res.vehicle_id;
				}
			}

			start_tests();
		}
		catch (std::exception &ex) {
			NODELET_ERROR("[%s] hndl_gazupdate failed with %s ", name_.c_str(), ex.what());
		}
	});

	vehicle_id_ = 1;

	thread_pool_.create_thread([this] {

	  try {
		  //Services
		  ros::ServiceClient
			  srv_client_ = nh_.serviceClient<platooning::getVehicleId>(platooning_services::VEHICLE_ID);

		  ros::Duration sec;
		  sec.sec = 20;
		  if (srv_client_.waitForExistence(ros::Duration(sec))) {

			  platooning::getVehicleId::Request req;
			  platooning::getVehicleId::Response res;

			  if (srv_client_.call(req, res)) {
				  this->vehicle_id_ = res.vehicle_id;
			  }
		  }
	  }
	  catch (std::exception &ex) {
		  NODELET_ERROR("[%s] hndl_gazupdate failed with %s ", name_.c_str(), ex.what());
	  }

	});

	NODELET_INFO("[%s] init done", name_.c_str());

}

/*****************************************************************************
** Testcases
*****************************************************************************/

void Moduletest_platooning::test_send_platoontoggle_recv_platoonstate_creating() {

	set_current_test("test_send_platoontoggle_recv_platoonstate_creating");

	register_timeout_callback(boost::bind(&Moduletest_platooning::hndl_test_send_platoontoggle_recv_platoonstate_creating_timeout, this));

	pub_map_.emplace(topics::TOGGLE_PLATOONING, ros::Publisher());
	pub_map_[topics::TOGGLE_PLATOONING] = nh_.advertise<platooningToggle>(topics::TOGGLE_PLATOONING, 1);

	sub_map_.emplace(topics::PLATOONINGSTATE, ros::Subscriber());
	sub_map_[topics::PLATOONINGSTATE] = nh_.subscribe(topics::PLATOONINGSTATE, 1,
	                                                  &Moduletest_platooning::hndl_test_send_platoontoggle_recv_platoonstate_creating,
	                                                this);

	//wait for platooning nodelet to subscribe
	while(pub_map_[topics::TOGGLE_PLATOONING].getNumSubscribers() < 1 ) {
		boost::this_thread::sleep_for( boost::chrono::milliseconds(200));
	}

	auto msg = boost::shared_ptr<platooningToggle>( new platooningToggle );
	msg->vehicle_id = vehicle_id_;
	msg->lvfv = "LV";
	msg->enable_platooning = true;
	msg->platoon_speed = 3;
	msg->inner_platoon_distance = 4;
	msg->lvfv = "LV";
	msg->vehicle_id = vehicle_id_;

	pub_map_[topics::TOGGLE_PLATOONING].publish(msg);

}

void Moduletest_platooning::hndl_test_send_platoontoggle_recv_platoonstate_creating(platooningState msg) {

	TestResult res;
	res.success = true;

	if (!msg.platoon_members.empty()) {
		res.success = false;
		res.comment = "platoon not empty. was " + std::to_string(msg.platoon_members.size());
	}

	if( msg.ipd != 4.0f ) {
		res.success = false;
		res.comment += "\nidp should be 4 was " + std::to_string( msg.ipd);
	}

	if( msg.ps != 3.0f) {
		res.success = false;
		res.comment += "\nps should be 3 was " + std::to_string( msg.ps);
	}

	if( msg.platooning_state != to_string(PlatooningModeEnum::CREATING) ) {
		res.success = false;
		res.comment += "\nmode should be CREATING, was \"" + to_string(PlatooningModeEnum::CREATING) + "\"";
	}

	platooningstate_.i_am_FV = msg.i_am_FV;
	platooningstate_.i_am_LV = msg.i_am_LV;
	platooningstate_.ipd = msg.ipd;
	platooningstate_.platoon_id = msg.platoon_id;
	platooningstate_.platoon_members = msg.platoon_members;
	platooningstate_.platooning_state = msg.platooning_state;
	platooningstate_.ps = msg.ps;
	platooningstate_.vehicle_id = msg.vehicle_id;

	if( res.success ) {
		//cleanup
		auto msg_to_send = boost::shared_ptr<platooningToggle>( new platooningToggle);
		msg_to_send->enable_platooning = false;
		pub_map_[topics::TOGGLE_PLATOONING].publish(msg_to_send);

		finalize_test(res);
	}
}

void Moduletest_platooning::hndl_test_send_platoontoggle_recv_platoonstate_creating_timeout() {
	TestResult res;
	res.success = false;
	res.comment = "timeout before proper platooningstate was received.";

	if (!platooningstate_.platoon_members.empty()) {
		res.success = false;
		res.comment = "platoon not empty. was " + std::to_string(platooningstate_.platoon_members.size());
	}

	if( platooningstate_.ipd != 4 ) {
		res.success = false;
		res.comment += "\nidp should be 4 was " + std::to_string( platooningstate_.ipd);
	}

	if( platooningstate_.ps != 3) {
		res.success = false;
		res.comment += "\nps should be 3 was " + std::to_string( platooningstate_.ps);
	}

	if( platooningstate_.platooning_state != to_string(PlatooningModeEnum::CREATING) ) {
		res.success = false;
		res.comment += "\nstate should be \"CREATING\", was \"" + to_string(PlatooningModeEnum::CREATING) + "\"";
	}

	//cleanup
	auto msg_to_send = boost::shared_ptr<platooningToggle>( new platooningToggle);
	msg_to_send->vehicle_id = vehicle_id_;
	msg_to_send->enable_platooning = false;
	pub_map_[topics::TOGGLE_PLATOONING].publish(msg_to_send);

	finalize_test(res);
}

void Moduletest_platooning::test_send_fv_request_recv_lv_accept() {

	set_current_test("test_send_fv_request_recv_lv_accept");

	pub_map_.emplace(topics::TOGGLE_PLATOONING, nh_.advertise<platooningToggle>(topics::TOGGLE_PLATOONING, 1));

	pub_map_.emplace(topics::IN_FV_REQUEST, nh_.advertise<fv_request>(topics::IN_FV_REQUEST, 1));

	sub_map_.emplace(topics::OUT_LV_ACCEPT, nh_.subscribe(topics::OUT_LV_ACCEPT, 1,
	                                                      &Moduletest_platooning::hndl_tc_send_fv_request_recv_lv_accept,
	                                                      this));

	//wait for platooning nodelet to subscribe
	while(pub_map_[topics::TOGGLE_PLATOONING].getNumSubscribers() < 1 ) {
		boost::this_thread::sleep_for( boost::chrono::milliseconds(200));
	}

	auto toggle_msg = boost::shared_ptr<platooningToggle>( new platooningToggle);
	toggle_msg->vehicle_id = vehicle_id_;
	toggle_msg->lvfv = "LV";
	toggle_msg->enable_platooning = true;
	toggle_msg->platoon_speed = 3;
	toggle_msg->inner_platoon_distance = 4;

	pub_map_[topics::TOGGLE_PLATOONING].publish(toggle_msg);

	//wait for platooning nodelet to subscribe
	while(pub_map_[topics::IN_FV_REQUEST].getNumSubscribers() < 1 ) {
		boost::this_thread::sleep_for( boost::chrono::milliseconds(200));
	}

	//wait for platooning nodelet to subscribe
	while(sub_map_[topics::OUT_LV_ACCEPT].getNumPublishers() < 1 ) {
		boost::this_thread::sleep_for( boost::chrono::milliseconds(200));
	}

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

	//cleanup

	pub_map_.emplace(topics::IN_FV_LEAVE, nh_.advertise<fv_leave>(topics::IN_FV_LEAVE, 1));

	auto leave_msg = boost::shared_ptr<fv_leave>( new fv_leave);
	leave_msg->src_vehicle = 5;
	leave_msg->platoon_id = msg.platoon_id;
	pub_map_[ topics::IN_FV_LEAVE].publish(leave_msg);


	auto toggle_msg = boost::shared_ptr<platooningToggle>( new platooningToggle);
	toggle_msg->vehicle_id = vehicle_id_;
	toggle_msg->enable_platooning = false;
	pub_map_[topics::TOGGLE_PLATOONING].publish(toggle_msg);

	finalize_test(res);
}

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Moduletest_platooning, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
