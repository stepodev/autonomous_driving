/**
 * @file /testing/src/Moduletest_prioritization.cpp
 *
 * @brief Nodelet implementation of Moduletest_prioritization
 *
 * @author stepo
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <platooning/Platooning.hpp>
#include "platooning/Moduletest_prioritization.hpp"

// %Tag(FULLTEXT)%
namespace platooning {

/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

Moduletest_prioritization::Moduletest_prioritization() = default;

/*****************************************************************************
** Destructors
*****************************************************************************/

Moduletest_prioritization::~Moduletest_prioritization() = default;

/*****************************************************************************
** Initializers
*****************************************************************************/

/**
* Set-up necessary publishers/subscribers
* @return true, if successful
*/
void Moduletest_prioritization::onInit() {

	name_ = "Moduletest_prioritization";

	register_testcases(boost::bind(&Moduletest_prioritization::test_remotecontrol_toggle_and_speed_recv_vehiclecontrol, this));
	register_testcases(boost::bind(&Moduletest_prioritization::test_platooning_toggle_and_speed_recv_vehiclecontrol, this));

	NODELET_INFO("[%s] init done", name_.c_str());

	vehicle_id_server_ = nh_.advertiseService(platooning_services::VEHICLE_ID, &Moduletest_prioritization::provide_vehicle_id, this);

	start_tests();
}

/*****************************************************************************
** Testcases
*****************************************************************************/

void Moduletest_prioritization::test_remotecontrol_toggle_and_speed_recv_vehiclecontrol() {
	set_current_test("test_remotecontrol_toggle_and_speed_recv_vehiclecontrol");
	NODELET_INFO("[%s] started testcase %s", name_.c_str(), get_current_test().c_str());

	//mockup publishers
	pub_map_.emplace(topics::TOGGLE_REMOTECONTROL, nh_.advertise<platooning::remotecontrolToggle>(topics::TOGGLE_REMOTECONTROL, 1));
	pub_map_.emplace(topics::INPUT_REMOTECONTROL, nh_.advertise<platooning::remotecontrolInput>(topics::INPUT_REMOTECONTROL, 1));

	sub_map_.emplace(topics::VEHICLE_CONTROL,
	                 nh_.subscribe(topics::VEHICLE_CONTROL, 1,&Moduletest_prioritization::hndl_test_remotecontrol_toggle_and_speed_recv_vehiclecontrol,this));

	while( sub_map_[topics::VEHICLE_CONTROL].getNumPublishers() < 1 ) {
		boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
	}

	//toggle remotecontrol on
	auto togglemsg = boost::shared_ptr<platooning::remotecontrolToggle>( new platooning::remotecontrolToggle );
	togglemsg->enable_remotecontrol = true;
	togglemsg->vehicle_id = 1;

	pub_map_[topics::TOGGLE_REMOTECONTROL].publish(togglemsg);

	//send remotecontrol input
	auto inputmsg = boost::shared_ptr<platooning::remotecontrolInput>( new platooning::remotecontrolInput );
	inputmsg->vehicle_id = 1;
	inputmsg->remote_speed = 2;
	inputmsg->remote_angle = 3;
	inputmsg->emergency_stop = false;

	pub_map_[topics::INPUT_REMOTECONTROL].publish(inputmsg);

	while( vehiclecontrol_received == false ) {
		boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
	}

	//check vehiclecontrol received
	boost::mutex::scoped_lock lock(mtx);

	TestResult res;

	if( vc_msg.velocity != 2 ) {
		res.comment = "velo shouldve been 2, was " + std::to_string(vc_msg.velocity) + "\n";
	}

	if( vc_msg.steering_angle != 3 ) {
		res.comment = "velo shouldve been 3, was " + std::to_string(vc_msg.steering_angle);
	}

	if( vc_msg.velocity != 2 || vc_msg.steering_angle != 3) {
		res.success = false;
		finalize_test(res);
		return;
	}

	vehiclecontrol_received = false;

	lock.unlock();

	//toggle remotecontrol
	togglemsg = boost::shared_ptr<platooning::remotecontrolToggle>( new platooning::remotecontrolToggle );
	togglemsg->enable_remotecontrol = false;
	togglemsg->vehicle_id = 1;
	pub_map_[topics::TOGGLE_REMOTECONTROL].publish(togglemsg);

	boost::this_thread::sleep_for(boost::chrono::milliseconds(200));

	//send remotecontrol input
	inputmsg = boost::shared_ptr<platooning::remotecontrolInput>( new platooning::remotecontrolInput );
	inputmsg->vehicle_id = 1;
	inputmsg->remote_speed = 2;
	inputmsg->remote_angle = 3;
	inputmsg->emergency_stop = false;
	pub_map_[topics::INPUT_REMOTECONTROL].publish(inputmsg);

	boost::this_thread::sleep_for(boost::chrono::seconds(2));

	if( vehiclecontrol_received ) {
		TestResult res;
		res.success = false;
		res.comment = "remotecontrol shouldve been toggled off, still received vehiclecontrol";
		finalize_test(res);
		return;
	}

	res.success = true;
	res.comment = "no vehiclecontrol received";
	finalize_test(res);

}

void Moduletest_prioritization::hndl_test_remotecontrol_toggle_and_speed_recv_vehiclecontrol(const platooning::vehicleControl &msg) {
	boost::mutex::scoped_lock lock(mtx);

	vehiclecontrol_received = true;

	vc_msg = msg;
}

bool Moduletest_prioritization::provide_vehicle_id(platooning::getVehicleId::Request &req,
                                                   platooning::getVehicleId::Response &res) {
		res.vehicle_id = 1;
		return true;
}

void Moduletest_prioritization::test_platooning_toggle_and_speed_recv_vehiclecontrol() {
	set_current_test("test_platooning_toggle_and_speed_recv_vehiclecontrol");
	NODELET_INFO("[%s] started testcase %s", name_.c_str(), get_current_test().c_str());

	//mockup publishers
	pub_map_.emplace(topics::TOGGLE_PLATOONING, nh_.advertise<platooning::platooningToggle>(topics::TOGGLE_PLATOONING, 1));
	pub_map_.emplace(topics::PLATOONINGSTATE, nh_.advertise<platooning::platooningState>(topics::PLATOONINGSTATE, 1));
	pub_map_.emplace(topics::SENSOR_VELOCITY, nh_.advertise<platooning::speed>(topics::SENSOR_VELOCITY, 1));
	pub_map_.emplace(topics::SENSOR_DISTANCE, nh_.advertise<platooning::distance>(topics::SENSOR_DISTANCE, 1));
	pub_map_.emplace(topics::CALCULATED_VELOCITY, nh_.advertise<platooning::speed>(topics::CALCULATED_VELOCITY, 1));

	sub_map_.emplace(topics::VEHICLE_CONTROL,
	                 nh_.subscribe(topics::VEHICLE_CONTROL, 1,&Moduletest_prioritization::hndl_callback_test_platooning_toggle_and_speed_recv_vehiclecontrol,this));

	while( sub_map_[topics::VEHICLE_CONTROL].getNumPublishers() < 1 ) {
		boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
	}

	//toggle platooning
	auto togglemsg = boost::shared_ptr<platooning::platooningToggle>( new platooning::platooningToggle );
	togglemsg->enable_platooning = true;
	togglemsg->vehicle_id = 1;

	pub_map_[topics::TOGGLE_PLATOONING].publish(togglemsg);

	//send state
	auto inputmsg = boost::shared_ptr<platooning::platooningState>( new platooning::platooningState );
	inputmsg->vehicle_id = 1;
	inputmsg->i_am_LV = false;
	inputmsg->i_am_FV = true;
	inputmsg->ipd = 1;
	inputmsg->ps = 1;
	inputmsg->platoon_id = 1;
	inputmsg->platooning_state = to_string(platooning::PlatooningModeEnum::RUNNING);
	inputmsg->platoon_members = { 1, 2 };

	vehiclecontrol_received = false;

	pub_map_[topics::PLATOONINGSTATE].publish(inputmsg);

	//publish calculated velocity
	auto velomsg = boost::shared_ptr<platooning::speed>( new platooning::speed );
	velomsg->speed = 1;
	pub_map_[topics::CALCULATED_VELOCITY].publish(velomsg);

	//check received vehiclecontrol
	while( vehiclecontrol_received == false ) {
		boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
	}

	//check vehiclecontrol received
	boost::mutex::scoped_lock lock(mtx);

	TestResult res;

	if( vc_msg.velocity != 1.0f ) {
		res.comment = "velo shouldve been 1.0f, was " + std::to_string(vc_msg.velocity) + "\n";
	}

	if( vc_msg.steering_angle != 0.0f ) {
		res.comment = "angle shouldve been 0.0f, was " + std::to_string(vc_msg.steering_angle);
	}

	if( vc_msg.velocity != 1.0f || vc_msg.steering_angle != 0.0f) {
		res.success = false;
		finalize_test(res);
		return;
	}

	vehiclecontrol_received = false;

	lock.unlock();

	//toggle platooning
	togglemsg = boost::shared_ptr<platooning::platooningToggle>( new platooning::platooningToggle );
	togglemsg->enable_platooning = false;
	togglemsg->vehicle_id = 1;

	pub_map_[topics::TOGGLE_PLATOONING].publish(togglemsg);

	//check not received vehiclecontrol
	boost::this_thread::sleep_for(boost::chrono::seconds(2));

	if( vehiclecontrol_received ) {
		TestResult res;
		res.success = false;
		res.comment = "platooning shouldve been toggled off, still received vehiclecontrol";
		finalize_test(res);
		return;
	}

	res.success = true;
	res.comment = "no vehiclecontrol received";
	finalize_test(res);

}

void Moduletest_prioritization::hndl_callback_test_platooning_toggle_and_speed_recv_vehiclecontrol(const platooning::vehicleControl &msg) {
	boost::mutex::scoped_lock lock(mtx);

	vehiclecontrol_received = true;

	vc_msg = msg;
}

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Moduletest_prioritization, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
