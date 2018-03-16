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

#include "platooning/Moduletest_longitudinalcontrol.hpp"

// %Tag(FULLTEXT)%
namespace platooning {

/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

Moduletest_longitudinalcontrol::Moduletest_longitudinalcontrol() = default;

/*****************************************************************************
** Destructors
*****************************************************************************/

Moduletest_longitudinalcontrol::~Moduletest_longitudinalcontrol() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/

/**
* Set-up necessary publishers/subscribers
* @return true, if successful
*/
void Moduletest_longitudinalcontrol::onInit() {

	name_ = "Moduletest_longitudinalcontrol";

	register_testcases(boost::bind(&Moduletest_longitudinalcontrol::send_new_data_recv_accel, this));

	NODELET_INFO("[%s] init done", name_.c_str());

	start_tests();
}

/*****************************************************************************
** Testcases
*****************************************************************************/

void Moduletest_longitudinalcontrol::send_new_data_recv_accel(){

	set_current_test("send_sensor_distance_recv_accel");
	NODELET_INFO("[%s] started testcase %s", name_.c_str(), get_current_test().c_str());

	//mockup publishers
	pub_map_.clear();
	pub_map_.emplace(topics::SENSOR_DISTANCE_TO_OBJ, ros::Publisher());
	pub_map_[topics::SENSOR_DISTANCE_TO_OBJ] = nh_.advertise<platooning::distance>(topics::SENSOR_DISTANCE_TO_OBJ, 1);

	pub_map_.emplace(topics::TARGET_DISTANCE, ros::Publisher());
	pub_map_[topics::TARGET_DISTANCE] = nh_.advertise<platooning::distance>(topics::TARGET_DISTANCE, 1);

	pub_map_.emplace(topics::CURRENT_SPEED, ros::Publisher());
	pub_map_[topics::CURRENT_SPEED] = nh_.advertise<platooning::speed>(topics::CURRENT_SPEED, 1);

	pub_map_.emplace(topics::TARGET_SPEED, ros::Publisher());
	pub_map_[topics::TARGET_SPEED] = nh_.advertise<platooning::targetSpeed>(topics::TARGET_SPEED, 1);

	//wait for nodelet to subscribe to topic
	while(pub_map_[topics::SENSOR_DISTANCE_TO_OBJ].getNumSubscribers() < 1 ) {
		boost::this_thread::sleep_for( boost::chrono::milliseconds(200));
	}

	//send sensor distance, target distance, current speed
	auto sensor_distance = boost::shared_ptr<platooning::distance>(new platooning::distance);
	sensor_distance->distance = 3;
	pub_map_[topics::SENSOR_DISTANCE_TO_OBJ].publish(sensor_distance);

	auto target_dist = boost::shared_ptr<platooning::targetDistance>(new platooning::targetDistance);
	target_dist->distance = 3;
	pub_map_[topics::TARGET_DISTANCE].publish(target_dist);

	auto current_speed = boost::shared_ptr<platooning::speed>(new platooning::speed);
	current_speed->speed = 3;
	pub_map_[topics::CURRENT_SPEED].publish(current_speed);

	//mockup subscriber to accel to catch calculated accel after sent targetspeed
	sub_map_.clear();
	sub_map_.emplace(topics::ACCELERATION, ros::Subscriber());
	sub_map_[topics::ACCELERATION] = nh_.subscribe(topics::ACCELERATION, 1,
	                                               &Moduletest_longitudinalcontrol::hndl_tc_send_new_data_recv_accel,
	                                               this);

	auto target_speed = boost::shared_ptr<platooning::targetSpeed>(new platooning::targetSpeed);
	target_speed->target_speed = 3;
	pub_map_[topics::TARGET_SPEED].publish(target_speed);
}

void Moduletest_longitudinalcontrol::hndl_tc_send_new_data_recv_accel(platooning::acceleration msg) {
	TestResult res;
	res.success = true;

	//all data sent was 3, which should result in accel 0
	if (msg.accelleration != 0) {
		res.success = false;
		res.comment = "acceleration was expected to be 1, was " + std::to_string(msg.accelleration) ;
	}

	if (!res.success) {
		NODELET_ERROR("[%s] error with %s ", name_.c_str(), res.comment.c_str());
	}

	finalize_test(res);

}

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Moduletest_longitudinalcontrol, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
