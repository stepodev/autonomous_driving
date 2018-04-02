/**
 * @file testing/src/Moduletest_radiointerface.hpp
 * @author stepo
 * @date 22,03,2018
 * @brief Contains header of Moduletest_longitudinalprocessing class
 *
 */

/*****************************************************************************
** Includes
*****************************************************************************/

#include "platooning/Moduletest_longitudinalprocessing.hpp"

// %Tag(FULLTEXT)%
namespace platooning {

/*****************************************************************************
** Constructors
*****************************************************************************/

Moduletest_longitudinalprocessing::Moduletest_longitudinalprocessing()
	: io_worker_(io_service_), update_timer(io_service_) {

	thread_pool_.create_thread([this] { io_service_.run(); });

}

/*****************************************************************************
** Destructors
*****************************************************************************/

Moduletest_longitudinalprocessing::~Moduletest_longitudinalprocessing() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/

/**
* Set-up necessary publishers/subscribers
*/
void Moduletest_longitudinalprocessing::onInit() {

	name_ = "Moduletest_longitudinalprocessing";

	register_testcases(boost::bind(&Moduletest_longitudinalprocessing::test_send_new_data_recv_velocity, this));
	register_testcases(boost::bind(&Moduletest_longitudinalprocessing::test_change_velocity_keep_up, this));

	NODELET_INFO("[%s] init done", name_.c_str());

	start_tests();
}

/*****************************************************************************
** Testcases
*****************************************************************************/

void Moduletest_longitudinalprocessing::test_send_new_data_recv_velocity(){

	set_current_test("test_send_new_data_recv_velocity");
	NODELET_INFO("[%s] started testcase %s", name_.c_str(), get_current_test().c_str());

	//mockup publishers
	pub_map_.emplace(topics::SENSOR_DISTANCE, ros::Publisher());
	pub_map_[topics::SENSOR_DISTANCE] = nh_.advertise<platooning::distance>(topics::SENSOR_DISTANCE, 1);

	pub_map_.emplace(topics::TARGET_DISTANCE, ros::Publisher());
	pub_map_[topics::TARGET_DISTANCE] = nh_.advertise<platooning::distance>(topics::TARGET_DISTANCE, 1);

	pub_map_.emplace(topics::SENSOR_VELOCITY, ros::Publisher());
	pub_map_[topics::SENSOR_VELOCITY] = nh_.advertise<platooning::speed>(topics::SENSOR_VELOCITY, 1);

	pub_map_.emplace(topics::TARGET_SPEED, ros::Publisher());
	pub_map_[topics::TARGET_SPEED] = nh_.advertise<platooning::targetSpeed>(topics::TARGET_SPEED, 1);

	//wait for nodelet to subscribe to topic
	while(pub_map_[topics::SENSOR_DISTANCE].getNumSubscribers() < 1 ) {
		boost::this_thread::sleep_for( boost::chrono::milliseconds(200));
	}

	//send sensor distance, target distance, current speed

	auto target_speed = boost::shared_ptr<platooning::targetSpeed>(new platooning::targetSpeed);
	target_speed->target_speed = 3;
	pub_map_[topics::TARGET_SPEED].publish(target_speed);

	auto target_dist = boost::shared_ptr<platooning::targetDistance>(new platooning::targetDistance);
	target_dist->distance = 3;
	pub_map_[topics::TARGET_DISTANCE].publish(target_dist);

	auto current_speed = boost::shared_ptr<platooning::speed>(new platooning::speed);
	current_speed->speed = 3;
	pub_map_[topics::SENSOR_VELOCITY].publish(current_speed);
	pub_map_[topics::SENSOR_VELOCITY].publish(current_speed);

	//mockup subscriber to velocity to catch calculated velocity
	sub_map_.emplace(topics::CALCULATED_VELOCITY, ros::Subscriber());
	sub_map_[topics::CALCULATED_VELOCITY] = nh_.subscribe(topics::CALCULATED_VELOCITY, 1,
	                                               &Moduletest_longitudinalprocessing::hndl_test_send_new_data_recv_velocity,
	                                               this);

	auto sensor_distance = boost::shared_ptr<platooning::distance>(new platooning::distance);
	sensor_distance->distance = 3;
	pub_map_[topics::SENSOR_DISTANCE].publish(sensor_distance);
	boost::this_thread::sleep_for( boost::chrono::milliseconds(20));
	pub_map_[topics::SENSOR_DISTANCE].publish(sensor_distance);
}

void Moduletest_longitudinalprocessing::hndl_test_send_new_data_recv_velocity(const speed &msg) {
	TestResult res;
	res.success = true;

	//all data sent was 3, which should result in velocity = 3
	if (msg.speed != 3.0) {
		res.success = false;
		res.comment = "velocity was expected to be 3, was " + std::to_string(msg.speed) ;
	}

	if (!res.success) {
		NODELET_ERROR("[%s] error with %s ", name_.c_str(), res.comment.c_str());
	}

	finalize_test(res);

}
void Moduletest_longitudinalprocessing::test_change_velocity_keep_up() {
	set_current_test("test_change_velocity_keep_up");

	set_timeout(boost::posix_time::time_duration(boost::posix_time::seconds(10)));

	register_timeout_callback(boost::bind(&Moduletest_longitudinalprocessing::hndl_test_change_velocity_keep_up_scenario_over, this));

	NODELET_INFO("[%s] started testcase %s", name_.c_str(), get_current_test().c_str());

	//mockup publishers
	pub_map_.emplace(topics::SENSOR_DISTANCE, ros::Publisher());
	pub_map_[topics::SENSOR_DISTANCE] = nh_.advertise<platooning::distance>(topics::SENSOR_DISTANCE, 1);

	pub_map_.emplace(topics::TARGET_DISTANCE, ros::Publisher());
	pub_map_[topics::TARGET_DISTANCE] = nh_.advertise<platooning::distance>(topics::TARGET_DISTANCE, 1);

	pub_map_.emplace(topics::SENSOR_VELOCITY, ros::Publisher());
	pub_map_[topics::SENSOR_VELOCITY] = nh_.advertise<platooning::speed>(topics::SENSOR_VELOCITY, 1);

	pub_map_.emplace(topics::TARGET_SPEED, ros::Publisher());
	pub_map_[topics::TARGET_SPEED] = nh_.advertise<platooning::targetSpeed>(topics::TARGET_SPEED, 1);

	//wait for nodelet to subscribe to topic
	while(pub_map_[topics::SENSOR_DISTANCE].getNumSubscribers() < 1 ) {
		boost::this_thread::sleep_for( boost::chrono::milliseconds(200));
	}

	//send sensor distance, target distance, current speed

	auto target_speed = boost::shared_ptr<platooning::targetSpeed>(new platooning::targetSpeed);
	target_speed->target_speed = 8;
	pub_map_[topics::TARGET_SPEED].publish(target_speed);

	auto target_dist = boost::shared_ptr<platooning::targetDistance>(new platooning::targetDistance);
	target_dist->distance = 1;
	pub_map_[topics::TARGET_DISTANCE].publish(target_dist);

	auto current_speed = boost::shared_ptr<platooning::speed>(new platooning::speed);
	current_speed->speed = 2;
	pub_map_[topics::SENSOR_VELOCITY].publish(current_speed);
	pub_map_[topics::SENSOR_VELOCITY].publish(current_speed);

	//mockup subscriber to velocity to catch calculated velocity
	sub_map_.emplace(topics::CALCULATED_VELOCITY, ros::Subscriber());
	sub_map_[topics::CALCULATED_VELOCITY] = nh_.subscribe(topics::CALCULATED_VELOCITY, 1,
	                                                      &Moduletest_longitudinalprocessing::hndl_test_change_velocity_keep_up_velo,
	                                                      this);

	auto sensor_distance = boost::shared_ptr<platooning::distance>(new platooning::distance);
	sensor_distance->distance = 1;
	pub_map_[topics::SENSOR_DISTANCE].publish(sensor_distance);
	boost::this_thread::sleep_for( boost::chrono::milliseconds(20));
	pub_map_[topics::SENSOR_DISTANCE].publish(sensor_distance);

	lv_pos = 1;
	fv_pos = 0;
	lv_velo = 2;
	fv_velo = 2;

	//track start time to be able to change lv velo after a period
	start_time = boost::posix_time::second_clock::local_time();

	update_timer.expires_from_now(TIMER_FREQ);
	update_timer.async_wait(boost::bind(&Moduletest_longitudinalprocessing::hndl_test_change_velocity_keep_up_timer, this,
	                                                    boost::asio::placeholders::error));
}

/**
 * @brief called every TIMER_FREQ. Checks for a crash, updates positions, publishes new distances and speeds
 * @param e errorcode of timer
 */
void Moduletest_longitudinalprocessing::hndl_test_change_velocity_keep_up_timer(const boost::system::error_code &e) {

	if (boost::asio::error::operation_aborted == e) {
		NODELET_ERROR("[%s] check_dead_datasrc timer cancelled", name_.c_str());
		return;
	}

	lv_pos += lv_velo * 0.02;
	fv_pos += fv_velo * 0.02;

	/*
	std::cout << "###############################################################################################\n"
	          << "lv_velo " << lv_velo << " lv_pos " << lv_pos << "\nfv_velo " << fv_velo << " fv_pos " << fv_pos
	          << "\nvelodiff " << lv_velo - fv_velo
	          << "\ndistdiff " << lv_pos - fv_pos << std::endl;
	*/

	if( lv_pos - fv_pos <= 0.0f ) {
		TestResult res;
		res.success = false;
		res.comment = "lv_pos is " + std::to_string(lv_pos) +  " fv_pos is:" + std::to_string(fv_pos)
			+ " distance is: " + std::to_string( lv_pos -fv_pos ) + " so crash";

		update_timer.cancel();
		finalize_test(res);
		return;
	}

	//changes velos of lv 2 -> 4 -> 6 -> 1
	int runtime = (boost::posix_time::second_clock::local_time() - start_time ).total_seconds();

	if( runtime >= 1 && runtime < 2) {
		lv_velo = 4;
	}

	if( runtime >= 4 && runtime < 5) {
		lv_velo = 6;
	}

	if( runtime >= 6 && runtime < 7) {
		lv_velo = 1;
	}

	//publish speed and distance
	auto current_speed = boost::shared_ptr<platooning::speed>(new platooning::speed);
	current_speed->speed = fv_velo;
	pub_map_[topics::SENSOR_VELOCITY].publish(current_speed);

	auto sensor_distance = boost::shared_ptr<platooning::distance>(new platooning::distance);
	sensor_distance->distance = std::min(lv_pos - fv_pos, 5.0f);
	pub_map_[topics::SENSOR_DISTANCE].publish(sensor_distance);

	update_timer.expires_from_now(TIMER_FREQ);
	update_timer.async_wait(boost::bind(&Moduletest_longitudinalprocessing::hndl_test_change_velocity_keep_up_timer, this,
	                                                    boost::asio::placeholders::error));
}

void Moduletest_longitudinalprocessing::hndl_test_change_velocity_keep_up_velo(const platooning::speed &s) {
	fv_velo = std::min(s.speed,10.0f);
}

void Moduletest_longitudinalprocessing::hndl_test_change_velocity_keep_up_scenario_over() {
	TestResult res;
	res.success = true;
	res.comment = "moduletest scenario timer ran out.";

	if( lv_pos - fv_pos >= 1.1 ) {
		res.success = false;
		res.comment = "lv_pos is " + std::to_string(lv_pos) +  " fv_pos is:" + std::to_string(fv_pos)
			+ " distance is: " + std::to_string( lv_pos - fv_pos ) + " so too far";

		update_timer.cancel();
		finalize_test(res);
		return;
	}

	if (!res.success) {
		NODELET_ERROR("[%s] error with %s ", name_.c_str(), res.comment.c_str());
	}

	finalize_test(res);
}

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Moduletest_longitudinalprocessing, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
