/**
 * @file include/platooning/LongitudinalProcessing.hpp
 * @author stepo
 * @date 22.03.2018
 * @brief LongitudinalProcessing implementation
 */

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include "platooning/LongitudinalProcessing.hpp"

namespace platooning {

/*****************************************************************************
** Constructors
*****************************************************************************/

LongitudinalProcessing::LongitudinalProcessing()
	: io_worker_(io_service_), detect_dead_datasource_timer(io_service_) {

	thread_pool_.create_thread([this] { io_service_.run(); });

};

/*****************************************************************************
** Destructors
*****************************************************************************/

LongitudinalProcessing::~LongitudinalProcessing() {
	io_service_.stop();
}


/*****************************************************************************
** Initializers
*****************************************************************************/

/**
* Set-up necessary publishers/subscribers
*/
void LongitudinalProcessing::onInit() {

	sub_current_velocity_ = nh_.subscribe(topics::SENSOR_VELOCITY, 1,
	                                      &LongitudinalProcessing::hndl_current_velocity, this);

	sub_target_velocity_ = nh_.subscribe(topics::TARGET_SPEED, 1,
	                                     &LongitudinalProcessing::hndl_targetSpeed, this);

	sub_distance_to_obj_ = nh_.subscribe(topics::SENSOR_DISTANCE, 1,
	                                     &LongitudinalProcessing::hndl_distance_from_sensor, this);

	sub_target_distance_ = nh_.subscribe(topics::TARGET_DISTANCE, 1,
	                                     &LongitudinalProcessing::hndl_target_distance, this);

	pub_velocity_ = nh_.advertise<platooning::speed>(topics::CALCULATED_VELOCITY, 1);

	detect_dead_datasource_timer.expires_from_now(SOURCECHECK_FREQ);
	detect_dead_datasource_timer.async_wait(boost::bind(&LongitudinalProcessing::check_dead_datasrc, this,
	                                                    boost::asio::placeholders::error));

	NODELET_INFO("[%s] init done", name_.c_str());

}

/*****************************************************************************
** Handlers
*****************************************************************************/

/**
 * @brief Receives sensor distance, checks whether we have two measurements and updates velocity if true. Sets flag that
 * distance measurement was received.
 * @param msg a message containing the distance to an object
 */
void LongitudinalProcessing::hndl_distance_from_sensor(const platooning::distance &msg) {

	dead_data_reminder_warn_ = true;

	data_src_flags |= RANGE_DATA_CHECK;

	try {
		//check if we received new data
		boost::recursive_mutex::scoped_lock l(calc_mutex_);

		previous_distance_ = current_distance_;
		current_distance_ = msg.distance;

		previous_distance_timestamp_ = current_distance_timestamp_;
		current_distance_timestamp_ = boost::posix_time::microsec_clock::local_time();

		if (previous_distance_timestamp_ == boost::posix_time::min_date_time) {
			return;
		}

		update_velocity();
	} catch (std::exception &ex) {
		NODELET_ERROR("[%s] hndl_distance_from_sensor crash with %s", name_.c_str(), ex.what());
	}
}

/**
 * @brief received new target distance. Checks whether new distance is in appropriate range and if so, sets the
 * target distance of the pd controller.
 * @param msg a message with new target distance
 */
void LongitudinalProcessing::hndl_target_distance(const platooning::targetDistance &msg) {
	try {
		if (msg.distance != -pd_controller_.get_target_position()) {

			boost::recursive_mutex::scoped_lock l(calc_mutex_);

			if (msg.distance < MINIMUM_DISTANCE) {
				NODELET_WARN("[%s] target distance shorter than 2. Setting to 2.", name_.c_str());
				pd_controller_.set_target_position(-MINIMUM_DISTANCE);

			} else {
				pd_controller_.set_target_position(-msg.distance);
			}
		}
	} catch (std::exception &ex) {
		NODELET_ERROR("[%s] hndl_target_distance crash with %s", name_.c_str(), ex.what());
	}
};

/**
 * @brief Handles current velocity data. updates it if new. Sets flag that it was received.
 * @param msg a message containing current velocity
 */
void LongitudinalProcessing::hndl_current_velocity(const platooning::speed &msg) {

	dead_data_reminder_warn_ = true;

	data_src_flags |= VELOCITY_DATA_CHECK;

	try {
		if (msg.speed != current_velocity_) {

			boost::recursive_mutex::scoped_lock l(calc_mutex_);

			current_velocity_ = msg.speed;
		}
	} catch (std::exception &ex) {
		NODELET_ERROR("[%s] hndl_current_velocity crash with %s", name_.c_str(), ex.what());
	}
}

/**
 * @brief Received new target speed.
 * @param msg a message containing new target speed
 */
void LongitudinalProcessing::hndl_targetSpeed(const platooning::targetSpeed &msg) {

	try {
		if (msg.target_speed != target_velocity_) {

			boost::recursive_mutex::scoped_lock l(calc_mutex_);

			target_velocity_ = msg.target_speed;
		}
	} catch (std::exception &ex) {
		NODELET_ERROR("[%s] hndl_targetSpeed crash with %s", name_.c_str(), ex.what());
	}
}

/**
 * @brief Publishes velocity based on current target velocity, target distance, current velocity and current distance
 *
 * Calculates relative speed based on change in distance, then calculates how much that speed should be adjusted to
 * get to target distance.
 *
 * using the pd controller
 */
void LongitudinalProcessing::update_velocity() {

    //wait for second ranging
	if (previous_distance_timestamp_ == boost::posix_time::min_date_time) {
		return;
	}

	boost::recursive_mutex::scoped_lock l(calc_mutex_);

	auto outmsg = boost::shared_ptr<platooning::speed>(new platooning::speed);

	//calculate time_step in seconds since velocity is in m/s and distance in m
	float time_step = (current_distance_timestamp_ - previous_distance_timestamp_).total_milliseconds() / 1000.0f;

	//ensure we are not dividing by 0
	if (time_step <= 0.0f) {
		return;
	}

	float relative_velocity = (previous_distance_ - current_distance_) / time_step;

	//less than 0.8 units distance and driving towards: should emergency stop
	if( current_distance_ <= EMERGENCY_DISTANCE && relative_velocity < 0) {
		outmsg->speed = -5.0f;

		pub_velocity_.publish(outmsg);
		return;
	}

	//if relative velo diff is slower than 5 units per second and distance is off by less than 5 units, just keep speed
	if( fabsf(relative_velocity) < 0.05 && fabsf(current_distance_ - -pd_controller_.get_target_position()) < 0.05 )  {
		//dont publish? or publish current velo?
	} else {
		//we want to drive the speed of guy in front of us, plus catch-up/breaking velo
		//distance to negative since we calculate distance to point 0 on x axis
		float calculated_velocity = current_velocity_ +
			pd_controller_.calulate_velocity(-current_distance_, relative_velocity);

/*
		NODELET_INFO(
			"ms: %i time_step: %f range_diff: %f relative_vel: %f current_vel: %f calc_vel: %f\ndist:%f target:%f prev_dist %f",
			(int) (current_distance_timestamp_ - previous_distance_timestamp_).total_milliseconds(),
			time_step,
			current_distance_ - previous_distance_,
			relative_velocity,
			current_velocity_,
			calculated_velocity,
			current_distance_,
			-pd_controller_.get_target_position(),
			previous_distance_);
*/
		outmsg->speed = calculated_velocity;

		pub_velocity_.publish(outmsg);
	}
}

/**
 * @brief Handler for timer that checks for up-to-date distance and velocity measurements. Currently calls for an
 * emergency stop if not
 * @param e the timer error code
 */
void LongitudinalProcessing::check_dead_datasrc(const boost::system::error_code &e) {

	if (boost::asio::error::operation_aborted == e) {
		NODELET_ERROR("[%s] check_dead_datasrc timer cancelled", name_.c_str());
		return;
	}

	if ((data_src_flags & RANGE_DATA_CHECK) != RANGE_DATA_CHECK) {


		if( dead_data_reminder_warn_ ) {
			NODELET_ERROR("[%s] RANGE_DATA_CHECK false. range data not received for %i",
			              name_.c_str(), (int) SOURCECHECK_FREQ.total_milliseconds());
			dead_data_reminder_warn_ = false;
		}

		auto outmsg = boost::shared_ptr<platooning::speed>(new platooning::speed);
		outmsg->speed = 0;
		pub_velocity_.publish(outmsg);
		dead_data_reminder_warn_ = false;
	}

	if ((data_src_flags & VELOCITY_DATA_CHECK) != VELOCITY_DATA_CHECK) {
		if( dead_data_reminder_warn_) {
			NODELET_ERROR("[%s] VELOCITY_DATA_CHECK false. velocity data not received for %i",
			              name_.c_str(), (int) SOURCECHECK_FREQ.total_milliseconds());
			dead_data_reminder_warn_ = false;
		}

		auto outmsg = boost::shared_ptr<platooning::speed>(new platooning::speed);
		outmsg->speed = 0;
		pub_velocity_.publish(outmsg);

	}

	data_src_flags = 0;

	detect_dead_datasource_timer.expires_from_now(SOURCECHECK_FREQ);
	detect_dead_datasource_timer.async_wait(boost::bind(&LongitudinalProcessing::check_dead_datasrc, this,
	                                                    boost::asio::placeholders::error));
}

float LongitudinalProcessing::PDController::calulate_velocity(const Distance &current_position,
                                                              const Velocity &current_velocity) {
	return kp_ * (target_relative_position_ - current_position) - kd_ * current_velocity;
}

LongitudinalProcessing::PDController::PDController() = default;

void LongitudinalProcessing::PDController::set_target_position(const Distance &target_postion) {
	target_relative_position_ = target_postion;
}

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::LongitudinalProcessing, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
