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
#include "platooning/LongitudinalProcessing.hpp"

namespace platooning {


/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

LongitudinalProcessing::LongitudinalProcessing()
	: io_worker_(io_service_), detect_dead_datasource_timer(io_service_) {

	thread_pool_.create_thread([this] { io_service_.run(); });

};

/*****************************************************************************
** Destructors
*****************************************************************************/

LongitudinalProcessing::~LongitudinalProcessing() {};


/*****************************************************************************
** Initializers
*****************************************************************************/

/**
* Set-up necessary publishers/subscribers
*/
void LongitudinalProcessing::onInit() {

	sub_current_speed_ = nh_.subscribe(topics::SENSOR_VELOCITY, 1,
	                                   &LongitudinalProcessing::hndl_current_velocity, this);

	sub_target_speed_ = nh_.subscribe(topics::TARGET_SPEED, 1,
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

void LongitudinalProcessing::hndl_distance_from_sensor(const platooning::distance &msg) {
	//NODELET_INFO(  "[%s]  recv distance from sensor.", name_.c_str());

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

void LongitudinalProcessing::hndl_target_distance(const platooning::targetDistance &msg) {
	try {
		if (msg.distance != -pd_controller_.get_target_position()) {

			boost::recursive_mutex::scoped_lock l(calc_mutex_);

			if (msg.distance < 0.5) {
				NODELET_ERROR("[%s] target distance shorter than 0.5. Setting to 1.", name_.c_str());
				pd_controller_.set_target_position(-1);

			} else {
				pd_controller_.set_target_position(-msg.distance);
			}
		}
	} catch (std::exception &ex) {
		NODELET_ERROR("[%s] hndl_target_distance crash with %s", name_.c_str(), ex.what());
	}
};

void LongitudinalProcessing::hndl_current_velocity(const platooning::speed &msg) {

	data_src_flags |= VELOCITY_DATA_CHECK;

	//NODELET_INFO( "[%s] recv speed.", name_.c_str());
	try {
		if (msg.speed != current_velocity_) {

			boost::recursive_mutex::scoped_lock l(calc_mutex_);

			current_velocity_ = msg.speed;
		}
	} catch (std::exception &ex) {
		NODELET_ERROR("[%s] hndl_current_velocity crash with %s", name_.c_str(), ex.what());
	}
}

void LongitudinalProcessing::hndl_targetSpeed(const platooning::targetSpeed &msg) {
	//NODELET_INFO( "[%s] recv targetSpeed.", name_.c_str());
	try {
		if (msg.target_speed != target_velocity_) {

			boost::recursive_mutex::scoped_lock l(calc_mutex_);

			target_velocity_ = msg.target_speed;
		}
	} catch (std::exception &ex) {
		NODELET_ERROR("[%s] hndl_targetSpeed crash with %s", name_.c_str(), ex.what());
	}
}

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

	float calculated_velocity = pd_controller_.calulate_velocity(-current_distance_, current_velocity_);

	/*
	//every 40th time. remove! bug happens on one of those variables. invalid memory
	if (ix++ % 20 == 0) {
		NODELET_INFO(
			"ms: %i time_step: %f range_diff: %f relative_vel: %f spring_vel %f current_vel: %f calc_vel: %f\ndist:%f target:%f prev_dist %f",
			(int) (current_distance_timestamp_ - previous_distance_timestamp_).total_milliseconds(),
			time_step,
			range_diff,
			relative_velocity,
			spring_velocity,
			current_velocity_,
			calculated_velocity,
			current_distance_,
			-pd_controller_.get_target_position(),
		previous_distance_);
	}
*/
	outmsg->speed = calculated_velocity;

	pub_velocity_.publish(outmsg);

}
void LongitudinalProcessing::check_dead_datasrc(const boost::system::error_code &e) {

	if (boost::asio::error::operation_aborted == e) {
		NODELET_ERROR("[%s] check_dead_datasrc timer cancelled", name_.c_str());
		return;
	}

	if ((data_src_flags & RANGE_DATA_CHECK) != RANGE_DATA_CHECK) {
		NODELET_ERROR("[%s] RANGE_DATA_CHECK false. range data not received for %i",
		              name_.c_str(), (int) SOURCECHECK_FREQ.total_milliseconds());

		auto outmsg = boost::shared_ptr<platooning::speed>(new platooning::speed);
		outmsg->speed = 0;
		pub_velocity_.publish(outmsg);

	}

	if ((data_src_flags & VELOCITY_DATA_CHECK) != VELOCITY_DATA_CHECK) {
		NODELET_ERROR("[%s] VELOCITY_DATA_CHECK false. velocity data not received for %i",
		              name_.c_str(), (int) SOURCECHECK_FREQ.total_milliseconds());

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
LongitudinalProcessing::PDController::PDController() {

}

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::LongitudinalProcessing, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
