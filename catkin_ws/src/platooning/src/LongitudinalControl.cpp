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
#include <boost/thread/mutex.hpp>
#include "platooning/LongitudinalControl.hpp"

namespace platooning {


/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

LongitudinalControl::LongitudinalControl() {};

/*****************************************************************************
** Destructors
*****************************************************************************/

LongitudinalControl::~LongitudinalControl() {};


/*****************************************************************************
** Initializers
*****************************************************************************/

/**
* Set-up necessary publishers/subscribers
*/
void LongitudinalControl::onInit() {

	sub_current_speed_ = nh_.subscribe(topics::CURRENT_SPEED, 1,
	                                   &LongitudinalControl::hndl_current_speed, this);

	sub_target_speed_ = nh_.subscribe(topics::TARGET_SPEED, 1,
	                                  &LongitudinalControl::hndl_targetSpeed, this);

	sub_distance_to_obj_ = nh_.subscribe(topics::SENSOR_DISTANCE_TO_OBJ, 1,
	                                     &LongitudinalControl::hndl_distance_from_sensor, this);

	sub_target_distance_ = nh_.subscribe(topics::TARGET_DISTANCE, 1,
	                                     &LongitudinalControl::hndl_target_distance, this);

	pub_acceleration_ = nh_.advertise<platooning::acceleration>(topics::ACCELERATION, 1);

	current_distance_ = 0;
	current_speed_ = 0;
	target_distance_ = 0;
	target_speed_ = 0;

	NODELET_INFO("[%s] init done", name_.c_str());

}

/*****************************************************************************
** Handlers
*****************************************************************************/

void LongitudinalControl::hndl_distance_from_sensor(const platooning::distance &msg) {
	//NODELET_INFO(  "[%s]  recv distance from sensor.", name_.c_str());

	try {
		//check if we received new data
		if (msg.distance != current_distance_) {
			current_distance_ = msg.distance;
			calculate_acceleration();
		}
	} catch (std::exception &ex) {
		NODELET_ERROR("[%s] hndl_distance_from_sensor crash with %s", name_.c_str(), ex.what());
	}
}

void LongitudinalControl::hndl_target_distance(const platooning::targetDistance &msg) {
	//NODELET_INFO( "[%s] recv targetDistance.", name_.c_str());
	try {
		if (msg.distance != target_distance_) {
			target_distance_ = msg.distance;
			calculate_acceleration();
		}
	} catch (std::exception &ex) {
		NODELET_ERROR("[%s] hndl_target_distance crash with %s", name_.c_str(), ex.what());
	}
};

void LongitudinalControl::hndl_current_speed(const platooning::speed &msg) {

	//NODELET_INFO( "[%s] recv speed.", name_.c_str());
	try {
		if (msg.speed != current_speed_) {
			current_speed_ = msg.speed;
			calculate_acceleration();
		}
	} catch (std::exception &ex) {
		NODELET_ERROR("[%s] hndl_current_speed crash with %s", name_.c_str(), ex.what());
	}
}

void LongitudinalControl::hndl_targetSpeed(const platooning::targetSpeed &msg) {
	//NODELET_INFO( "[%s] recv targetSpeed.", name_.c_str());
	try {
		if (msg.target_speed != target_speed_) {
			target_speed_ = msg.target_speed;
			calculate_acceleration();
		}
	} catch (std::exception &ex) {
		NODELET_ERROR("[%s] hndl_targetSpeed crash with %s", name_.c_str(), ex.what());
	}
}
void LongitudinalControl::calculate_acceleration() {

	NODELET_WARN("[%s] calculate_acceleration \nspeed: %f targetspeed %f\ndistance %f targetdistance %f\ndiff:%f fabs:%f maxdiff:%f bool:%i",
	             name_.c_str(), current_speed_, target_speed_, current_distance_, target_distance_,
	             (current_speed_ - target_speed_),
	             std::fabs(current_speed_ - target_speed_),
	             target_speed_ * 0.05,
	             (std::fabs(current_speed_ - target_speed_) <= target_speed_ * 0.05));

	try {
		//decel if we exceed target speeds
		if (current_speed_ > target_speed_ * 1.05) {
			boost::mutex::scoped_lock l(calc_mutex_);

			NODELET_WARN("[%s] decel dt target speed exceeded \nspeed: %f targetspeed %f\ndistance %f targetdistance %f",
			             name_.c_str(), current_speed_, target_speed_, current_distance_, target_distance_);

			auto outmsg = boost::shared_ptr<platooning::acceleration>(new platooning::acceleration);
			outmsg->accelleration = -1.0;

			pub_acceleration_.publish(outmsg);
			return;
		}

		//decel if we too close
		if (current_distance_ < target_distance_ * 0.95) {
			boost::mutex::scoped_lock l(calc_mutex_);

			NODELET_WARN("[%s] decel dt target distence under \nspeed: %f targetspeed %f\ndistance %f targetdistance %f",
			             name_.c_str(), current_speed_, target_speed_, current_distance_, target_distance_);

			auto outmsg = boost::shared_ptr<platooning::acceleration>(new platooning::acceleration);
			outmsg->accelleration = -1.0;

			pub_acceleration_.publish(outmsg);
			return;
		}

		//do nothing if target speed is current speed
		if(std::fabs(current_speed_ - target_speed_) <= (target_speed_ * 0.05)) {

			boost::mutex::scoped_lock l(calc_mutex_);

			NODELET_WARN("[%s] we at target speed \ndiff:%f fabs:%f maxdiff:%f bool:%i",
			             name_.c_str(),
			             (current_speed_ - target_speed_),
			             std::fabs(current_speed_ - target_speed_),
			             target_speed_ * 0.05,
			             (std::fabs(current_speed_ - target_speed_) <= (target_speed_ * 0.05)));

			auto outmsg = boost::shared_ptr<platooning::acceleration>(new platooning::acceleration);
			outmsg->accelleration = 0.0;

			pub_acceleration_.publish(outmsg);
			return;
		}

		//do nothing if target distance is current distance and we are in acceptable speed range
		if (current_speed_ < target_speed_ * 1.05
			&& std::fabs(current_distance_ - target_distance_) < target_distance_ * 0.05) {
			boost::mutex::scoped_lock l(calc_mutex_);

			NODELET_WARN("[%s] we in distance and speed zone \nspeed: %f targetspeed %f\ndistance %f targetdistance %f",
			             name_.c_str(), current_speed_, target_speed_, current_distance_, target_distance_);

			auto outmsg = boost::shared_ptr<platooning::acceleration>(new platooning::acceleration);
			outmsg->accelleration = 0.0;

			pub_acceleration_.publish(outmsg);

			return;
		}

		//accelerate if we need to keep up
		if (current_speed_ < target_speed_
			&& current_distance_ > target_distance_) {
			boost::mutex::scoped_lock l(calc_mutex_);

			NODELET_WARN("[%s] we too slow \nspeed: %f targetspeed %f\ndistance %f targetdistance %f",
			             name_.c_str(), current_speed_, target_speed_, current_distance_, target_distance_);

			auto outmsg = boost::shared_ptr<platooning::acceleration>(new platooning::acceleration);
			outmsg->accelleration = 1.0;

			pub_acceleration_.publish(outmsg);
			return;
		}

	} catch (std::exception &ex) {
		NODELET_ERROR("[%s] calculate failed with %s\nspeed: %f targetspeed %f\ndistance %f targetdistance %f",
		              name_.c_str(), ex.what(), current_speed_, target_speed_, current_distance_, target_distance_);
	}

	NODELET_ERROR("[%s] calculate_acceleration really shouldnt be here.\n", name_.c_str());
}

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::LongitudinalControl, nodelet::Nodelet);
// %EndTag(FULLTEXT)%