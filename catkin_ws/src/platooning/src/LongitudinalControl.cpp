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

	sub_current_speed_ = nh_.subscribe(topics::CURRENTSPEED, 1,
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
}

/*****************************************************************************
** Handlers
*****************************************************************************/

void LongitudinalControl::hndl_distance_from_sensor(const platooning::distance &msg) {

	//check if we received new data
	if( msg.distance != current_distance_ ) {
		current_distance_ = msg.distance;
		calculate_acceleration();
	}
}

void LongitudinalControl::hndl_target_distance(const platooning::targetDistance &msg) {
	if( msg.distance != target_distance_ ) {
		target_distance_ = msg.distance;
		calculate_acceleration();
	}
};

void LongitudinalControl::hndl_current_speed(const platooning::speed &msg) {
	if( msg.speed != current_speed_ ) {
		current_speed_ = msg.speed;
		calculate_acceleration();
	}
}

void LongitudinalControl::hndl_targetSpeed(const platooning::targetSpeed &msg) {
	if( msg.target_speed != target_speed_ ) {
		target_speed_ = msg.target_speed;
		calculate_acceleration();
	}
}
void LongitudinalControl::calculate_acceleration() {

	calc_mutex_.lock();

	float accel = current_speed_;

	//check if we are too fast based on target distance
	if( target_distance_ - current_distance_ > (target_distance_ * 1.05) ) {
		accel = 1;
	} else if( target_distance_ - current_distance_ < (target_distance_ * 1.05))  {
		accel = -1;
	} else {
		NODELET_WARN( std::string("[" + name_ +"] target distance and current distance equal, yet we are calculating").c_str());
		//is equal, do nothing
		calc_mutex_.unlock();
		return;
	}

	//check if we too fast
	if( target_speed_ - current_speed_ > ( target_speed_ * 1.05 ) ) {
		accel = 1;
	} else if( target_speed_ - current_speed_ < ( target_speed_ * 1.05 ) ) {
		accel = -1;
	} else {
		NODELET_WARN( std::string("[" + name_ +"] target speed and current speed equal, yet we are calculating").c_str());
		//is equal, do nothing
		calc_mutex_.unlock();
		return;
	}

	calc_mutex_.unlock();

	auto outmsg = boost::shared_ptr<platooning::acceleration>( new platooning::acceleration);
	outmsg->accelleration = accel;

	pub_acceleration_.publish(outmsg);
}

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::LongitudinalControl, nodelet::Nodelet);
// %EndTag(FULLTEXT)%