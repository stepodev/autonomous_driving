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

	NODELET_INFO( std::string("[" + name_ + "] init done").c_str());

}

/*****************************************************************************
** Handlers
*****************************************************************************/

void LongitudinalControl::hndl_distance_from_sensor(const platooning::distance &msg) {

	NODELET_INFO( std::string( "[" + name_ + "] recv distance from sensor.").c_str());

	//check if we received new data
	if( msg.distance != current_distance_ ) {
		current_distance_ = msg.distance;
		calculate_acceleration();
	}
}

void LongitudinalControl::hndl_target_distance(const platooning::targetDistance &msg) {

	NODELET_INFO( std::string( "[" + name_ + "] recv targetDistance.").c_str());


	if( msg.distance != target_distance_ ) {
		target_distance_ = msg.distance;
		calculate_acceleration();
	}
};

void LongitudinalControl::hndl_current_speed(const platooning::speed &msg) {

	NODELET_INFO( std::string( "[" + name_ + "] recv speed.").c_str());


	if( msg.speed != current_speed_ ) {
		current_speed_ = msg.speed;
		calculate_acceleration();
	}
}

void LongitudinalControl::hndl_targetSpeed(const platooning::targetSpeed &msg) {

	NODELET_INFO( std::string( "[" + name_ + "] recv targetSpeed.").c_str());


	if( msg.target_speed != target_speed_ ) {
		target_speed_ = msg.target_speed;
		calculate_acceleration();
	}
}
void LongitudinalControl::calculate_acceleration() {

	//decel if we exceed target speeds
	if( current_speed_ > target_speed_ * 1.05 ) {
		calc_mutex_.lock();

		auto outmsg = boost::shared_ptr<platooning::acceleration>( new platooning::acceleration);
		outmsg->accelleration = -1;

		pub_acceleration_.publish(outmsg);
		calc_mutex_.unlock();
		return;
	}

	//decel if we exceed target distance
	if( current_distance_ < target_distance_ * 0.95 ) {
		calc_mutex_.lock();

		auto outmsg = boost::shared_ptr<platooning::acceleration>( new platooning::acceleration);
		outmsg->accelleration = -1;

		pub_acceleration_.publish(outmsg);
		calc_mutex_.unlock();
		return;
	}

	//do nothing if target speed is current speed
	if( abs( current_speed_ - target_speed_ ) <= target_speed_ * 1.05) {
		calc_mutex_.lock();

		auto outmsg = boost::shared_ptr<platooning::acceleration>( new platooning::acceleration);
		outmsg->accelleration = 0;

		pub_acceleration_.publish(outmsg);
		calc_mutex_.unlock();
		return;
	}

	//do nothing if target distance is current distance and we are in acceptable speed range
	if( current_speed_ < target_speed_ *  1.05
		&& abs( current_distance_ - target_distance_ ) < target_distance_ * 1.05) {
		calc_mutex_.lock();

		auto outmsg = boost::shared_ptr<platooning::acceleration>( new platooning::acceleration);
		outmsg->accelleration = 0;

		pub_acceleration_.publish(outmsg);
		calc_mutex_.unlock();
		return;
	}

	//accelerate if we need to keep up
	if( current_speed_ < target_speed_
		&& current_distance_ < target_distance_) {
		calc_mutex_.lock();

		auto outmsg = boost::shared_ptr<platooning::acceleration>( new platooning::acceleration);
		outmsg->accelleration = 1;

		pub_acceleration_.publish(outmsg);
		calc_mutex_.unlock();
		return;
	}

	NODELET_ERROR( std::string( "[" + name_ + "][calculate_acceleration] really shouldnt be here.\n"
	             + "currentspeed:" + std::to_string(current_speed_) + " target speed:" + std::to_string(target_speed_)
	             + "\ncurrentdistance:" + std::to_string(current_distance_) + " target distance:" + std::to_string(target_distance_)
	             ).c_str());

}

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::LongitudinalControl, nodelet::Nodelet);
// %EndTag(FULLTEXT)%