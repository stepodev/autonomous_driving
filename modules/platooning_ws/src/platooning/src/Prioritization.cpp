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
#include <platooning/Platooning.hpp>
#include "platooning/Prioritization.hpp"

namespace platooning {


/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

Prioritization::Prioritization() {};

/*****************************************************************************
** Destructors
*****************************************************************************/
Prioritization::~Prioritization() {};


/*****************************************************************************
** Class Variables
*****************************************************************************/


/*****************************************************************************
** Initializers
*****************************************************************************/

/**
* Set-up necessary publishers/subscribers
* @return true, if successful
*/
void Prioritization::onInit() {
	thread_pool_.create_thread( [this] {

		//Services
		ros::ServiceClient srv_client_ = nh_.serviceClient<platooning::getVehicleId>(platooning_services::VEHICLE_ID);

		ros::Duration sec;
		sec.sec = 2;
		if( srv_client_.waitForExistence(ros::Duration(sec))) {

			platooning::getVehicleId::Request req;
			platooning::getVehicleId::Response res;

			if( srv_client_.call(req, res)) {
				this->vehicle_id_ = res.vehicle_id;
			}
		} else {
			NODELET_INFO("[%s] failed getting vehicle_id from service. defaulting to 0", name_.c_str());
			vehicle_id_ = 0;
		}

	});

	sub_remotecontrolToggle_ = nh_.subscribe(topics::TOGGLE_REMOTECONTROL, 1,
	                                        &Prioritization::hndl_remotecontrolToggle, this);
	sub_remotecontrolInput_ = nh_.subscribe(topics::INPUT_REMOTECONTROL, 1,
	                                       &Prioritization::hndl_remotecontrolInput, this);
	sub_platooningToggle_ = nh_.subscribe(topics::TOGGLE_PLATOONING, 1,
	                                     &Prioritization::hndl_platooningToggle, this);
	sub_platooningState_ = nh_.subscribe(topics::PLATOONINGSTATE, 1,
	                                    &Prioritization::hndl_platooningState, this);
	sub_sensor_velocity_ = nh_.subscribe(topics::SENSOR_VELOCITY, 1,
	                                     &Prioritization::hndl_current_speed, this);
	sub_calc_velocity_ = nh_.subscribe(topics::CALCULATED_VELOCITY, 1,
	                                     &Prioritization::hndl_calc_velocity, this);

	pub_targetAngle_ = nh_.advertise<platooning::targetAngle>(topics::TARGET_ANGLE, 1);
	pub_targetSpeed_ = nh_.advertise<platooning::targetSpeed>(topics::TARGET_SPEED, 1);
	pub_targetDistance_ = nh_.advertise<platooning::targetDistance>(topics::TARGET_DISTANCE, 1);
	pub_vehicleControl_ = nh_.advertise<platooning::vehicleControl>(topics::VEHICLE_CONTROL, 1);

	mode_ = PrioritizationMode::NONE;

	vehicle_id_ = 0;
	target_angle_ = 0;
	target_distance_ = 0;
	target_speed_ = 0;
	current_speed_ = 0;

	NODELET_INFO("[%s] init done", name_.c_str());
}

/*****************************************************************************
** Handlers
*****************************************************************************/

/**
 * @brief Toggle mode in and out of remotecontrol input.
 *
 * Toggle any state into REMOTECONTROL state, moves state to NONE only iff current
 * state is REMOTECONTROL
 *
 * @param msg remotecontrolToggle message
 */
void Prioritization::hndl_remotecontrolToggle(const remotecontrolToggle &msg) {

	if( msg.vehicle_id != vehicle_id_) {
		return;
	}

	if (mode_ != PrioritizationMode::REMOTECONTROL && msg.enable_remotecontrol) {
		NODELET_INFO("[%s] enabled remotecontrol", name_.c_str());
		mode_ = PrioritizationMode::REMOTECONTROL;
	}

	if( mode_ == PrioritizationMode::REMOTECONTROL && !msg.enable_remotecontrol) {
		NODELET_INFO("[%s] disabled remotecontrol", name_.c_str());
		mode_ = PrioritizationMode::NONE;
	}

}

/**
 * @brief Handles control input from controller PC
 *
 * If we receive an emergencystop we immediately send targetspeed=0. Steering angle and Speed only get sent
 * if they deviate from previous values.
 */
void Prioritization::hndl_remotecontrolInput(const remotecontrolInput &msg) {

	if( msg.vehicle_id != vehicle_id_) {
		return;
	}

	if (mode_ != PrioritizationMode::REMOTECONTROL) {
		NODELET_ERROR("[%s] received remotecontrol input while in mode %s", name_.c_str(), to_string(mode_).c_str());
		return;
	}

	NODELET_INFO("[%s] received remotecontrol input", name_.c_str());

	if (mode_ == PrioritizationMode::REMOTECONTROL) {

		//always send emergencystop
		if( msg.emergency_stop ) {
			target_speed_ = 0;

			auto outmsg = boost::shared_ptr<platooning::vehicleControl>(new platooning::vehicleControl);
			outmsg->steering_angle = target_angle_;
			outmsg->velocity = target_speed_;

			pub_vehicleControl_.publish(outmsg);
			return;
		}

		if( msg.remote_speed != target_speed_ || msg.remote_angle != target_angle_) {

			target_angle_ = msg.remote_angle;
			target_speed_ = msg.remote_speed;

			auto outmsg = boost::shared_ptr<platooning::vehicleControl>(new platooning::vehicleControl);
			outmsg->steering_angle = target_angle_;
			outmsg->velocity = target_speed_;

			pub_vehicleControl_.publish(outmsg);
		}
	}
}

/**
 * @brief Toggle any state into PLATOONING state, moves state to NONE only iff current
 * state is PLATOONING
 * @param msg
 */
void Prioritization::hndl_platooningToggle(const platooningToggle &msg) {

	if( msg.vehicle_id != vehicle_id_) {
		return;
	}

	NODELET_INFO("[%s] received platooning toggle", name_.c_str());

	if (mode_ != PrioritizationMode::PLATOONING && msg.enable_platooning) {
		mode_ = PrioritizationMode::PLATOONING;
	}

	if( mode_ == PrioritizationMode::PLATOONING && !msg.enable_platooning) {
		mode_ = PrioritizationMode::NONE;
	}
}

/**
 * @brief Receives platooning state from platooning nodelet. Iff platooningstate is RUNNING forwards target speed
 * and distance
 * @param msg platooningState message
 */
void Prioritization::hndl_platooningState(const platooningState &msg) {

	platooning_state_ = msg;

	if (mode_ != PrioritizationMode::PLATOONING) {
		NODELET_ERROR("[%s] received platooningstate input while in mode %s", name_.c_str(), to_string(mode_).c_str());
		return;
	}

	if (mode_ == PrioritizationMode::PLATOONING && msg.platooning_state == "RUNNING") {

		//reset angle to zero. will be set by lateralcontrol if ever
		target_angle_ = 0;

		if( msg.ps != target_speed_) {
			target_speed_ = msg.ps;
			auto outmsg = boost::shared_ptr<platooning::targetSpeed>( new targetSpeed );
			outmsg->target_speed = target_speed_;
			pub_targetSpeed_.publish(outmsg);
		}

		if( target_distance_ != msg.ipd ) {
			target_distance_ = msg.ipd;

			auto outmsg = boost::shared_ptr<platooning::targetDistance>(new targetDistance);
			outmsg->distance = target_distance_;
			pub_targetDistance_.publish(outmsg);
		}
	}

}

/**
 * @brief saves received current speed from sensor
 * @param msg speed message
 */
void Prioritization::hndl_current_speed(const speed & msg) {

	current_speed_ = msg.speed;

}

/**
 * @brief handles calculated velocity from longitudinal processing and forwards it to vehicleControl topic iff
 * state is PLATOONING. FV can exceed targetspeed to catch up.
 * @param msg message with calculated velocity
 */
void Prioritization::hndl_calc_velocity(const speed &msg) {

	if (mode_ != PrioritizationMode::PLATOONING ) {
		return;
	}

	auto outmsg = boost::shared_ptr<vehicleControl>(new vehicleControl);

	if (platooning_state_.i_am_LV && platooning_state_.platooning_state == "RUNNING") {
		outmsg->velocity = std::max(LOWEST_SPEED, std::min(target_speed_, std::min( msg.speed, HIGHEST_SPEED)));
	}

	/**< followers can underceed (new word) min speed to get away from stupid leader
	and exceed max speed to catch up */
	if (platooning_state_.i_am_FV) {
		outmsg->velocity = std::max(LOWEST_SPEED * 1.2f, std::min(msg.speed, std::min(target_speed_ * 1.2f, HIGHEST_SPEED )));
	};

	outmsg->steering_angle = target_angle_;

	pub_vehicleControl_.publish(outmsg);

}

} // namespace platooning


PLUGINLIB_EXPORT_CLASS(platooning::Prioritization, nodelet::Nodelet);

