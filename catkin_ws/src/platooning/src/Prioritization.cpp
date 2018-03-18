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
		ros::ServiceClient srv_client_ = srv_client_ = nh_.serviceClient<platooning::getVehicleId>(platooning_services::VEHICLE_ID);

		ros::Duration sec;
		sec.sec = 20;
		if( srv_client_.waitForExistence(ros::Duration(sec))) {

			platooning::getVehicleId::Request req;
			platooning::getVehicleId::Response res;

			if( srv_client_.call(req, res)) {
				this->vehicle_id_ = res.vehicle_id;
			}
		}

	});

	sub_remotecontrolToggle = nh_.subscribe(topics::TOGGLE_REMOTECONTROL, 1,
	                                        &Prioritization::hndl_remotecontrolToggle, this);
	sub_remotecontrolInput = nh_.subscribe(topics::REMOTECONTROL_INPUT, 1,
	                                       &Prioritization::hndl_remotecontrolInput, this);
	sub_platooningToggle = nh_.subscribe(topics::TOGGLE_PLATOONING, 1,
	                                     &Prioritization::hndl_platooningToggle, this);
	sub_platooningState = nh_.subscribe(topics::PLATOONINGSTATE, 1,
	                                    &Prioritization::hndl_platooningState, this);

	pub_targetAngle = nh_.advertise<platooning::targetAngle>(topics::TARGET_ANGLE, 1);
	pub_targetSpeed = nh_.advertise<platooning::targetSpeed>(topics::TARGET_SPEED, 1);
	pub_targetDistance = nh_.advertise<platooning::targetDistance>(topics::TARGET_DISTANCE, 1);

	state_ = PrioritizationState::NONE;

	target_angle_ = 0;
	target_distance_ = 0;
	target_speed_ = 0;
}

/*****************************************************************************
** Handlers
*****************************************************************************/

void Prioritization::hndl_remotecontrolToggle(platooning::remotecontrolToggle msg) {

	if (state_ != PrioritizationState::REMOTECONTROL && msg.enable_remotecontrol) {
		state_ = PrioritizationState::REMOTECONTROL;
	}

}

void Prioritization::hndl_remotecontrolInput(platooning::remotecontrolInput msg) {
	if (state_ != PrioritizationState::REMOTECONTROL) {
		NODELET_ERROR("[%s] received remotecontrol input while in mode %s", name_.c_str(), PrioritizationStateString[state_].c_str());
	}

	if (state_ == PrioritizationState::REMOTECONTROL) {

		//always send emergencystop
		if( msg.emergency_stop ) {
			target_speed_ = 0;
			auto outmsg = boost::shared_ptr<platooning::targetSpeed>(new targetSpeed);
			outmsg->target_speed = target_speed_;
			pub_targetSpeed.publish(outmsg);
		}

		if( msg.remote_speed != target_speed_ ) {
			target_speed_ = msg.remote_speed;
			auto outmsg = boost::shared_ptr<platooning::targetSpeed>(new targetSpeed);
			outmsg->target_speed = target_speed_;
			pub_targetSpeed.publish(outmsg);
		}

		if( msg.remote_angle != target_angle_ ) {
			target_angle_ = msg.remote_angle;
			auto outmsg = boost::shared_ptr<platooning::targetAngle>(new targetAngle);
			outmsg->steering_angle = target_angle_;
			pub_targetAngle.publish(outmsg);
		}
	}
}

void Prioritization::hndl_platooningToggle(platooning::platooningToggle msg) {

	if (state_ != PrioritizationState::PLATOONING && msg.enable_platooning) {
		state_ = PrioritizationState::PLATOONING;
		//needs to be turned off so we dont start driving on toggle

		//acceleration_ = msg.platoon_speed;
		//target_distance_ = msg.inner_platoon_distance;
	}
}

void Prioritization::hndl_platooningState(platooning::platooningState msg) {

	if (state_ != PrioritizationState::PLATOONING) {
		NODELET_ERROR("[%s] received platooningstate input while in mode %s", name_.c_str(), PrioritizationStateString[state_].c_str());
	}

	if (state_ == PrioritizationState::PLATOONING && msg.platooning_state == "RUNNING") {

		//drive faster as FV to catch up
		if( msg.i_am_FV && msg.ps * 1.3 != target_speed_ ) {
			target_speed_ = msg.ps * 1.3;
			auto outmsg = boost::shared_ptr<platooning::targetSpeed>( new targetSpeed );
			outmsg->target_speed = target_speed_;
			pub_targetSpeed.publish(outmsg);
		} else if( msg.i_am_LV && msg.ps != target_speed_) {
			target_speed_ = msg.ps;
			auto outmsg = boost::shared_ptr<platooning::targetSpeed>( new targetSpeed );
			outmsg->target_speed = target_speed_;
			pub_targetSpeed.publish(outmsg);
		}

		if( target_distance_ != msg.ipd ) {
			target_distance_ = msg.ipd;
			auto outmsg = boost::shared_ptr<platooning::targetDistance>(new targetDistance);
			outmsg->distance = target_distance_;
			pub_targetDistance.publish(outmsg);
		}
	}

};

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Prioritization, nodelet::Nodelet);

