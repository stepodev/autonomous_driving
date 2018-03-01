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
#include "Prioritization.hpp"

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

	sub_remotecontrolToggle = nh_.subscribe(topics::TOGGLE_REMOTECONTROL, 1,
	                                        &Prioritization::hndl_remotecontrolToggle, this);
	sub_remotecontrolInput = nh_.subscribe(topics::REMOTECONTROLINPUT, 1,
	                                       &Prioritization::hndl_remotecontrolInput, this);
	sub_platooningToggle = nh_.subscribe(topics::TOGGLE_PLATOONING, 1,
	                                     &Prioritization::hndl_platooningToggle, this);
	sub_platooningState = nh_.subscribe(topics::PLATOONINGSTATE, 1,
	                                    &Prioritization::hndl_platooningState, this);

	pub_oughtData = nh_.advertise<platooning::oughtData>(topics::OUGHTDATA, 100);

	state_ = PrioritizationState::NONE;

	current_oughtData_.speed = 0;
	current_oughtData_.distance = 0;
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
		NODELET_ERROR(std::string("[" + name_ + "] received remotecontrol input while in mode "
			                          + PrioritizationStateString[state_]).c_str());
	}

	if (state_ == PrioritizationState::REMOTECONTROL) {

		current_oughtData_.speed = msg.remote_speed;
		current_oughtData_.distance = msg.remote_angle;

		if (msg.emergency_stop) {
			current_oughtData_.speed = 0;
		}

		auto outmsg = boost::make_shared<oughtData>(current_oughtData_);
		outmsg->speed = current_oughtData_.speed;
		outmsg->distance = current_oughtData_.distance;

		pub_oughtData.publish(outmsg);
	}
}

void Prioritization::hndl_platooningToggle(platooning::platooningToggle msg) {

	if (state_ != PrioritizationState::PLATOONING && msg.enable_platooning) {
		state_ = PrioritizationState::PLATOONING;
		current_oughtData_.speed = msg.platoon_speed;
		current_oughtData_.distance = msg.inner_platoon_distance;
	}
}

void Prioritization::hndl_platooningState(platooning::platooningState msg) {

	if (state_ != PrioritizationState::PLATOONING) {
		NODELET_ERROR(std::string("[" + name_ + "] received remotecontrol input while in mode "
			                          + PrioritizationStateString[state_]).c_str());
	}

	if (state_ == PrioritizationState::PLATOONING) {
		current_oughtData_.speed = msg.ps;
		current_oughtData_.distance = msg.ipd;

		auto outmsg = boost::make_shared<oughtData>(current_oughtData_);
		outmsg->speed = current_oughtData_.speed;
		outmsg->distance = current_oughtData_.distance;

		pub_oughtData.publish(outmsg);
	}

};

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Prioritization, nodelet::Nodelet);

