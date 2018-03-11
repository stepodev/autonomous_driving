#include <boost/thread/thread.hpp>
#include "platooning/UserInterface.hpp"

namespace platooning {
UserInterface::UserInterface() {};
UserInterface::~UserInterface() {
	spin_report_ = false;
	report_spinner_thread_.interrupt();
};

void UserInterface::onInit() {

	ui_msg_ = boost::shared_ptr<platooning::userInterface>(new platooning::userInterface);

	pub_userinterface_ = nh_.advertise<userInterface>(topics::USERINTERFACE, 1, true);

	sub_in_lv_broadcast_ = nh_.subscribe(topics::IN_LV_BROADCAST, 1,
	                                     &UserInterface::hndl_in_lv_broadcast, this);
	sub_in_lv_accept_ = nh_.subscribe(topics::IN_LV_ACCEPT, 1,
	                                  &UserInterface::hndl_in_lv_accept, this);
	sub_in_lv_reject_ = nh_.subscribe(topics::IN_LV_REJECT, 1,
	                                  &UserInterface::hndl_in_lv_reject, this);
	sub_out_lv_broadcast_ = nh_.subscribe(topics::OUT_LV_BROADCAST, 1,
	                                      &UserInterface::hndl_out_lv_broadcast, this);
	sub_out_lv_accept_ = nh_.subscribe(topics::OUT_LV_ACCEPT, 1,
	                                   &UserInterface::hndl_out_lv_accept, this);
	sub_out_lv_reject_ = nh_.subscribe(topics::OUT_LV_REJECT, 1,
	                                   &UserInterface::hndl_out_lv_reject, this);
	sub_in_fv_request = nh_.subscribe(topics::IN_FV_REQUEST, 1,
	                                  &UserInterface::hndl_in_fv_request, this);
	sub_in_fv_heartbeat = nh_.subscribe(topics::IN_FV_HEARTBEAT, 1,
	                                    &UserInterface::hndl_in_fv_heartbeat, this);
	sub_in_fv_leave = nh_.subscribe(topics::IN_FV_LEAVE, 1,
	                                &UserInterface::hndl_in_fv_leave, this);
	sub_out_fv_request = nh_.subscribe(topics::OUT_FV_REQUEST, 1,
	                                   &UserInterface::hndl_out_fv_request, this);
	sub_out_fv_heartbeat = nh_.subscribe(topics::OUT_FV_HEARTBEAT, 1,
	                                     &UserInterface::hndl_out_fv_heartbeat, this);
	sub_out_fv_leave = nh_.subscribe(topics::OUT_FV_LEAVE, 1,
	                                 &UserInterface::hndl_out_fv_leave, this);
	sub_toggle_remotecontrol = nh_.subscribe(topics::TOGGLE_REMOTECONTROL, 1,
	                                         &UserInterface::hndl_remotecontrol_toggle, this);
	sub_toggle_platooning = nh_.subscribe(topics::TOGGLE_PLATOONING, 1,
	                                      &UserInterface::hndl_platooning_toggle, this);
	sub_remotecontrol_input = nh_.subscribe(topics::REMOTECONTROL_INPUT, 1,
	                                        &UserInterface::hndl_remotecontrol_input, this);
	sub_speed = nh_.subscribe(topics::CURRENT_SPEED, 1,
	                          &UserInterface::hndl_current_speed, this);
	sub_target_speed = nh_.subscribe(topics::TARGET_SPEED, 1,
	                                 &UserInterface::hndl_target_speed, this);
	sub_distance_to_obj = nh_.subscribe(topics::SENSOR_DISTANCE_TO_OBJ, 1,
	                                    &UserInterface::hndl_sensor_dist_to_obj, this);
	sub_target_dist = nh_.subscribe(topics::TARGET_DISTANCE, 1,
	                                    &UserInterface::hndl_target_dist, this);
	sub_steeringangle = nh_.subscribe(topics::STEERING_ANGLE, 1,
	                                  &UserInterface::hndl_steering_angle, this);
	sub_acceleration = nh_.subscribe(topics::ACCELERATION, 1,
	                                 &UserInterface::hndl_acceleration, this);
	sub_platooning_state = nh_.subscribe(topics::PLATOONINGSTATE, 1,
	                                     &UserInterface::hndl_platooningState, this);

	report_spinner_thread_ = boost::thread([this] {
		try {
			while (this->spin_report_) {
				if (pub_userinterface_.getNumSubscribers() > 0) {
					this->pub_userinterface_.publish(this->ui_msg_);
				}
				boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
			}
		} catch (std::exception &ex) {
			std::cerr << "[" + this->name_ + "] reportthread crash with " << ex.what() << std::endl;
		}
	});
};

void UserInterface::hndl_in_lv_broadcast(const lv_broadcast &msg) {
	ui_msg_->inner_platoon_distance = msg.ipd;
	ui_msg_->platoon_speed = msg.ps;
	std::copy(msg.followers.begin(), msg.followers.end(), std::back_inserter(ui_msg_->platoon_members));
	ui_msg_->platoon_size = msg.followers.size();
}

void UserInterface::hndl_in_lv_accept(const lv_accept &msg) {

}

void UserInterface::hndl_in_lv_reject(const lv_reject &msg) {

}

void UserInterface::hndl_in_fv_request(const fv_request &msg) {

}

void UserInterface::hndl_in_fv_leave(const fv_leave &msg) {

}

void UserInterface::hndl_in_fv_heartbeat(const fv_heartbeat &msg) {

}

void UserInterface::hndl_out_lv_broadcast(const lv_broadcast &msg) {

}

void UserInterface::hndl_out_lv_accept(const lv_accept &msg) {

}

void UserInterface::hndl_out_lv_reject(const lv_reject &msg) {

}

void UserInterface::hndl_out_fv_request(const fv_request &msg) {

}

void UserInterface::hndl_out_fv_leave(const fv_leave &msg) {

}

void UserInterface::hndl_out_fv_heartbeat(const fv_heartbeat &msg) {

}

void UserInterface::hndl_remotecontrol_toggle(const remotecontrolToggle &msg) {
	ui_msg_->enable_remotecontrol = msg.enable_remotecontrol;
}

void UserInterface::hndl_platooning_toggle(const platooningToggle &msg) {

}

void UserInterface::hndl_remotecontrol_input(const remotecontrolInput &msg) {

}

void UserInterface::hndl_sensor_dist_to_obj(const distance &msg) {
	ui_msg_->actual_distance = msg.distance;
}

void UserInterface::hndl_current_speed(const speed &msg) {
	ui_msg_->speed = msg.speed;
}

void UserInterface::hndl_steering_angle(const steeringAngle &msg) {

}

void UserInterface::hndl_acceleration(const acceleration &msg) {

}

void UserInterface::hndl_target_dist(const targetDistance &msg) {

}
void UserInterface::hndl_target_speed(const targetSpeed &msg) {

}

void UserInterface::hndl_platooningState(const platooningState &msg) {
	ui_msg_->src_vehicle = msg.vehicle_id;
	ui_msg_->platooning_state = msg.platooning_state;
	ui_msg_->following_vehicle = msg.i_am_FV;
	ui_msg_->leading_vehicle = msg.i_am_LV;
}

}

PLUGINLIB_EXPORT_CLASS(platooning::UserInterface, nodelet::Nodelet);
