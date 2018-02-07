#include "Platooning.hpp"

namespace platooning
{
	Platooning::Platooning() {

	}
	Platooning::~Platooning() {}

  /**
   * TODO:
   * - handle timeouts
   *       start boost::asio::deadline_timer for each vehicle and restart it when package arrives?
   *       what if it doesnt arrive in time?
   *       race conditions?
   * - all the rest.
   */

	void Platooning::onInit() {

    //TODO:

		// subscribers nh_.subscribe("Topic", Rate, &Callback, this)
		sub_fv_leave = nh_.subscribe(topics::IN_FV_LEAVE, 100, &Platooning::hndl_fv_leave, this);
		sub_lv_accept = nh_.subscribe(topics::IN_LV_ACCEPT, 100, &Platooning::hndl_lv_accept, this);
		sub_lv_reject = nh_.subscribe(topics::IN_LV_REJECT, 100, &Platooning::hndl_lv_reject, this);
		sub_fv_request = nh_.subscribe(topics::IN_FV_REQUEST, 100, &Platooning::hndl_fv_request, this);
		sub_lv_broadcast = nh_.subscribe(topics::IN_LV_BROADCAST, 100, &Platooning::hndl_lv_broadcast, this);
		sub_fv_heartbeat = nh_.subscribe(topics::IN_FV_HEARTBEAT, 100, &Platooning::hndl_fv_heartbeat, this);
		sub_platooning_toggle = nh_.subscribe(topics::TOGGLE_PLATOONING, 100, &Platooning::hndl_platooning_toggle, this);

		//publishers nh_.advertise<MsgType>("Topic")
		pub_fv_leave = nh_.advertise<platooning::fv_leave>(topics::OUT_FV_LEAVE, 100);
		pub_lv_accept = nh_.advertise<platooning::lv_accept>(topics::OUT_LV_ACCEPT, 100);
		pub_lv_reject = nh_.advertise<platooning::lv_reject>(topics::OUT_LV_REJECT, 100);
		pub_fv_request = nh_.advertise<platooning::fv_request>(topics::OUT_FV_REQUEST, 100);
		pub_lv_broadcast = nh_.advertise<platooning::lv_broadcast>(topics::OUT_LV_BROADCAST, 1);
		pub_fv_heartbeat = nh_.advertise<platooning::fv_heartbeat>(topics::OUT_FV_HEARTBEAT, 100);
		pub_platooning_state = nh_.advertise<platooning::platooningState>(topics::PLATOONINGSTATE, 100);

		//take parameters that set role and vehicle_id if exist
		if( nh_.hasParam("set_role_leader")) {
			nh_.getParam("set_role_leader", is_leader_vehicle_ );
			is_follower_vehicle_ = !is_leader_vehicle_;
		}

		if( nh_.hasParam("vehicle_id")) {
      int paramvehicleid = 0;
      nh_.getParam("vehicle_id", paramvehicleid);

      if( paramvehicleid < 1 ) {
        NODELET_ERROR( std::string( "[" + name_ + "] invalid vehicle id parameter").c_str() );
      } else {
        vehicle_id_ = (uint32_t)paramvehicleid;
      }
		}

	};

	void Platooning::hndl_fv_leave(const platooning::fv_leave& msg) {
		NODELET_DEBUG("handling fv_leave");

		if( platooning_state_ == IDLE ) {
			return;
		}

		if( is_leader_vehicle_ ) {

		}

		if( is_follower_vehicle_) {

		}

	}

	void Platooning::hndl_lv_accept(const platooning::lv_accept& msg) {
		NODELET_DEBUG("handling lv_accept");

		if( platooning_state_ == IDLE ) {
			return;
		}

		if( is_leader_vehicle_ ) {

		}

		if( is_follower_vehicle_) {

		}
	}

	void Platooning::hndl_lv_reject(const platooning::lv_reject& msg) {
		NODELET_DEBUG("handling lv_reject");

		if( platooning_state_ == IDLE ) {
			return;
		}

		if( is_leader_vehicle_ ) {

		}

		if( is_follower_vehicle_) {

		}
	}

	void Platooning::hndl_fv_request(const platooning::fv_request& msg) {
		NODELET_DEBUG("handling fv_request");

		if( platooning_state_ == IDLE ) {
			return;
		}

		if( is_leader_vehicle_ ) {

		}

		if( is_follower_vehicle_) {

		}
	}

	void Platooning::hndl_lv_broadcast(const platooning::lv_broadcast& msg) {
		NODELET_DEBUG("handling lv_broadcast");

		if( platooning_state_ == IDLE ) {
			return;
		}

		if( is_leader_vehicle_ ) {

		}

		if( is_follower_vehicle_) {

		}
	}

	void Platooning::hndl_fv_heartbeat(const platooning::fv_heartbeat& msg) {
		NODELET_DEBUG("handling fv_heartbeat");

		if( platooning_state_ == IDLE ) {
			return;
		}

		if( is_leader_vehicle_ ) {

		}

		if( is_follower_vehicle_) {

		}
	}

	void Platooning::hndl_platooning_toggle(const platooning::platooningToggle& msg) {
		NODELET_DEBUG("handling platooning_toggle");

		if( !msg.enable_platooning ) {
			platooning_state_ = IDLE;
			return;
		}

		if( msg.enable_platooning && platooning_state_ == IDLE ) {
			platooning_state_ = CREATING;
		}

		if( msg.enable_platooning && platooning_state_ != IDLE) {
			NODELET_ERROR( std::string( "[" +  name_ + "] received platooning start while state was "
																	+ PlatooningStateStrings[platooning_state_]).c_str());
		}

		if( is_leader_vehicle_ ) {
      platoon_data_.src_vehicle = vehicle_id_;
      platoon_data_.platoon_id = vehicle_id_;
      platoon_data_.inner_platoon_distance = msg.inner_platoon_distance;
      platoon_data_.platoon_speed = msg.platoon_speed;
			//waiting for FV_REQUEST
		}

		if( is_follower_vehicle_) {
			auto msg_to_send = boost::shared_ptr<fv_request>( new fv_request() );
			msg_to_send->src_vehicle = vehicle_id_;

			pub_fv_request.publish(msg_to_send);
		}
	}
}

PLUGINLIB_EXPORT_CLASS(platooning::Platooning, nodelet::Nodelet);

