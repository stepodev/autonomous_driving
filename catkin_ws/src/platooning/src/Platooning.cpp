#include <chrono>
#include "platooning/Platooning.hpp"

namespace platooning {
Platooning::Platooning() :
	fv_heartbeat_sender_(io_service_),
	fv_heartbeat_checker_(io_service_),
	lv_broadcast_sender_(io_service_),
	lv_broadcast_checker_(io_service_) {

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

	// subscribers nh_.subscribe("Topic", Rate, &Callback, this)
	sub_platooning_toggle =
		nh_.subscribe(topics::TOGGLE_PLATOONING, 100, &Platooning::hndl_msg_platooning_toggle, this);

	//publishers nh_.advertise<MsgType>("Topic")
	pub_platooning_state = nh_.advertise<platooning::platooningState>(topics::PLATOONINGSTATE, 100);

	//take parameters that set role and vehicle_id if exist
	if (nh_.hasParam("set_role_leader")) {
		bool is_leader;
		nh_.getParam("set_role_leader", is_leader);

		if (is_leader) {
			platoon_role_ = LV;
		} else {
			platoon_role_ = FV;
		}
	}

	if (nh_.hasParam("vehicle_id")) {
		int paramvehicleid = 0;
		nh_.getParam("vehicle_id", paramvehicleid);

		if (paramvehicleid < 1) {
			NODELET_ERROR(std::string("[" + name_ + "] invalid vehicle id parameter").c_str());
		} else {
			vehicle_id_ = (uint32_t) paramvehicleid;
		}
	}

	if (platoon_role_ == LV) {
		sub_fv_request = nh_.subscribe(topics::IN_FV_REQUEST, 100, &Platooning::hndl_msg_fv_request, this);
		sub_fv_heartbeat = nh_.subscribe(topics::IN_FV_HEARTBEAT, 100, &Platooning::hndl_msg_fv_heartbeat, this);
		sub_fv_leave = nh_.subscribe(topics::IN_FV_LEAVE, 100, &Platooning::hndl_msg_fv_leave, this);

		pub_lv_accept = nh_.advertise<platooning::lv_accept>(topics::OUT_LV_ACCEPT, 100);
		pub_lv_reject = nh_.advertise<platooning::lv_reject>(topics::OUT_LV_REJECT, 100);
		pub_lv_broadcast = nh_.advertise<platooning::lv_broadcast>(topics::OUT_LV_BROADCAST, 1);
	} else {
		sub_lv_accept = nh_.subscribe(topics::IN_LV_ACCEPT, 100, &Platooning::hndl_msg_lv_accept, this);
		sub_lv_reject = nh_.subscribe(topics::IN_LV_REJECT, 100, &Platooning::hndl_msg_lv_reject, this);
		sub_lv_broadcast = nh_.subscribe(topics::IN_LV_BROADCAST, 100, &Platooning::hndl_msg_lv_broadcast, this);

		pub_fv_leave = nh_.advertise<platooning::fv_leave>(topics::OUT_FV_LEAVE, 100);
		pub_fv_request = nh_.advertise<platooning::fv_request>(topics::OUT_FV_REQUEST, 100);
		pub_fv_heartbeat = nh_.advertise<platooning::fv_heartbeat>(topics::OUT_FV_HEARTBEAT, 100);
	}

	NODELET_INFO( std::string( "[" + name_ + "] init done").c_str());

};

void Platooning::reset_state() {
	thread_pool_.interrupt_all();
	sub_fv_leave.shutdown();
	sub_lv_accept.shutdown();
	sub_lv_reject.shutdown();
	sub_fv_request.shutdown();
	sub_lv_broadcast.shutdown();
	sub_fv_heartbeat.shutdown();
	sub_platooning_toggle.shutdown();

	pub_fv_leave.shutdown();
	pub_lv_accept.shutdown();
	pub_lv_reject.shutdown();
	pub_fv_request.shutdown();
	pub_lv_broadcast.shutdown();
	pub_fv_heartbeat.shutdown();
	pub_platooning_state.shutdown();

	vehicle_id_ = 3; //saved vehicle id. hardcoded for now

	//our role in the platoon. hardcoded for now
	platoon_role_ = LV;

	//state vars
	platooning_state_ = IDLE;
	platoon_data_ = platooningState();

	//heartbeat timer
	io_service_.stop();
	fv_heartbeat_sender_.cancel();
	fv_heartbeat_checker_.cancel();
	lv_broadcast_sender_.cancel();
	lv_broadcast_checker_.cancel();

	//follower list and timeouts;
	fv_heartbeat_timeout_tracker_.clear();
	lv_broadcast_timeout_tracker_.first = 0;
	lv_broadcast_timeout_tracker_.second = boost::chrono::system_clock::time_point();

	onInit();
}

void Platooning::hndl_msg_fv_leave(const platooning::fv_leave &msg) {
	NODELET_WARN("handling fv_leave");

	if (platooning_state_ == IDLE) {
		return;
	}

	if (platoon_role_ == LV) {
		auto ix = std::find(platoon_data_.platoon_members.begin(),platoon_data_.platoon_members.end(),msg.src_vehicle);
		if( ix != platoon_data_.platoon_members.end() ) {
			fv_heartbeat_timeout_tracker_.erase(*ix);
			platoon_data_.platoon_members.erase(ix);
		}

		if( platoon_data_.platoon_members.empty() ) {
			reset_state();
			platooning_state_ = CREATING;
			auto msg_to_send = boost::shared_ptr<platooning::platooningState>(new platooning::platooningState());
			//does this really copy?
			*msg_to_send = platoon_data_;
			pub_platooning_state.publish(msg_to_send);
		}
	}

	if (platoon_role_ == FV) {
		NODELET_ERROR(std::string("[" + name_ + "] received fv_leave while FV. shouldnt happen").c_str());
		return;
	}

}

/**
 * @brief On lv_accept receipt, starts lv_broadcast timeout timers and the fv_heartbeat senders
 * @param const platooning::lv_accept &
 */
void Platooning::hndl_msg_lv_accept(const platooning::lv_accept &msg) {
	NODELET_WARN("handling lv_accept");

	if (platooning_state_ == IDLE) {
		return;
	}

	if (platoon_role_ == LV) {
		NODELET_ERROR(std::string("[" + name_ + "] received lv_accept while LV. shouldnt happen").c_str());
		return;
	}

	if (platoon_role_ == FV) {
		platooning_state_ = RUNNING;

		lv_broadcast_timeout_tracker_.first = msg.src_vehicle;
		lv_broadcast_timeout_tracker_.second = boost::chrono::system_clock::now();

		hndl_timeout_lv_broadcast(boost::system::error_code());
		send_fv_heartbeat(boost::system::error_code());
	}
}

/**
 * @brief as FV we ignore reject and just keep sending
 * @param const platooning::lv_reject &
 */
void Platooning::hndl_msg_lv_reject(const platooning::lv_reject &msg) {
	NODELET_WARN("handling lv_reject");

	if (platooning_state_ != RUNNING) {
		return;
	}

	if (platoon_role_ == LV) {
		NODELET_ERROR(std::string("[" + name_ + "] received lv_reject while LV. shouldnt happen").c_str());
		return;
	}

	if (platoon_role_ == FV) {
		//keep sendin
	}
}

/**
 * @brief If we are LV in RUNNING or CREATING, we add the src_vehicle and send accept. Also start timers.
 * @param const platooning::fv_request &
 */
void Platooning::hndl_msg_fv_request(const platooning::fv_request &msg) {
	NODELET_WARN("handling fv_request");

	if (platooning_state_ == IDLE) {
		return;
	}

	//max platoonsize 5
	if( platoon_data_.platoon_members.size() > 5 ) {
		return;
	}

	if (platoon_role_ == LV) {

		//already in platoon
		if (fv_heartbeat_timeout_tracker_.find(msg.src_vehicle) == fv_heartbeat_timeout_tracker_.end()) {
			fv_heartbeat_timeout_tracker_.emplace(msg.src_vehicle, boost::chrono::system_clock::now());
		}

		auto msg_to_send = boost::shared_ptr<lv_accept>(new lv_accept);
		msg_to_send->src_vehicle = vehicle_id_;
		msg_to_send->platoon_id = platoon_data_.platoon_id;
		msg_to_send->dst_vehicle = msg.src_vehicle;

		pub_lv_accept.publish(msg_to_send);

		hndl_timeout_fv_heartbeat(boost::system::error_code());
		send_lv_broadcast(boost::system::error_code());
	}

	if (platoon_role_ == FV) {
		NODELET_ERROR(std::string("[" + name_ + "] received fv_request while FV. shouldnt happen").c_str());
		return;
	}
}

/**
 * @brief On receive broadcast, update speed, platoon distance, members and note that LV sent broadcast.
 * @param const platooning::lv_broadcast &
 */
void Platooning::hndl_msg_lv_broadcast(const platooning::lv_broadcast &msg) {
	NODELET_WARN("handling lv_broadcast");

	if (platooning_state_ != RUNNING) {
		return;
	}

	if (msg.src_vehicle != lv_broadcast_timeout_tracker_.first
		|| msg.platoon_id != platoon_data_.platoon_id) {
		NODELET_ERROR(std::string("[" + name_ + "] received lv_broadcast from invalid source.\nwas vehicle_id:"
			                          + std::to_string(msg.src_vehicle) + " expected:"
			                          + std::to_string(lv_broadcast_timeout_tracker_.first) + "\n"
			                          + "platoon:" + std::to_string(msg.platoon_id) + " expected:"
			                          + std::to_string(platoon_data_.platoon_id)
		).c_str());
		return;
	}

	if (platoon_role_ == LV) {
		NODELET_ERROR(std::string("[" + name_ + "] received lv_broadcast while LV. shouldnt happen").c_str());
		return;
	}

	if (platoon_role_ == FV) {
		update_platoonState(msg);
		lv_broadcast_timeout_tracker_.second = boost::chrono::system_clock::now();
	}
}

/**
 * @brief On receive fv_heartbeat, update timetracker for that vehicle.
 * @param const platooning::fv_heartbeat &
 */
void Platooning::hndl_msg_fv_heartbeat(const platooning::fv_heartbeat &msg) {
	NODELET_WARN("handling fv_heartbeat");

	if (platooning_state_ == IDLE) {
		return;
	}

	if (msg.platoon_id != platoon_data_.platoon_id) {
		return;
	}

	if (platoon_role_ == LV) {
		if (fv_heartbeat_timeout_tracker_.find(msg.src_vehicle) != fv_heartbeat_timeout_tracker_.end()) {
			fv_heartbeat_timeout_tracker_[msg.src_vehicle] = boost::chrono::system_clock::now();
		} else {
			NODELET_ERROR(std::string(
				"[" + name_ + "] received fv_heartbeat, but FV not in platoon_members. shouldnt happen").c_str());
		}
		return;
	}

	if (platoon_role_ == FV) {
		NODELET_ERROR(std::string("[" + name_ + "] received fv_heartbeat while FV. shouldnt happen").c_str());
		return;
	}
}

/**
* @brief If platooning is on and we are FV, turns platooning off. Else starts CREATING mode.
*        If we are LV, we wait, if we are FV we start sending FV_REQUEST.
* @param const platooning::platooningToggle &
*/
void Platooning::hndl_msg_platooning_toggle(const platooning::platooningToggle &msg) {
	NODELET_WARN("handling platooning_toggle");

	if( !msg.enable_platooning ) {
		//turn off platooning as FV
		if ( platoon_role_ == FV) {

			platooning_state_ = IDLE;
			fv_heartbeat_sender_.cancel();

			auto msg_to_send = boost::shared_ptr<fv_leave>(new fv_leave());
			msg_to_send->src_vehicle = vehicle_id_;
			msg_to_send->platoon_id = platoon_data_.platoon_id;

			pub_fv_leave.publish(msg_to_send);
			return;
		}

		//turn off platooning as LV only if no members
		if ( platoon_role_ == LV && !platoon_data_.platoon_members.empty()) {
			NODELET_ERROR(std::string("[" + name_ + "] i am leader vehicle. exiting platoon not allowed when still members in platoon").c_str());
			//TODO: send error to ui
			return;
		}

		if ( platoon_role_ == LV && platoon_data_.platoon_members.empty()) {
			reset_state();
			platooning_state_ = IDLE;
			return;
		}
	} // end msg.enable == false

	if( msg.enable_platooning) {
		//received platooning enable, start in creating.
		//if other state than IDLE, error
		if ( platooning_state_ == IDLE) {
			platooning_state_ = CREATING;
		} else if (msg.enable_platooning && platooning_state_ != IDLE) {
			NODELET_ERROR(std::string("[" + name_ + "] received platooning start while state was "
				                          + PlatooningStateStrings[platooning_state_]).c_str());
		}

		//if we are LV, start setting state and wait for fv_request
		if (platoon_role_ == LV) {
			platoon_data_.vehicle_id = vehicle_id_;
			platoon_data_.platoon_id = vehicle_id_;

			platoon_data_.i_am_LV = true;
			platoon_data_.i_am_FV = false;

			platoon_data_.ipd = msg.inner_platoon_distance;
			platoon_data_.ps = msg.platoon_speed;
			platoon_data_.platooning_state = PlatooningStateStrings[platooning_state_];

			auto msg_to_send = boost::shared_ptr<platooningState>(new platooningState());
			//does this really copy?
			*msg_to_send = platoon_data_;
			pub_platooning_state.publish(msg_to_send);
			//waiting for FV_REQUEST
		}

		//if we are fv, start sending fv_requests until we are accepted
		if (platoon_role_ == FV) {
			auto msg_to_send = boost::shared_ptr<fv_request>(new fv_request());
			msg_to_send->src_vehicle = vehicle_id_;

			thread_pool_.create_thread([this, msg_to_send] {
				while (this->platooning_state_ != RUNNING) {
					pub_fv_request.publish(msg_to_send);
					boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
				}
			});
		}
	} // end msg.enable == true

}

/**
 * @brief Updates internal platoonstate from received broadcast.
 * @param lv_broadcast
 */
void Platooning::update_platoonState(lv_broadcast bc) {
	bool has_changed = false;

	if (bc.platoon_id != platoon_data_.platoon_id) {
		return;
	}

	if (bc.ps != platoon_data_.ps) {
		platoon_data_.ps = bc.ps;
		has_changed = true;
	}

	if (bc.ipd != platoon_data_.ipd) {
		platoon_data_.ipd = bc.ipd;
		has_changed = true;
	}

	if (bc.followers != platoon_data_.platoon_members) {
		platoon_data_.platoon_members = bc.followers;
		has_changed = true;
	}

	if (has_changed) {
		auto msg_to_send = boost::shared_ptr<platooning::platooningState>(new platooning::platooningState());

		//does this really copy?
		*msg_to_send = platoon_data_;

		pub_platooning_state.publish(msg_to_send);
	}
}

/**
 * @brief Checks for each platoon member whether they've checked in recently.
 *        If not removes them from platoon_members. If platoonsize > 0, restarts timer.
 * @param const boost::system::error_code &
 */
void Platooning::hndl_timeout_fv_heartbeat(const boost::system::error_code &e) {
	//timer cancelled
	if (boost::asio::error::operation_aborted == e) {
		return;
	}

	if (e) {
		NODELET_ERROR(std::string("[" + name_ + "] hndl_timeout_fv_heartbeat uncaught error " + e.message()).c_str());
	}

	//kick out all members that havent sent a heartbeat in time
	bool members_timed_out = false;
	boost::chrono::system_clock::time_point now = boost::chrono::system_clock::now();
	std::vector<uint32_t> to_del;
	for (auto &member : fv_heartbeat_timeout_tracker_) {
		if ((now - member.second) > boost::chrono::milliseconds(HEARTBEAT_TIMEOUT.total_milliseconds())) {
			to_del.push_back(member.first); //save timed out member
			members_timed_out = true;
			//send courtesy reject
			auto msg_to_send = boost::shared_ptr<platooning::lv_reject>(new platooning::lv_reject());
			msg_to_send->src_vehicle = vehicle_id_;
			msg_to_send->dst_vehicle = member.first;
			pub_lv_reject.publish(msg_to_send);
		}
	}

	//delete from timeout tracking
	//end member vector
	for (auto &member : to_del) {
		fv_heartbeat_timeout_tracker_.erase(member);
		auto ix = std::find(platoon_data_.platoon_members.begin(), platoon_data_.platoon_members.end(), member);
		if( ix != platoon_data_.platoon_members.end() ) {
			platoon_data_.platoon_members.erase(ix);
		}

	}

	if (fv_heartbeat_timeout_tracker_.empty()) {
		platooning_state_ = CREATING;
	} else {

		fv_heartbeat_checker_.expires_from_now(HEARTBEAT_TIMEOUT);
		fv_heartbeat_checker_.async_wait(boost::bind(&Platooning::hndl_timeout_fv_heartbeat, this,
		                                             boost::asio::placeholders::error));

		if (io_service_.stopped()) {
			thread_pool_.create_thread([this] {
				while (this->platooning_state_ != IDLE) {
					io_service_.run();
				}
			});
		}
	}

	if (members_timed_out) {
		//TODO: send errormessage to ui
		auto msg_to_send = boost::shared_ptr<platooning::platooningState>(new platooning::platooningState());
		*msg_to_send = platoon_data_;
		pub_platooning_state.publish(msg_to_send);
	}
}

/**
 * @brief Checks when the last broadcast was, if longer than timeout, leave platoon.
 *        Else restart timer
 * @param const boost::system::error_code &
 */
void Platooning::hndl_timeout_lv_broadcast(const boost::system::error_code &e) {
	//timer cancelled
	if (boost::asio::error::operation_aborted == e) {
		return;
	}

	if (e) {
		NODELET_ERROR(std::string("[" + name_ + "] hndl_timeout_fv_heartbeat uncaught error " + e.message()).c_str());
		return;
	}

	boost::chrono::system_clock::time_point now = boost::chrono::system_clock::now();

	if (now - lv_broadcast_timeout_tracker_.second
		> boost::chrono::milliseconds(BROADCAST_TIMEOUT.total_milliseconds())) {
		NODELET_FATAL(std::string("[" + name_ + "] hndl_timeout_lv_broadcast: LV timeout").c_str());

		reset_state();

		//TODO: send errormessage to ui
		auto msg_to_send = boost::shared_ptr<platooning::platooningState>(new platooning::platooningState());
		*msg_to_send = platoon_data_;
		pub_platooning_state.publish(msg_to_send);
	} else {
		//restart timers and check if io_service is still running
		lv_broadcast_checker_.expires_from_now(BROADCAST_TIMEOUT);
		lv_broadcast_checker_.async_wait(boost::bind(&Platooning::hndl_timeout_lv_broadcast, this,
		                                             boost::asio::placeholders::error));

		if (io_service_.stopped()) {
			thread_pool_.create_thread([this] {
				while (this->platooning_state_ != IDLE) {
					io_service_.run();
				}
			});
		}
	}
}

/**
 * @brief sends heartbeat, restarts timer.
 * @param const boost::system::error_code &
 */
void Platooning::send_fv_heartbeat(const boost::system::error_code &e) {

	if (boost::asio::error::operation_aborted == e) {
		return;
	}

	//send fv_heartbeat
	auto msg_to_send = boost::shared_ptr<fv_heartbeat>(new fv_heartbeat);
	msg_to_send->src_vehicle = vehicle_id_;
	msg_to_send->platoon_id = platoon_data_.platoon_id;
	pub_fv_heartbeat.publish(msg_to_send);

	//restart timers
	fv_heartbeat_sender_.expires_from_now(HEARTBEAT_FREQ);
	fv_heartbeat_sender_.async_wait(boost::bind(&Platooning::send_fv_heartbeat, this,
	                                            boost::asio::placeholders::error));

	//check if io_serivce is still on
	if (io_service_.stopped()) {
		thread_pool_.create_thread([this] {
			while (this->platooning_state_ != IDLE) {
				io_service_.run();
			}
		});
	}
}

/**
 * @brief sends broadcast, restarts timer.
 * @param const boost::system::error_code &
 */
void Platooning::send_lv_broadcast(const boost::system::error_code &e) {

	if (boost::asio::error::operation_aborted == e) {
		return;
	}

	lv_broadcast_sender_.expires_from_now(BROADCAST_FREQ);
	lv_broadcast_sender_.async_wait(boost::bind(&Platooning::send_lv_broadcast, this,
	                                            boost::asio::placeholders::error));

	if (io_service_.stopped()) {
		thread_pool_.create_thread([this] {
			while (this->platooning_state_ != IDLE) {
				io_service_.run();
			}
		});
	}
}

}

PLUGINLIB_EXPORT_CLASS(platooning::Platooning, nodelet::Nodelet);

