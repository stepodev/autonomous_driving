/**
 * @file include/platooning/Platooning.cpp
 * @author stepo
 * @date 22.03.2018
 * @brief Platooning header file

 */

#include "platooning/Platooning.hpp"

namespace platooning {

Platooning::Platooning() :
	io_worker_(io_service_),
	fv_heartbeat_sender_(io_service_),
	fv_heartbeat_checker_(io_service_),
	lv_broadcast_sender_(io_service_),
	lv_broadcast_checker_(io_service_) {

	thread_pool_.create_thread( [this]{ io_service_.run();});

}

Platooning::~Platooning() {
	fv_heartbeat_sender_.cancel();
	fv_heartbeat_checker_.cancel();
	lv_broadcast_sender_.cancel();
	lv_broadcast_checker_.cancel();

	io_service_.stop();

	if( !io_service_.stopped() ) {
		thread_pool_.interrupt_all();
	}

	thread_pool_.join_all();

}

void Platooning::onInit() {

	service_ = nh_.advertiseService(platooning_services::VEHICLE_ID, &Platooning::provide_vehicle_id, this);

	// subscribers nh_.subscribe("Topic", Rate, &Callback, this)
	sub_platooning_toggle_ =
		nh_.subscribe(topics::TOGGLE_PLATOONING, 100, &Platooning::hndl_msg_platooning_toggle, this);

	set_vehicle_id(get_vehicle_id_param());

	NODELET_INFO("[%s] init done", name_.c_str());

};

bool Platooning::provide_vehicle_id(platooning::getVehicleId::Request &req,
                                    platooning::getVehicleId::Response &res) {

	res.vehicle_id = get_vehicle_id();

	return true;
}

void Platooning::reset() {

	//heartbeat timer
	fv_heartbeat_sender_.cancel();
	fv_heartbeat_checker_.cancel();
	lv_broadcast_sender_.cancel();
	lv_broadcast_checker_.cancel();

	//io_service_.stop();

	thread_pool_.interrupt_all();

	//thread_pool_.join_all();

	//thread_pool_.create_thread( [this]{ io_service_.run();});

	shutdown_pubs_and_subs();

	//state vars
	PlatooningState::reset();

	set_vehicle_id(get_vehicle_id_param());
	set_platoon_id(get_vehicle_id());

	//follower list and timeouts;
	fv_heartbeat_timeout_tracker_.clear();
	lv_broadcast_timeout_tracker_.first = 0;
	lv_broadcast_timeout_tracker_.second = boost::posix_time::microsec_clock::local_time();
}

/**
 * @brief handles received fv_leave messages for LV. Remove vehicle that requested to leave. If no members left
 *        return to CREATING mode
 * @param fv_leave
 */
void Platooning::hndl_msg_fv_leave(const platooning::fv_leave &msg) {
	if (get_mode() == PlatooningModeEnum::IDLE) {
		return;
	}

	if (msg.platoon_id != get_platoon_id()) {
		return;
	}

	NODELET_WARN("[%s] received FV_LEAVE from vehicle %i", name_.c_str(), msg.src_vehicle);

	if (get_role() == PlatoonRoleEnum::LV) {
		remove_member(msg.src_vehicle);
		fv_heartbeat_timeout_tracker_.erase(msg.src_vehicle);

		if (!has_members()) {
			reset();
			set_role(PlatoonRoleEnum::LV);
			set_mode(PlatooningModeEnum::CREATING);
		}
	}

	if (get_role() == PlatoonRoleEnum::FV) {
		NODELET_ERROR("[%s] received FV_LEAVE while FV. shouldnt happen", name_.c_str());
		return;
	}

}

/**
 * @brief On lv_accept receipt, starts lv_broadcast timeout timers and the fv_heartbeat senders
 * @param const platooning::lv_accept &
 */
void Platooning::hndl_msg_lv_accept(const platooning::lv_accept &msg) {

	if (msg.dst_vehicle != get_vehicle_id()) {
		return;
	}

	if (get_mode() == PlatooningModeEnum::IDLE) {
		return;
	}

	if (get_role() == PlatoonRoleEnum::LV) {
		NODELET_ERROR("[%s] received lv_accept while LV. shouldnt happen", name_.c_str());
		return;
	}

	if (get_role() == PlatoonRoleEnum::FV) {
		NODELET_INFO("[%s] received LV_ACCEPT from vehicle %i of platoon %i",
		             name_.c_str(),
		             msg.src_vehicle,
		             msg.platoon_id);

		set_mode(PlatooningModeEnum::RUNNING);
		set_platoon_id(msg.platoon_id);
		set_leader_id(msg.src_vehicle);

		lv_broadcast_timeout_tracker_.first = msg.src_vehicle;
		lv_broadcast_timeout_tracker_.second = boost::posix_time::microsec_clock::local_time();

		hndl_timeout_lv_broadcast(boost::system::error_code());
		send_fv_heartbeat(boost::system::error_code());
	}
}

/**
 * @brief as FV we ignore reject and just keep sending
 * @param const platooning::lv_reject &
 */
void Platooning::hndl_msg_lv_reject(const platooning::lv_reject &msg) {

	if (msg.dst_vehicle != get_vehicle_id()) {
		return;
	}

	if (get_mode() != PlatooningModeEnum::CREATING) {
		return;
	}

	NODELET_WARN("[%s] received LV_REJECT from vehicle %i of platoon %i",
	             name_.c_str(),
	             msg.src_vehicle,
	             msg.platoon_id);

	if (get_role() == PlatoonRoleEnum::LV) {
		NODELET_ERROR("[%s] received lv_reject while LV. shouldnt happen", name_.c_str());
		return;
	}

	if (get_role() == PlatoonRoleEnum::FV) {
		//keep sendin
	}
}

/**
 * @brief If we are LV in RUNNING or CREATING, we add the src_vehicle and send accept. Also start timers.
 * @param const platooning::fv_request &
 */
void Platooning::hndl_msg_fv_request(const platooning::fv_request &msg) {
	if (get_mode() != PlatooningModeEnum::CREATING && get_mode() != PlatooningModeEnum::IDLE) {
		NODELET_WARN("[%s] ignoring FV_REQUEST dt platooning mode %s",
		             name_.c_str(), to_string(get_mode()).c_str());
		return;
	}

	//max platoonsize 5
	if (member_count() > 5) {
		NODELET_WARN("[%s] ignoring FV_REQUEST dt platoonsize exceeded", name_.c_str());
		return;
	}

	if (get_role() == PlatoonRoleEnum::LV) {

		NODELET_INFO("[%s] accepting FV_REQUEST for vehicle %i", name_.c_str(), msg.src_vehicle);

		add_member(msg.src_vehicle);

		auto msg_to_send = boost::shared_ptr<lv_accept>(new lv_accept);
		msg_to_send->src_vehicle = get_vehicle_id();
		msg_to_send->platoon_id = get_platoon_id();
		msg_to_send->dst_vehicle = msg.src_vehicle;

		pub_lv_accept_.publish(msg_to_send);

		set_mode(PlatooningModeEnum::RUNNING);

		//emplace new member in timers
		if (fv_heartbeat_timeout_tracker_.find(msg.src_vehicle) != fv_heartbeat_timeout_tracker_.end()) {
			fv_heartbeat_timeout_tracker_.emplace(msg.src_vehicle, boost::posix_time::microsec_clock::local_time());
		} else {
			fv_heartbeat_timeout_tracker_[msg.src_vehicle] = boost::posix_time::microsec_clock::local_time();
		}

		send_lv_broadcast(boost::system::error_code());
		hndl_timeout_fv_heartbeat(boost::system::error_code());
	}

	if (get_role() == PlatoonRoleEnum::FV) {
		NODELET_ERROR("[%s] received FV_REQUEST while FV. shouldnt happen", name_.c_str());
		return;
	}
}

/**
 * @brief On receive broadcast, update speed, platoon distance, members and note that LV sent broadcast.
 * @param const platooning::lv_broadcast &
 */
void Platooning::hndl_msg_lv_broadcast(const platooning::lv_broadcast &msg) {

	if (get_mode() != PlatooningModeEnum::RUNNING) {
		return;
	}

	if (get_role() == PlatoonRoleEnum::LV) {
		NODELET_ERROR("[%s] received LV_BROADCAST while LV. shouldnt happen", name_.c_str());
		return;
	}

	if (msg.src_vehicle != get_leader_id()
		|| msg.platoon_id != get_platoon_id()) {
		NODELET_ERROR(
			"[%s] received lv_broadcast from invalid source. \nwas vehicle_id %i, expected %i\nplatoon_id %i expected %i ",
			name_.c_str(),
			msg.src_vehicle,
			get_leader_id(),
			msg.platoon_id,
			get_platoon_id());
		return;
	}

	if (get_role() == PlatoonRoleEnum::FV) {
		update_state(msg);
		lv_broadcast_timeout_tracker_.second = boost::posix_time::microsec_clock::local_time();
	}
}

/**
 * @brief On receive fv_heartbeat, update timetracker for that vehicle.
 * @param const platooning::fv_heartbeat &
 */
void Platooning::hndl_msg_fv_heartbeat(const platooning::fv_heartbeat &msg) {

	if (get_mode() == PlatooningModeEnum::IDLE) {
		NODELET_ERROR("[%s] received FV_HEARTBEAT while IDLE. shouldnt happen", name_.c_str());
		return;
	}

	//wrong platoon id, ignore
	if (msg.platoon_id != get_platoon_id()) {
		return;
	}

	if (get_role() == PlatoonRoleEnum::LV) {
		if (fv_heartbeat_timeout_tracker_.find(msg.src_vehicle) != fv_heartbeat_timeout_tracker_.end()) {
			fv_heartbeat_timeout_tracker_[msg.src_vehicle] = boost::posix_time::microsec_clock::local_time();
		} else {
			NODELET_ERROR("[%s] received FV_HEARTBEAT, but FV not in platoon_members. shouldnt happen", name_.c_str());
		}
		return;
	}

	if (get_role() == PlatoonRoleEnum::FV) {
		NODELET_ERROR("[%s] received FV_HEARTBEAT while FV. shouldnt happen", name_.c_str());
		return;
	}
}

/**
* @brief If platooning is on and we are FV, turns platooning off. Else starts CREATING mode.
*        If we are LV, we wait, if we are FV we start sending FV_REQUEST.
* @param const platooning::platooningToggle &
*/
void Platooning::hndl_msg_platooning_toggle(const platooning::platooningToggle &msg) {
	if (msg.vehicle_id != get_vehicle_id()) {
		return;
	}

	//allow role change only during idle or none
	if ((get_mode() != PlatooningModeEnum::IDLE)
		&& ((msg.lvfv == "FV" && get_role() != PlatoonRoleEnum::FV)
			|| (msg.lvfv == "LV" && get_role() != PlatoonRoleEnum::LV))) {
		NODELET_ERROR("[%s] attempt to change platooning role while in platooning mode %s",
		              name_.c_str(),
		              to_string(get_mode()).c_str());
		return;
	}

	if (!msg.enable_platooning) {
		//turn off platooning as FV
		if (get_role() == PlatoonRoleEnum::FV) {

			NODELET_INFO("[%s] turning off platooning as FV", name_.c_str());

			auto msg_to_send = boost::shared_ptr<fv_leave>(new fv_leave());
			msg_to_send->src_vehicle = get_vehicle_id();
			msg_to_send->platoon_id = get_platoon_id();

			pub_fv_leave_.publish(msg_to_send);
			reset();
			return;
		}

		//turn off platooning as LV only if no members
		if (get_role() == PlatoonRoleEnum::LV && has_members()) {
			NODELET_ERROR(
				"[%s] Attempt to stop platooning while LV with platoon members. exiting platoon not allowed when still members in platoon.",
				name_.c_str());
			//TODO: send error to ui
			return;
		}

		if (get_role() == PlatoonRoleEnum::LV && !has_members()) {
			NODELET_INFO("[%s] turning off platooning as LV", name_.c_str());
			reset();
			return;
		}
	} // end msg.enable == false

	if (msg.enable_platooning) {

		//change platoon role
		if (msg.lvfv == "FV") {
			set_role(PlatoonRoleEnum::FV);

			NODELET_INFO(std::string("[%s] changing platooning role to %s").c_str(),
			             name_.c_str(),
			             to_string(get_role()).c_str());
		} else if (msg.lvfv == "LV") {
			set_role(PlatoonRoleEnum::LV);
			set_platoon_id(get_vehicle_id());
			NODELET_INFO(std::string("[%s] changing platooning role to %s").c_str(),
			             name_.c_str(),
			             to_string(get_role()).c_str());
		} else {
			NODELET_ERROR("[%s] attempt to change platooning role to unknown role %s ",
			              name_.c_str(),
			              msg.lvfv.c_str());
			return;
		}

		//received platooning enable, start in creating.
		//if other state than IDLE, error
		if (get_mode() == PlatooningModeEnum::IDLE) {
			set_mode(PlatooningModeEnum::CREATING);
			set_platoon_distance(msg.inner_platoon_distance);
			set_platoon_speed(msg.platoon_speed);
			NODELET_INFO(std::string("[%s] going into platooning mode %s").c_str(), name_.c_str(),
			             to_string(get_mode()).c_str());

		} else if (msg.enable_platooning && get_mode() != PlatooningModeEnum::IDLE) {
			NODELET_ERROR("[%s] received platooning start while mode was %s", name_.c_str(),
			              to_string(get_mode()).c_str());
			return;
		}

		//if we are fv, start sending fv_requests until we are accepted
		if (get_role() == PlatoonRoleEnum::FV) {

			NODELET_INFO("[%s] %s in %s mode. Sending FV_REQUEST", name_.c_str(), to_string(get_role()).c_str(),
			             to_string(get_mode()).c_str());

			thread_pool_.create_thread([this] {
				auto msg_to_send = boost::shared_ptr<fv_request>(new fv_request());
				msg_to_send->src_vehicle = get_vehicle_id();
				while (this->get_mode() == PlatooningModeEnum::CREATING) {

					NODELET_INFO("[%s] %s in %s mode. Sending FV_REQUEST",
					             name_.c_str(),
					             to_string(get_role()).c_str(),
					             to_string(get_mode()).c_str());

					pub_fv_request_.publish(msg_to_send);
					boost::this_thread::sleep_for(boost::chrono::seconds(2));
				}
			});
		}
	} // end msg.enable == true

}

/**
 * @brief Updates internal platoonstate from received broadcast.
 * @param lv_broadcast
 */
void Platooning::update_state(const lv_broadcast &bc) {
	if (bc.platoon_id != get_platoon_id()) {
		return;
	}

	if (bc.ps != get_platoon_speed()) {
		NODELET_INFO("[%s] LV_BROADCAST change received. platoon speed from %f to %f",
		             name_.c_str(),
		             bc.ps,
		             get_platoon_speed());
		set_platoon_speed(bc.ps);
	}

	if (bc.ipd != get_platoon_distance()) {
		NODELET_INFO("[%s] LV_BROADCAST change received. platoon distance from %f to %f",
		             name_.c_str(),
		             bc.ipd,
		             get_platoon_distance());
		set_platoon_distance(bc.ipd);
	}

	if (bc.followers != get_members()) {

		NODELET_INFO("[%s] LV_BROADCAST change received. platoon members changed", name_.c_str());

		set_members(bc.followers);
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
		NODELET_ERROR("[%s] hndl_timeout_fv_heartbeat uncaught error type: %s ", name_.c_str(), e.message().c_str());
	}

	//kick out all members that havent sent a heartbeat in time
	std::vector<uint32_t> to_del;
	for (auto &member : fv_heartbeat_timeout_tracker_) {

		boost::posix_time::time_duration dur = boost::posix_time::microsec_clock::local_time() - member.second;

		if (dur > HEARTBEAT_TIMEOUT) {

			NODELET_WARN( "[%s] FV_HEARTBEAT vehicle %i timeout with timediff: %i max: %i", name_.c_str(), (int)member.first,
			              (int)dur.total_milliseconds(), (int)HEARTBEAT_TIMEOUT.total_milliseconds());

			to_del.push_back(member.first); //save timed out member
			//send courtesy reject
			//auto msg_to_send = boost::shared_ptr<platooning::lv_reject>(new platooning::lv_reject());
			//msg_to_send->src_vehicle = get_vehicle_id();
			//msg_to_send->dst_vehicle = member.first;
			//pub_lv_reject_.publish(msg_to_send);
		}
	}

	//delete from timeout tracking
	//end member vector
	for (auto &member : to_del) {
		fv_heartbeat_timeout_tracker_.erase(member);
		remove_member(member);
	}

	if (fv_heartbeat_timeout_tracker_.empty()) {

		if( has_members() ) {
			NODELET_ERROR("[%s] heartbeat tracker empty but still has members", name_.c_str());
		}

		NODELET_WARN("[%s] All members in platoon timed out. Fallback to CREATING", name_.c_str());
		set_mode(PlatooningModeEnum::CREATING);
	} else {

		fv_heartbeat_checker_.expires_from_now(HEARTBEAT_TIMEOUT);
		fv_heartbeat_checker_.async_wait(boost::bind(&Platooning::hndl_timeout_fv_heartbeat, this,
		                                             boost::asio::placeholders::error));
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
		NODELET_ERROR("[%s] hndl_timeout_fv_heartbeat uncaught error %s", name_.c_str(), e.message().c_str());
		return;
	}

	boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();

	if (now - lv_broadcast_timeout_tracker_.second > BROADCAST_TIMEOUT) {
		NODELET_FATAL("[%s] LV_BROADCAST timeout. Resetting platooning", name_.c_str());

		reset();

	} else {
		//restart timers and check if io_service is still running
		lv_broadcast_checker_.expires_from_now(BROADCAST_TIMEOUT);
		lv_broadcast_checker_.async_wait(boost::bind(&Platooning::hndl_timeout_lv_broadcast, this,
		                                             boost::asio::placeholders::error));
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

	try {
		//send fv_heartbeat
		auto msg_to_send = boost::shared_ptr<fv_heartbeat>(new fv_heartbeat);
		msg_to_send->src_vehicle = get_vehicle_id();
		msg_to_send->platoon_id = get_platoon_id();
		pub_fv_heartbeat_.publish(msg_to_send);

		//restart timers
		fv_heartbeat_sender_.expires_from_now(HEARTBEAT_FREQ);
		fv_heartbeat_sender_.async_wait(boost::bind(&Platooning::send_fv_heartbeat, this,
		                                            boost::asio::placeholders::error));
	} catch ( std::exception& ex ) {
		NODELET_ERROR("[%s] send_fv_heartbeat failed with %s", name_.c_str(), ex.what() );
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

	try {

	//send fv_heartbeat
	auto msg_to_send = boost::shared_ptr<lv_broadcast>(new lv_broadcast);
	msg_to_send->src_vehicle = get_vehicle_id();
	msg_to_send->platoon_id = get_platoon_id();
	msg_to_send->ps = get_platoon_speed();
	msg_to_send->ipd = get_platoon_distance();
	msg_to_send->followers = get_members();
	pub_lv_broadcast_.publish(msg_to_send);

	lv_broadcast_sender_.expires_from_now(BROADCAST_FREQ);
	lv_broadcast_sender_.async_wait(boost::bind(&Platooning::send_lv_broadcast, this,
	                                            boost::asio::placeholders::error));

	} catch ( std::exception& ex ) {
		NODELET_ERROR("[%s] send_lv_broadcast failed with %s", name_.c_str(), ex.what() );
	}
}

uint32_t Platooning::get_vehicle_id_param() {

	uint32_t vehicle_id = 1;
	if (nh_.hasParam("vehicle_id")) {
		int paramvehicleid = 0;
		nh_.getParam("vehicle_id", paramvehicleid);

		if (paramvehicleid < 1) {
			NODELET_ERROR("[%s] invalid vehicle id parameter. Defaulting to 1", name_.c_str());
			vehicle_id = 1;
		} else {
			vehicle_id = (uint32_t) paramvehicleid;
		}
	} else {
		NODELET_ERROR("[%s] no vehicle id parameter. Defaulting to 1", name_.c_str());
		vehicle_id = 1;
	}

	return vehicle_id;
}

void Platooning::shutdown_pubs_and_subs() {
	sub_fv_leave_.shutdown();
	sub_lv_accept_.shutdown();
	sub_lv_reject_.shutdown();
	sub_fv_request_.shutdown();
	sub_lv_broadcast_.shutdown();
	sub_fv_heartbeat_.shutdown();

	pub_fv_leave_.shutdown();
	pub_lv_accept_.shutdown();
	pub_lv_reject_.shutdown();
	pub_fv_request_.shutdown();
	pub_lv_broadcast_.shutdown();
	pub_fv_heartbeat_.shutdown();

}

void Platooning::set_role(const PlatoonRoleEnum &roleenum) {

	PlatooningState::set_role(roleenum);

	//shutdown all
	shutdown_pubs_and_subs();

	//spin up relevant subs and pubgs
	if (get_role() == PlatoonRoleEnum::LV) {

		NODELET_WARN( "[%s] starting pubs and subs for LV role", name_.c_str() );

		sub_fv_request_ = nh_.subscribe(topics::IN_FV_REQUEST, 100, &Platooning::hndl_msg_fv_request, this);
		sub_fv_heartbeat_ = nh_.subscribe(topics::IN_FV_HEARTBEAT, 100, &Platooning::hndl_msg_fv_heartbeat, this);
		sub_fv_leave_ = nh_.subscribe(topics::IN_FV_LEAVE, 100, &Platooning::hndl_msg_fv_leave, this);

		pub_lv_accept_ = nh_.advertise<platooning::lv_accept>(topics::OUT_LV_ACCEPT, 100);
		pub_lv_reject_ = nh_.advertise<platooning::lv_reject>(topics::OUT_LV_REJECT, 100);
		pub_lv_broadcast_ = nh_.advertise<platooning::lv_broadcast>(topics::OUT_LV_BROADCAST, 1);
	} else if (get_role() == PlatoonRoleEnum::FV) {

		NODELET_WARN( "[%s] starting pubs and subs for FV role", name_.c_str() );

		sub_lv_accept_ = nh_.subscribe(topics::IN_LV_ACCEPT, 100, &Platooning::hndl_msg_lv_accept, this);
		sub_lv_reject_ = nh_.subscribe(topics::IN_LV_REJECT, 100, &Platooning::hndl_msg_lv_reject, this);
		sub_lv_broadcast_ = nh_.subscribe(topics::IN_LV_BROADCAST, 100, &Platooning::hndl_msg_lv_broadcast, this);

		pub_fv_leave_ = nh_.advertise<platooning::fv_leave>(topics::OUT_FV_LEAVE, 100);
		pub_fv_request_ = nh_.advertise<platooning::fv_request>(topics::OUT_FV_REQUEST, 100);
		pub_fv_heartbeat_ = nh_.advertise<platooning::fv_heartbeat>(topics::OUT_FV_HEARTBEAT, 100);
	}
}

PlatooningState::PlatooningState() {
	pub_platooning_state_= nh_.advertise<platooning::platooningState>(topics::PLATOONINGSTATE, 1);
	reset();
}

void PlatooningState::reset() {
	boost::lock_guard<boost::shared_mutex> l(mtx_);

	role_ = PlatoonRoleEnum::NONE;
	current_mode_ = PlatooningModeEnum::IDLE;
	members_.clear();
	vehicle_id_ = 0;
	platoon_id_ = vehicle_id_;
	leader_id_ = UINT32_MAX;

	mtx_.unlock();

	publish_state();
}

void PlatooningState::set_role(const PlatoonRoleEnum &in) {
	boost::lock_guard<boost::shared_mutex> l(mtx_);

	role_ = in;

	mtx_.unlock();

	publish_state();

}

PlatoonRoleEnum PlatooningState::get_role() {

	boost::shared_lock<boost::shared_mutex> l(mtx_);

	return role_;
}

void PlatooningState::set_mode(const PlatooningModeEnum &m) {
	boost::lock_guard<boost::shared_mutex> l(mtx_);

	current_mode_ = m;

	mtx_.unlock();

	publish_state();
}

PlatooningModeEnum PlatooningState::get_mode() {
	boost::shared_lock<boost::shared_mutex> l(mtx_);

	return current_mode_;
}

void PlatooningState::add_member(const uint32_t &m ) {
	boost::lock_guard<boost::shared_mutex> l(mtx_);

	members_.emplace_back(m);

	mtx_.unlock();


	publish_state();
}

void PlatooningState::remove_member(const uint32_t &src_vehicle) {
	boost::lock_guard<boost::shared_mutex> l(mtx_);

	auto ix = std::find(members_.begin(), members_.end(), src_vehicle);
	if (ix != members_.end()) {
		members_.erase(ix);
	}

	mtx_.unlock();

	publish_state();
}

bool PlatooningState::is_member(const uint32_t &needle) {
	boost::shared_lock<boost::shared_mutex> l(mtx_);

	return std::find(members_.begin(), members_.end(), needle) == members_.end();
}

size_t PlatooningState::member_count() {
	boost::shared_lock<boost::shared_mutex> l(mtx_);

	return members_.size();
}

std::vector<uint32_t> PlatooningState::get_members() {
	boost::shared_lock<boost::shared_mutex> l(mtx_);

	return members_;
}

bool PlatooningState::has_members() {
	boost::shared_lock<boost::shared_mutex> l(mtx_);

	return !members_.empty();
}

void PlatooningState::set_members(const std::vector<uint32_t> &v) {
	boost::lock_guard<boost::shared_mutex> l(mtx_);

	members_.clear();

	for( auto x : v ) {
		members_.push_back(x);
	}

	std::sort(members_.begin(),members_.end(), std::greater<uint32_t>() );

	mtx_.unlock();

	publish_state();
}

void PlatooningState::set_leader_id(const uint32_t &vehicle_id) {
	boost::lock_guard<boost::shared_mutex> l(mtx_);

	leader_id_ = vehicle_id;

	mtx_.unlock();

	publish_state();
}

uint32_t PlatooningState::get_leader_id() {
	boost::shared_lock<boost::shared_mutex> l(mtx_);

	return leader_id_;
}

void PlatooningState::publish_state() {
	boost::shared_lock<boost::shared_mutex> l(mtx_);

	auto statemsg = boost::shared_ptr<platooning::platooningState>(new platooning::platooningState);

	statemsg->vehicle_id = vehicle_id_;
	statemsg->platoon_id = platoon_id_;
	statemsg->platooning_state = to_string(current_mode_);
	statemsg->ipd = platoon_distance_;
	statemsg->ps = platoon_speed_;
	if (role_ == PlatoonRoleEnum::FV) {
		statemsg->i_am_FV = true;
		statemsg->i_am_LV = false;
	} else {
		statemsg->i_am_FV = false;
		statemsg->i_am_LV = true;
	}
	statemsg->platoon_members = members_;

	pub_platooning_state_.publish(statemsg);

}

void PlatooningState::set_vehicle_id(const uint32_t &in) {
	boost::lock_guard<boost::shared_mutex> l(mtx_);

	vehicle_id_ = in;

	mtx_.unlock();

	publish_state();
}

uint32_t PlatooningState::get_vehicle_id() {
	boost::shared_lock<boost::shared_mutex> l(mtx_);

	return vehicle_id_;
}

void PlatooningState::set_platoon_id(const uint32_t &in) {
	boost::lock_guard<boost::shared_mutex> l(mtx_);

	platoon_id_ = in;

	mtx_.unlock();

	publish_state();
}
uint32_t PlatooningState::get_platoon_id() {
	return platoon_id_;
}

void PlatooningState::set_platoon_speed(const float &d) {
	boost::lock_guard<boost::shared_mutex> l(mtx_);

	platoon_speed_ = d;

	mtx_.unlock();

	publish_state();
}

float PlatooningState::get_platoon_speed() {
	boost::shared_lock<boost::shared_mutex> l(mtx_);

	return platoon_speed_;
}

void PlatooningState::set_platoon_distance(const float &d) {
	boost::lock_guard<boost::shared_mutex> l(mtx_);

	platoon_distance_ = d;

	mtx_.unlock();

	publish_state();
}

float PlatooningState::get_platoon_distance() {
	boost::shared_lock<boost::shared_mutex> l(mtx_);

	return platoon_distance_;
}

}

PLUGINLIB_EXPORT_CLASS(platooning::Platooning, nodelet::Nodelet);

