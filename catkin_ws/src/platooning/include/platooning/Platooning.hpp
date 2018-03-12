#ifndef PLATOONING_PLATOONINGNODE_HPP
#define PLATOONING_PLATOONINGNODE_HPP

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <unordered_map>

#include "Topics.hpp"
#include "MessageTypes.hpp"
#include "ServiceTypes.hpp"
#include "Services.hpp"

namespace platooning {

enum class PlatooningModeEnum {
	IDLE,
	RUNNING,
	CREATING
};

enum class PlatoonRoleEnum {
	NONE,
	LV,
	FV
};

std::string to_string( const PlatooningModeEnum& e ) {
	switch ( e ) {
		case PlatooningModeEnum::CREATING:
			return "CREATING";
		case PlatooningModeEnum::IDLE:
			return "IDLE";
		case PlatooningModeEnum::RUNNING:
			return "RUNNING";
	}

	return "";
}

std::string to_string( const PlatoonRoleEnum& e ) {
	switch ( e ) {
		case PlatoonRoleEnum::NONE:
			return "NONE";
		case PlatoonRoleEnum::LV:
			return "LV";
		case PlatoonRoleEnum::FV:
			return "FV";
	}

	return "";
}

class PlatooningState {
  private:
	ros::NodeHandle nh_;
	PlatoonRoleEnum role_;
	PlatooningModeEnum current_mode_;
	std::vector<uint32_t> members_;
	uint32_t vehicle_id_;
	uint32_t platoon_id_;
	uint32_t leader_id_;
	float platoon_speed_;
	float platoon_distance_;

	ros::Publisher pub_platooning_state_;

  public:

	PlatooningState();

	virtual void reset();
	void publish_state();

	virtual void set_role( const PlatoonRoleEnum& );
	PlatoonRoleEnum get_role();

	void set_mode( const PlatooningModeEnum& );
	PlatooningModeEnum get_mode();

	void add_member(const uint32_t&);
	void remove_member(const uint32_t&);
	bool is_member(const uint32_t&);
	bool has_members();
	size_t member_count();
	void set_members(std::vector<uint32_t>::const_iterator iterator, std::vector<uint32_t>::const_iterator normal_iterator);
	std::vector<uint32_t> get_members();

	void set_vehicle_id( const uint32_t&);
	uint32_t get_vehicle_id();

	void set_platoon_id( const uint32_t&);
	uint32_t get_platoon_id();

	void set_leader_id(const uint32_t &vehicle_id);
	uint32_t get_leader_id();

	void set_platoon_speed(const float &d);
	float get_platoon_speed();

	void set_platoon_distance(const float& d);
	float get_platoon_distance();

};

class Platooning : public nodelet::Nodelet, private PlatooningState {

  public:
	void onInit();
	Platooning();
	~Platooning();

  private:
	ros::NodeHandle nh_;
	std::string name_ = "Platooning";

	boost::thread_group thread_pool_;

	ros::ServiceServer service_;

	ros::Subscriber sub_fv_leave_;
	ros::Subscriber sub_lv_accept_;
	ros::Subscriber sub_lv_reject_;
	ros::Subscriber sub_fv_request_;
	ros::Subscriber sub_lv_broadcast_;
	ros::Subscriber sub_fv_heartbeat_;
	ros::Subscriber sub_platooning_toggle_;

	ros::Publisher pub_fv_leave_;
	ros::Publisher pub_lv_accept_;
	ros::Publisher pub_lv_reject_;
	ros::Publisher pub_fv_request_;
	ros::Publisher pub_lv_broadcast_;
	ros::Publisher pub_fv_heartbeat_;
	ros::Publisher pub_platooning_state_;

	//heartbeat timer
	boost::posix_time::milliseconds HEARTBEAT_FREQ = boost::posix_time::milliseconds(200);
	boost::posix_time::milliseconds HEARTBEAT_TIMEOUT = boost::posix_time::milliseconds(1000);
	boost::posix_time::milliseconds BROADCAST_FREQ = boost::posix_time::milliseconds(50);
	boost::posix_time::milliseconds BROADCAST_TIMEOUT = boost::posix_time::milliseconds(1000);
	boost::asio::io_service io_service_;
	boost::asio::io_service::work io_worker_;
	boost::asio::deadline_timer fv_heartbeat_sender_;
	boost::asio::deadline_timer fv_heartbeat_checker_;
	boost::asio::deadline_timer lv_broadcast_sender_;
	boost::asio::deadline_timer lv_broadcast_checker_;

	//follower list and timeouts;
	std::unordered_map<uint32_t, boost::chrono::system_clock::time_point> fv_heartbeat_timeout_tracker_;
	std::pair<uint32_t, boost::chrono::system_clock::time_point> lv_broadcast_timeout_tracker_;

	void hndl_msg_fv_leave(const platooning::fv_leave &msg);
	void hndl_msg_lv_accept(const platooning::lv_accept &msg);
	void hndl_msg_lv_reject(const platooning::lv_reject &msg);
	void hndl_msg_fv_request(const platooning::fv_request &msg);
	void hndl_msg_lv_broadcast(const platooning::lv_broadcast &msg);
	void hndl_msg_fv_heartbeat(const platooning::fv_heartbeat &msg);
	void hndl_msg_platooning_toggle(const platooning::platooningToggle &msg);

	void hndl_timeout_fv_heartbeat(const boost::system::error_code &e);
	void hndl_timeout_lv_broadcast(const boost::system::error_code &e);

	void send_fv_heartbeat(const boost::system::error_code &e);
	void send_lv_broadcast(const boost::system::error_code &e);

	void reset() override ;
	void set_role( const PlatoonRoleEnum& r) override ;
	bool provide_vehicle_id(getVehicleId::Request &res, getVehicleId::Response& );
	void shutdown_pubs_and_subs();
	void update_state(const lv_broadcast &bc);
	uint32_t get_vehicle_id_param();
};
}

#endif

