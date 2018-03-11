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

const boost::posix_time::milliseconds HEARTBEAT_FREQ(200);
const boost::posix_time::milliseconds HEARTBEAT_TIMEOUT(1000);
const boost::posix_time::milliseconds BROADCAST_FREQ(50);
const boost::posix_time::milliseconds BROADCAST_TIMEOUT(1000);

enum PlatooningStateEnum {
	IDLE,
	RUNNING,
	CREATING
};

enum PlatoonRole {
	NONE,
	LV,
	FV
};

const std::string PlatooningStateStrings[] = {
	"IDLE", "RUNNING", "CREATING"
};

const std::string PlatoonRoleStrings[] = {
	"NONE", "LV", "FV"
};

class Platooning : public nodelet::Nodelet {
  public:
	void onInit();
	Platooning();
	~Platooning();

  private:
	ros::NodeHandle nh_;
	std::string name_ = "Platooning";

	boost::thread_group thread_pool_;

	ros::ServiceServer service_;

	ros::Subscriber sub_fv_leave;
	ros::Subscriber sub_lv_accept;
	ros::Subscriber sub_lv_reject;
	ros::Subscriber sub_fv_request;
	ros::Subscriber sub_lv_broadcast;
	ros::Subscriber sub_fv_heartbeat;
	ros::Subscriber sub_platooning_toggle;

	ros::Publisher pub_fv_leave;
	ros::Publisher pub_lv_accept;
	ros::Publisher pub_lv_reject;
	ros::Publisher pub_fv_request;
	ros::Publisher pub_lv_broadcast;
	ros::Publisher pub_fv_heartbeat;
	ros::Publisher pub_platooning_state;

	uint32_t vehicle_id_;

	PlatoonRole platoon_role_ = NONE;

	//state vars
	PlatooningStateEnum platooning_state_ = IDLE;
	platooningState platoon_data_;

	//heartbeat timer
	boost::asio::io_service io_service_;
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

	void update_platoonState(lv_broadcast bc);

	void reset_state();
	void get_vehicle_id_param();

	bool provide_vehicle_id(getVehicleId::Request &res, getVehicleId::Response& );
};
}

#endif

