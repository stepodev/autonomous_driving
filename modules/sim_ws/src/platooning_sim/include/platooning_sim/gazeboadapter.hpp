//
// Created by stepo on 3/4/18.
//

#ifndef CAR_DEMO_GAZEBOADAPTER_HPP
#define CAR_DEMO_GAZEBOADAPTER_HPP

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>

#include "platooning/MessageTypes.hpp"
#include "platooning/Topics.hpp"
#include "platooning/UdpServer.hpp"
#include "prius_msgs/Control.h"
#include "prius_msgs/Speed.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Range.h"

const boost::posix_time::milliseconds BROADCAST_FREQ(50);
const boost::posix_time::milliseconds SOURCECHECK_FREQ(1000);

#define P1_RANGE 1
#define P2_RANGE 2
#define P1_SPEED 4
#define P2_SPEED 8

namespace platooning_sim {

class gazeboadapter : public nodelet::Nodelet {
  public:
	void onInit();

	gazeboadapter();

	~gazeboadapter();

  private:
	ros::NodeHandle nh_; /**< Some documentation for the member nh_. */
	std::string name_ = "gazeboadapter";
	platooning::gazupdate p1gazupdate;
	platooning::gazupdate p2gazupdate;
	platooning::gazupdate p3gazupdate;

	unsigned short src_flags_ = 0;

	boost::thread_group thread_pool_;
	std::unique_ptr<UdpServer> server_;

	ros::Publisher pub_p1_control_;
	ros::Publisher pub_p2_control_;
	ros::Publisher pub_p3_control_;

	ros::Subscriber sub_p1_speed_;
	ros::Subscriber sub_p2_speed_;
	ros::Subscriber sub_p3_speed_;

	ros::Subscriber sub_p1_sonar_front_;
	ros::Subscriber sub_p2_sonar_front_;
	ros::Subscriber sub_p3_sonar_front_;

	ros::Subscriber sub_p1_camera_front_;
	ros::Subscriber sub_p2_camera_front_;
	ros::Subscriber sub_p3_camera_front_;

	boost::asio::io_service io_service_;
	boost::asio::io_service::work io_worker_;
	boost::asio::deadline_timer gazupdate_send_timer_;
	boost::asio::deadline_timer detect_dead_datasource_timer;

	void hndl_recv_udp(boost::shared_ptr<std::pair<std::string, uint32_t>> packet);

	void hndl_p1_speed( const prius_msgs::Speed& msg);
	void hndl_p1_sonar( const sensor_msgs::Range& msg );
	void hndl_p1_camera(const sensor_msgs::Image& msg );

	void hndl_p2_speed( const prius_msgs::Speed& msg);
	void hndl_p2_sonar( const sensor_msgs::Range& msg );
	void hndl_p2_camera(const sensor_msgs::Image& msg );

	void hndl_p3_speed( const prius_msgs::Speed& msg);
	void hndl_p3_sonar( const sensor_msgs::Range& msg );
	void hndl_p3_camera(const sensor_msgs::Image& msg );

	void send_gazupdate(const boost::system::error_code &e);
	void process_stmsim(const platooning::stmupdate &stmupdate_);
	void check_dead_datasrc(const boost::system::error_code &e);
};

}



#endif //CAR_DEMO_GAZEBOADAPTER_HPP
