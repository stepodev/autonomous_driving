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

#include "gazebo_msgs/SpawnModel.h"
#include "platooning/include/MessageTypes.hpp"
#include "platooning/include/Topics.hpp"
#include "platooning/include/UdpServer.hpp"

namespace platooning_sim {

const uint32_t MAXVEHICLES = 3;

class gazeboadapter : public nodelet::Nodelet {
  public:
	void onInit();

	gazeboadapter();

	~gazeboadapter();

  private:
	ros::NodeHandle nh_; /**< Some documentation for the member nh_. */
	std::string name_ = "gazeboadapter";

	std::array<bool, 3> vehicles_enabled_;

	void hndl_recv_udp(std::pair<std::string, uint32_t> packet);
	ros::Publisher pub_;
	boost::thread_group thread_pool_;
	std::unique_ptr<UdpServer> server_;
};

}



#endif //CAR_DEMO_GAZEBOADAPTER_HPP
