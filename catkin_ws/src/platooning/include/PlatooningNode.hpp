#ifndef PLATOONING_PLATOONINGNODE_HPP
#define PLATOONING_PLATOONINGNODE_HPP

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "Topics.hpp"
#include "MessageTypes.hpp"
#include "platooning/platooningState.h"

namespace platooning {

	class PlatooningNode: public nodelet::Nodelet {
		public:
			void onInit();
			PlatooningNode();
			~PlatooningNode();

		private:
			ros::NodeHandle nh_;

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

			void hndl_fv_leave(const platooning::fv_leave& msg);
			void hndl_lv_accept(const platooning::lv_accept& msg);
			void hndl_lv_reject(const platooning::lv_reject& msg);
			void hndl_fv_request(const platooning::fv_request& msg);
			void hndl_lv_broadcast(const platooning::lv_broadcast& msg);
			void hndl_fv_heartbeat(const platooning::fv_heartbeat& msg);
			void hndl_platooning_toggle(const platooning::platooningToggle& msg);
	};
}

#endif

