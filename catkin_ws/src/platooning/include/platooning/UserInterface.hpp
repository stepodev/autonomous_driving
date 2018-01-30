/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/
#ifndef PLATOONING_USERINTERFACE_HPP
#define PLATOONING_USERINTERFACE_HPP

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <platooning/userInterface.h>
#include <pluginlib/class_list_macros.h>

namespace platooning {

	class UserInterface : public nodelet::Nodelet {
		public:
			virtual void onInit();
			UserInterface();
			~UserInterface();

		private:
			ros::NodeHandle nh_;
			ros::Publisher userInterfacePub;
			ros::Subscriber userInterfaceSubsc;
			void msg_callback(const platooning::userInterface& subscMsg);
	};
}

#endif
