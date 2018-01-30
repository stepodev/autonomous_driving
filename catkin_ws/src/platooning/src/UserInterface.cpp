#include "UserInterface.hpp"

namespace platooning
{
	UserInterface::UserInterface() {};
	UserInterface::~UserInterface() {};

	void UserInterface::onInit() {
		userInterfaceSubsc = nh_.subscribe("UserInterface", 10, &UserInterface::msg_callback, this);
		userInterfacePub = nh_.advertise<userInterface>("platooningProtocolOut", 10);
	};

	void UserInterface::msg_callback(const platooning::userInterface& subscMsg)
	{
		std::string lv = "Yes";
		std::string fv = "Yes";
		if (subscMsg.leading_vehicle == 0)
			lv = "No";
		if (subscMsg.following_vehicle == 0)
			fv = "No";
		std::cout << "\r" << "LV: " << lv << " | FV: " << fv << " | IPD: " << subscMsg.inner_platoon_distance << " | PSpeed: " << subscMsg.platoon_speed << " | VSpeed: " << subscMsg.speed << " " << std::flush;

		userInterfacePub.publish(subscMsg);
	}
}

PLUGINLIB_EXPORT_CLASS(platooning::UserInterface, nodelet::Nodelet);
