#include "ros/ros.h"
#include "std_msgs/String.h"
#include "talker.hpp"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

Talker::Talker() {}

Talker::onInit() {
  nh_ = getNodeHandle();

  chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);

}
