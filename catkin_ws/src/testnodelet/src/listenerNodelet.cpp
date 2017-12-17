#include "ros/ros.h"
#include "std_msgs/String.h"
#include "listener.hpp"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void Listener::chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void Listener::onInit()
{
  nh_ = getNodeHandle();
  sub = n.subscribe("chatter", 1000, chatterCallback);
}
