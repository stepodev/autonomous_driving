#ifndef LISTENER_ROS_NODELET_H
#define LISTENER_ROS_NODELET_H

#include "ros/ros.h"
#include <string>
#include "nodelet/nodelet.h"

class ListenerNodelet : public nodelet::Nodelet
  {
  public:
    ListenerNodelet();
    void chatterCallback(const std_msgs::String::ConstPtr& msg);

  private:
    virtual void onInit();

    ros::NodeHandle nh_;
    ros::Subscriber sub;
  };

}  // ListenerNodelet

#endif // LISTENER_ROS_NODELET_H