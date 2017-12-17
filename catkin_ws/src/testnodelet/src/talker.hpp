#ifndef TALKER_ROS_NODELET_H
#define TALKER_ROS_NODELET_H

#include "ros/ros.h"
#include <string>
#include "nodelet/nodelet.h"

class TalkerNodelet : public nodelet::Nodelet
  {
  public:
    TalkerNodelet();

  private:
    virtual void onInit();

    ros::Publisher chatter_pub;
    ros::NodeHandle nh_;
    ros::Rate loop_rate;
    
  };

}  // TalkerNodelet

#endif // TALKER_ROS_NODELET_H
