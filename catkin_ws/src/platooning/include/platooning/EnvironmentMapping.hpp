/**
 * @file include/platooning/EnvironmentMapping.hpp
 *
 * @brief Environment Mapping nodelet for platooning
 *
 * Takes optical data and provides data for the distance of objects
 *
 * @author jonas
 **/

#ifndef PLATOONING_ENVIRONMENTMAPPING_HPP
#define PLATOONING_ENVIRONMENTMAPPING_HPP

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "ros/ros.h"
#include <std_msgs/Float64.h>

namespace platooning {

  class EnvironmentMapping : public nodelet::Nodelet {
  public:
    virtual void onInit();

  private:
    ros::NodeHandle nh;
    ros::Publisher pub;
  };
}

#endif //PLATOONING_ENVIRONMENTMAPPING_HPP
