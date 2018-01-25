//
// Created by stepo on 1/22/18.
//

#ifndef PLATOONING_MODULETEST_HPP
#define PLATOONING_MODULETEST_HPP

#include <nodelet/nodelet.h>
#include <list>
#include <chrono>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <platooning/registerTestcases.h>

class Moduletest : public nodelet::Nodelet {

protected:

  void register_testcases( std::list<std::string> testcases_to_register );

  ros::NodeHandle nh_; /**< Some documentation for the member nh_. */

  std::string name_ = "Moduletest";

};


#endif //PLATOONING_MODULETEST_HPP
