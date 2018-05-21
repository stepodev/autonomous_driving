//
// Created by Paul Waljaew on 17.01.18.
//

#ifndef ROS1_PLATOON_MASTER_MASTER_HPP
#define ROS1_PLATOON_MASTER_MASTER_HPP

#include <boost/statechart/state_machine.hpp>

#include <ros/ros.h>

#include <nodelet/nodelet.h>

#include <master/states.hpp>

namespace platoon {
namespace master {

//******************************************************************
// VehicleMaster
//******************************************************************

class VehicleMaster : public nodelet::Nodelet {
 public:
  VehicleMaster() = default;
  ~VehicleMaster();

 private:
  virtual void onInit() override final;

  VehicleStatechart stm_;
};

}  // namespace master
}  // namespace platoon

#endif  // ROS1_PLATOON_MASTER_MASTER_HPP
