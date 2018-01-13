//
// Created by stepo on 12/16/17.
//


/**
 * @file RemoteControl.hpp
 * @author stepo
 *
 * @brief Remotecontroller nodelet for platooning
 *
 * Nodelet based remotecontoller. Takes remotecontrolmessages, remoteDrivingVector. publishes forcedDrviningVector
 *
 *
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PLATOONING_REMOTECONTROL_HPP
#define PLATOONING_REMOTECONTROL_HPP

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "platooning/remoteDrivingVector.h"
#include "platooning/enableRemoteControl.h"
#include "platooning/forcedDrivingVector.h"

namespace platooning {

/**
 * @ brief Remotecontrol nodelet
 *
 * Remotecontrol nodelet that receives messages from the radio controller
 * starting/stopping remotecontrol mode and provides forcedRemoteDrivingVector
 */
  class RemoteControl : public nodelet::Nodelet {
  public:
    virtual void onInit();

    RemoteControl(ros::NodeHandle &nh, std::string &name);

    RemoteControl();

    ~RemoteControl();

  private:
    ros::NodeHandle nh_;
    std::string name_;
    ros::Subscriber enableRemoteControlSubscriber;
    ros::Subscriber remoteDrivingVectorSubscriber;
    ros::Publisher forcedDrivingVectorPublisher;

    bool remoteDrivingEnabled = false;

    /**
     * @brief enables or disables remotecontrol
     * @param msg incoming topic message
     */
    void updateRemoteControlStatusHandler(const platooning::enableRemoteControl msg);

    /**
     * @brief handles remotedrivingvector
     * @param msg incoming topic message
     */
    void remoteDrivingVectorHandler(const platooning::remoteDrivingVector msg);

  };


} // namespace platooning

#endif //PLATOONING_REMOTECONTROL_HPP
