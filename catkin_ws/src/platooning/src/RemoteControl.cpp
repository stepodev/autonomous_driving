//
// Created by stepo on 12/16/17.
//


/**
 * @file /platooning/src/platooning.cpp
 *
 * @brief Nodelet implementation of RemoteContol
 *
 * @author stepo
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include "RemoteControl.hpp"


namespace platooning {

/**
 * @brief Remotecontrol Nodelet
 */

  RemoteControl::RemoteControl(ros::NodeHandle &nh, std::string &name) : nh_(nh), name_(name) {};

  RemoteControl::RemoteControl() {};

  RemoteControl::~RemoteControl() {};

  /**
  * Set-up necessary publishers/subscribers
  * @return true, if successful
  */
  void RemoteControl::onInit() {

    //subscribers of protocol nodelet
    enableRemoteControlSubscriber = nh_.subscribe("enableRemoteControlMsg", 10,
                                                  &RemoteControl::updateRemoteControlStatusHandler, this);

    remoteDrivingVectorSubscriber = nh_.subscribe("remoteDrivingVector",10,
                                                  &RemoteControl::remoteDrivingVectorHandler, this);


    //publisher of forced driving vector
    forcedDrivingVectorPublisher = nh_.advertise< platooning::forcedDrivingVector >("commands/forcedDrivingVector", 10);


  };

  /*
   * attach source to forcedrivingvector and publish
   */
  void RemoteControl::remoteDrivingVectorHandler(const platooning::remoteDrivingVector msg) {

    if( !remoteDrivingEnabled ) {
      NODELET_WARN_ONCE("received remotecontrolvector while remotecontrol disabled");
      return;
    }

    platooning::forcedDrivingVector outmsg;
    outmsg.speed = msg.speed;
    outmsg.steering_angle = msg.steering_angle;
    outmsg.source = "remotecontrol";

    forcedDrivingVectorPublisher.publish(outmsg);

  };

  /*
   * test errorcases and if all is fine, enable remotecontrol
   */
  void RemoteControl::updateRemoteControlStatusHandler(const platooning::enableRemoteControl msg) {

    if (!remoteDrivingEnabled && msg.enableRemoteControl) {
      NODELET_DEBUG("Enabling Remotecontrol");
      remoteDrivingEnabled = true;
    }

    if (remoteDrivingEnabled && !msg.enableRemoteControl) {
      NODELET_DEBUG("Enabling Remotecontrol");
      remoteDrivingEnabled = false;
    }

    if (remoteDrivingEnabled && msg.enableRemoteControl) {
      NODELET_WARN_ONCE("trying to enable remotecontrol while already enabled");
    }

    if (!remoteDrivingEnabled && !msg.enableRemoteControl) {
      NODELET_WARN_ONCE("trying to disable remotecontrol while already enabled");
    }
  };


} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::RemoteControl, nodelet::Nodelet);
// %EndTag(FULLTEXT)%