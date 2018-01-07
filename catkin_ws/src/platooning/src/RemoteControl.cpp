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
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "platooning/RemoteControl.hpp"
#include "std_msgs/String.h"
#include "platooning/remoteDrivingVector.h"


namespace platooning
{

/**
 * @brief Remotecontrol Nodelet
 */

    RemoteControl::RemoteControl(ros::NodeHandle& nh, std::string& name) : nh_(nh), name_(name){};
    RemoteControl::RemoteControl(){};
    RemoteControl::~RemoteControl(){};

    /**
    * Set-up necessary publishers/subscribers
    * @return true, if successful
    */
    void RemoteControl::onInit()
    {
        enableRemoteControlSubscriber = nh_.subscribe("enableRemoteControl", 10, &RemoteControl::enableRC, this);
        disableRemoteControlSubscriber = nh_.subscribe("disableRemoteControl", 10, &RemoteControl::disableRC, this);

        // advertise remoteDrivingVector and start stop
        //TODO: remoteDrivingVectorPublisher = nh_.advertise< platooning_msgs::remoteDrivingVector >("commands/remoteDrivingVector", 10);
        //TODO: remoteStartPublisher = nh_.advertise< platooning_msgs::remoteStart >("commands/remoteStart", 10);
        //TODO: remoteStopPublisher = nh_.advertise< platooning_msgs::remoteStop >("commands/remoteStop", 10);
    };


    void RemoteControl::enableRC(const std_msgs::EmptyConstPtr msg)
    {
        if( !remoteDrivingEnabled ) {
            NODELET_DEBUG("Enabling Remotecontrol");
            remoteDrivingEnabled = true;
        } else {
            NODELET_WARN_ONCE("Remotecontrol enable called while remotecontrol already enabled");
        }

    };

    void RemoteControl::disableRC(const std_msgs::EmptyConstPtr msg)
    {
        if( remoteDrivingEnabled ) {
            NODELET_DEBUG("Disabling Remotecontrol");
            remoteDrivingEnabled = false;
        } else {
            NODELET_WARN_ONCE("Remotecontrol disable called while remotecontrol already disabled");
        }

    };

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::RemoteControl, nodelet::Nodelet);
// %EndTag(FULLTEXT)%