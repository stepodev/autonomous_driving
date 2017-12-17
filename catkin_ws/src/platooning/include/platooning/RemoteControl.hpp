//
// Created by stepo on 12/16/17.
//


/**
 * @file /kobuki_controller_tutorial/include/kobuki_controller_tutorial/bump_blink_controller.hpp
 *
 * @brief Remotecontroller nodelet for platooning
 *
 * Nodelet based remotecontoller. Takes radiomessages start, stop, TODO message and publishes remoteDrivingVector.msg
 *
 * @author stepo
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PLATOONING_REMOTECONTROL_HPP
#define PLATOONING_REMOTECONTROL_HPP

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <std_msgs/Empty.h>

namespace platooning
{

/**
 * @ brief Remotecontrol nodelet
 *
 * Remotecontrol nodelet that receives messages from the radio controller and provides TODO remoteDrivingVector.msg and
 * start signal msg
 */
    class RemoteControl
    {
    public:
        RemoteControl(ros::NodeHandle& nh, std::string& name) : nh_(nh), name_(name){};
        ~RemoteControl(){};

        /**
         * Set-up necessary publishers/subscribers
         * @return true, if successful
         */
        bool init()
        {
            enableRemoteControlSubscriber = nh_.subscribe("enable", 10, &RemoteControl::enableRC, this);
            disableRemoteControlSubscriber = nh_.subscribe("disable", 10, &RemoteControl::disableRC, this);

            // advertise remoteDrivingVector and start stop
            //TODO: remoteDrivingVectorPublisher = nh_.advertise< platooning_msgs::remoteDrivingVector >("commands/remoteDrivingVector", 10);
            //TODO: remoteStartPublisher = nh_.advertise< platooning_msgs::remoteStart >("commands/remoteStart", 10);
            //TODO: remoteStopPublisher = nh_.advertise< platooning_msgs::remoteStop >("commands/remoteStop", 10);
            return true;
        };

    private:
        ros::NodeHandle nh_;
        std::string name_;
        ros::Subscriber enableRemoteControlSubscriber, disableRemoteControlSubscriber;

        ros::Publisher remoteDrivingVectorPublisher, remoteStartPublisher, remoteStopPublisher;

        /**
         * @brief ROS logging output for enabling the controller
         * @param msg incoming topic message
         */
        void enableRC(const std_msgs::EmptyConstPtr msg);

        /**
         * @brief ROS logging output for disabling the controller
         * @param msg incoming topic message
         */
        void disableRC(const std_msgs::EmptyConstPtr msg);

    };

    void RemoteControl::enableRC(const std_msgs::EmptyConstPtr msg)
    {

    };

    void RemoteControl::disableRC(const std_msgs::EmptyConstPtr msg)
    {

    };


} // namespace platooning
// %EndTag(FULLTEXT)%

#endif //PLATOONING_REMOTECONTROL_HPP
