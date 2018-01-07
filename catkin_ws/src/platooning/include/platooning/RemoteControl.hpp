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
 * messages starting/stopping remotecontrol mode and driving vector
 */
    class RemoteControl : public nodelet::Nodelet
    {
    public:
        virtual bool onInit();

        RemoteControl(ros::NodeHandle& nh, std::string& name);
        ~RemoteControl();

    private:
        ros::NodeHandle nh_;
        std::string name_;
        ros::Subscriber enableRemoteControlSubscriber, disableRemoteControlSubscriber;

        ros::Publisher remoteDrivingVectorPublisher, remoteStartPublisher, remoteStopPublisher;

        bool remoteDrivingEnabled = false;

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


} // namespace platooning
// %EndTag(FULLTEXT)%

#endif //PLATOONING_REMOTECONTROL_HPP
