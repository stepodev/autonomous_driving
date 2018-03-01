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
#include "../include/VehicleControl.hpp"

namespace platooning {


/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

    VehicleControl::VehicleControl() {};


/*****************************************************************************
** Destructors
*****************************************************************************/

    VehicleControl::~VehicleControl() {};


/*****************************************************************************
** Initializers
*****************************************************************************/

    /**
    * Set-up necessary publishers/subscribers
    * @return true, if successful
    */
    void VehicleControl::onInit() {
        //subscriber_pose = nh_.subscribe("turtle1/pose", 10, poseCallback);
        subscriber = nh_.subscribe("prioritisationDrivingVector", 10, &VehicleControl::prioritisationDrivingVectorCallback, this);
        publisher = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    };


/*****************************************************************************
** Handlers
*****************************************************************************/


    /*void VehicleControl::poseCallback(const turtlesim::PoseConstPtr &pose) {
        NODELET_DEBUG("Getting turtlesim/pose message");
        g_pose = pose;
    }*/

    void VehicleControl::prioritisationDrivingVectorCallback(platooning::prioritisationDrivingVector prioDrivingVector) {
        NODELET_DEBUG("Getting prioritisationDrivingVector message");
        NODELET_INFO("Getting prioritisationDrivingVector message");
        g_prioDrivingVector = prioDrivingVector;
        commandTurtle();
    }

    void VehicleControl::commandTurtle() {
        NODELET_DEBUG("Publishing turtlesim/twist message");
        NODELET_INFO("Publishing turtlesim/twist message");
        twist.linear.x = g_prioDrivingVector.speed;
        twist.angular.z = g_prioDrivingVector.steering_angle;
        publisher.publish(twist);
    }


} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::VehicleControl, nodelet::Nodelet);
// %EndTag(FULLTEXT)%