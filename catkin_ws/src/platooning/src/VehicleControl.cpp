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
#include <Topics.hpp>
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
	    subscriber = nh_.subscribe(topics::ACCELERATION, 10, &VehicleControl::accelerationCallback, this);
	    subscriber = nh_.subscribe(topics::STEERINGANGLE, 10, &VehicleControl::steeringAngleCallback, this);
        publisher = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    };


/*****************************************************************************
** Handlers
*****************************************************************************/


    /*void VehicleControl::poseCallback(const turtlesim::PoseConstPtr &pose) {
        NODELET_DEBUG("Getting turtlesim/pose message");
        g_pose = pose;
    }*/

    void VehicleControl::steeringAngleCallback(platooning::steeringAngle angle){
	    NODELET_DEBUG("Getting angle message");
	    NODELET_INFO("Getting angle message");
	    g_steeringAngle = angle;
	    commandTurtle();
    }

    void VehicleControl::accelerationCallback(platooning::acceleration accel) {
        NODELET_DEBUG("Getting accel message");
        NODELET_INFO("Getting accel message");
        g_acceleration.accelleration += accel.accelleration;
        commandTurtle();
    }

    void VehicleControl::commandTurtle() {
        NODELET_DEBUG("Publishing turtlesim/twist message");
        NODELET_INFO("Publishing turtlesim/twist message");
        twist.linear.x = g_acceleration.accelleration;
        twist.angular.z = g_steeringAngle.steering_angle;
        publisher.publish(twist);
    }


} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::VehicleControl, nodelet::Nodelet);
// %EndTag(FULLTEXT)%