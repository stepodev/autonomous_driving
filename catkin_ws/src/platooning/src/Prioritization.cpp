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
#include "Prioritization.hpp"

namespace platooning {


/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

    Prioritization::Prioritization() {};

/*****************************************************************************
** Destructors
*****************************************************************************/
    Prioritization::~Prioritization() {};

/*****************************************************************************
** Structures
*****************************************************************************/

    struct drivingVector {
        int velocity = 0;
        float steeringAngle = 0;
        int distanceToObject = 0;
    } currentDrivingVector;

    /*****************************************************************************
** Class Variables
*****************************************************************************/

    bool remoteControlMode = false;

/*****************************************************************************
** Initializers
*****************************************************************************/

    /**
    * Set-up necessary publishers/subscribers
    * @return true, if successful
    */
    void Prioritization::onInit() {

        //Subscriptions
        environtmentMappingSubscriber = nh_.subscribe("distanceToObject", 10, environmentMappingHandler,
                                                      this);
        laneKeepingSubscriber = nh_.subscribe("steeringAngle", 10, laneKeepingHandler, this);
        //platooningSubscriber = nh_.subscribe("templateMsg", 10, platooningHandler, this);
        remoteControlSubscriber = nh_.subscribe("remoteDrivingVector/remoteControlOn-Off", 10, remoteControlHandler, this);

        //Publications
        platooningPublisher = nh_.advertise<int>("prioritizationDrivingVector", 10);
        userInterfacePublisher = nh_.advertise<int>("stateData", 10);
        vehicleControlPublisher = nh_.advertise<int>("prioritizationDrivingVector", 10);
    };


/*****************************************************************************
** Handlers
*****************************************************************************/

    /*
     * handling an event and publishing something
     */
    void Prioritization::environmentMappingHandler(int distanceToObject) {
        if (distanceToObject < 10) {
            currentDrivingVector.velocity = 0;
        } else {

        }
        //vehicleControlPublisher.publish()
    }

    void Prioritization::laneKeepingHandler(float steeringAngle) {
        if (remoteControlMode == false ) {
            currentDrivingVector.steeringAngle = steeringAngle;
        }
        //vehicleControlPublisher.publish();
    }

    void Prioritization::remoteControlHandler(int remoteDrivingVector, bool remoteControlOn) {
        if (remoteControlOn == true) {
            remoteControlMode = true;
            //Set currentDrivingVector to the received remoteDrivingVector
        } else {
            remoteControlMode = false;
        }
    }

    void Prioritization::remoteControlHandler(platooning::drivingVector msg) {

    }

    void Prioritization::platooningHandler(int velocity, int innerPlatoonDistance) {
        //Geschwindigkeit erhöhen, solange Distanz zu klein?
        if(currentDrivingVector.distanceToObject > innerPlatoonDistance) {
            currentDrivingVector.velocity += 1;
        } else if (currentDrivingVector.distanceToObject > innerPlatoonDistance) {
            currentDrivingVector.velocity -= 1;
        } else {
            currentDrivingVector.velocity = velocity;
        }

        //platooningPublisher.publish();
    }


} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Prioritization, nodelet::Nodelet);

