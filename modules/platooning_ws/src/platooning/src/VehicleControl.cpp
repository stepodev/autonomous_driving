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
#include <platooning/Topics.hpp>
#include "platooning/VehicleControl.hpp"

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
    */
    void VehicleControl::onInit() {

    };


/*****************************************************************************
** Handlers
*****************************************************************************/


} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::VehicleControl, nodelet::Nodelet);
// %EndTag(FULLTEXT)%