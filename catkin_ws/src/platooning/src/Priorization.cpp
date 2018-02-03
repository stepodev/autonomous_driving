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
#include "../include/Priorization.hpp"

namespace platooning {


/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

  Priorization::Priorization() {};



/*****************************************************************************
** Destructors
*****************************************************************************/

  Priorization::~Priorization() {};


/*****************************************************************************
** Initializers
*****************************************************************************/

  /**
  * Set-up necessary publishers/subscribers
  * @return true, if successful
  */
  void Priorization::onInit() {

    //subscribers of protocol nodelet
    templateSubscriber = nh_.subscribe("templateMsg", 10,
                                                  &Priorization::templateTopicHandler, this);

    //publisher of forced driving vector
    templatePublisher = nh_.advertise< platooning::templateMsg >("commands/templateMsg", 10);


  };


/*****************************************************************************
** Handlers
*****************************************************************************/

  /*
   * handling an event and publishing something
   */
  void Priorization::templateTopicHandler(const platooning::templateMsg msg) {

    NODELET_DEBUG("handling a template");

    if( msg.templatebool || !msg.templatebool ) {
      templatePublisher.publish(msg);
    } else {
      NODELET_WARN("warning you of stuff");
    }

  }



} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Priorization, nodelet::Nodelet);
// %EndTag(FULLTEXT)%