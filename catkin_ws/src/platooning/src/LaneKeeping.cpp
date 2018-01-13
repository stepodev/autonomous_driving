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
#include "LaneKeeping.hpp"

namespace platooning {


/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

  LaneKeeping::LaneKeeping() {};



/*****************************************************************************
** Destructors
*****************************************************************************/

  LaneKeeping::~LaneKeeping() {};


/*****************************************************************************
** Initializers
*****************************************************************************/

  /**
  * Set-up necessary publishers/subscribers
  * @return true, if successful
  */
  void LaneKeeping::onInit() {

    //subscribers of protocol nodelet
    templateSubscriber = nh_.subscribe("templateMsg", 10,
                                                  &LaneKeeping::templateTopicHandler, this);

    //publisher of forced driving vector
    templatePublisher = nh_.advertise< platooning::templateMsg >("commands/templateMsg", 10);


  };


/*****************************************************************************
** Handlers
*****************************************************************************/

  /*
   * handling an event and publishing something
   */
  void LaneKeeping::templateTopicHandler(const platooning::templateMsg msg) {

    NODELET_DEBUG("handling a template");

    if( msg.templatebool || !msg.templatebool ) {
      templatePublisher.publish(msg);
    } else {
      NODELET_WARN("warning you of stuff");
    }

  }



} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::LaneKeeping, nodelet::Nodelet);
// %EndTag(FULLTEXT)%