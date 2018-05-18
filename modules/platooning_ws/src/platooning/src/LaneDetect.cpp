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
#include "platooning/LaneDetect.hpp"

namespace platooning {


/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

  LaneDetect::LaneDetect() {};



/*****************************************************************************
** Destructors
*****************************************************************************/

  LaneDetect::~LaneDetect() {};


/*****************************************************************************
** Initializers
*****************************************************************************/

  /**
  * Set-up necessary publishers/subscribers
  * @return true, if successful
  */
  void LaneDetect::onInit() {

    //subscribers of protocol nodelet
    templateSubscriber = nh_.subscribe("templateMsg", 10,
                                                  &LaneDetect::templateTopicHandler, this);

    //publisher of forced driving vector
    templatePublisher = nh_.advertise< platooning::templateMsg >("commands/templateMsg", 10);


  };


/*****************************************************************************
** Handlers
*****************************************************************************/

  /*
   * handling an event and publishing something
   */
  void LaneDetect::templateTopicHandler(const platooning::templateMsg msg) {

    NODELET_DEBUG("handling a template");

    if( msg.templatebool || !msg.templatebool ) {
      templatePublisher.publish(msg);
    } else {
      NODELET_WARN("warning you of stuff");
    }

  }



} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::LaneDetect, nodelet::Nodelet);
// %EndTag(FULLTEXT)%