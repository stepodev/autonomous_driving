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
#include "Template.hpp"

namespace platooning {


/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

  Template::Template() {};



/*****************************************************************************
** Destructors
*****************************************************************************/

  Template::~Template() {};


/*****************************************************************************
** Initializers
*****************************************************************************/

  /**
  * Set-up necessary publishers/subscribers
  */
  void Template::onInit() {

    //subscribers of protocol nodelet
    sub_templateTopic = nh_.subscribe("templateTopic", 10,
                                                  &Template::hndl_templateTopic, this);

    //publisher of forced driving vector

    pub_templateTopic = nh_.advertise< platooning::templateMsg >("templateMsg", 10);


  };


/*****************************************************************************
** Handlers
*****************************************************************************/

  /*
   * handling an event and publishing something
   */
  void Template::hndl_templateTopic(const platooning::templateMsg msg) {

    NODELET_DEBUG("handling a template");

    if( msg.templatebool || !msg.templatebool ) {
      pub_templateTopic.publish(msg);
    } else {
      NODELET_WARN("warning you of stuff");
    }

  }



} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Template, nodelet::Nodelet);
// %EndTag(FULLTEXT)%