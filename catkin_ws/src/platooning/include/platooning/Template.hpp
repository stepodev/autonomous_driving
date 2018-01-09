//
// Created by stepo on 12/16/17.
//


/**
 * @file include/platooning/RemoteControl.hpp
 *
 * @brief Remotecontroller nodelet for platooning
 *
 * Nodelet based remotecontoller. Takes remotecontrolmessages, remoteDrivingVector. publishes forcedDrviningVector
 *
 * @author stepo
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PLATOONING_TEMPLATE_HPP
#define PLATOONING_TEMPLATE_HPP

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "platooning/templateMsg.h" //includes topic aka message

namespace platooning {

/**
 * @ brief Template nodelet
 *
 * to achieve X does Y
 */
  class Template : public nodelet::Nodelet {
  public:
    virtual void onInit();

    Template(ros::NodeHandle &nh, std::string &name);

    Template();

    ~Template();

  private:
    ros::NodeHandle nh_;
    std::string name_;
    ros::Subscriber templateSubscriber;
    ros::Publisher templatePublisher;


    /**
     * @brief to achieve X does Y
     * @param msg incoming topic message
     */
    void templateTopicHandler(const platooning::templateMsg msg);

  };


} // namespace platooning
// %EndTag(FULLTEXT)%

#endif //PLATOONING_TEMPLATE_HPP
