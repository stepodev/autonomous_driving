/**
 * @file include/platooning/MessageDistribution.hpp
 *
 * @brief Header of MessageDistribution class
 *
 * @date 22.03.2018
 *
 * @author stepo
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PLATOONING_MESSAGEDISTRIBUTION_HPP
#define PLATOONING_MESSAGEDISTRIBUTION_HPP

/*****************************************************************************
** DEFINES
*****************************************************************************/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <boost/uuid/uuid.hpp>
#include "MessageTypes.hpp"
#include "Topics.hpp"

namespace platooning {

/*****************************************************************************
** Classes
*****************************************************************************/
/**
 * @class MessageDistribution
 *
 * @brief Uses the encode and decode functionality of the MessageTypes class to
 * put messages that need to be sent via radiointerface into the platoonProtocol
 * messagetype. Messages coming from the radiointerface get filtered or unpacked
 * into their proper messagetypes and sent on the appropriate topic.
 */

  class MessageDistribution : public nodelet::Nodelet {
  public:
    MessageDistribution();

    ~MessageDistribution();

    void onInit();

  private:
    ros::NodeHandle nh_;
    std::string name_ = "MessageDistribution";

    ros::Subscriber sub_platooningIn; /**< subscribers to incoming messages from radiointerface. */

    ros::Publisher pub_platooningOut; /**< provides to messages to send via radiointerface. */

    ros::Subscriber sub_ui;

    /** subscribers to platooning messages */
    ros::Subscriber sub_lv_broadcast;
    ros::Subscriber sub_lv_accept;
    ros::Subscriber sub_lv_reject;
    ros::Subscriber sub_fv_heartbeat;
    ros::Subscriber sub_fv_leave;
    ros::Subscriber sub_fv_request;

    /** publishers for platooning messages. */
    ros::Publisher pub_lv_broadcast;
    ros::Publisher pub_lv_accept;
    ros::Publisher pub_lv_reject;
    ros::Publisher pub_fv_heartbeat;
    ros::Publisher pub_fv_leave;
    ros::Publisher pub_fv_request;

    ros::Publisher pub_platooning_toggle;
    ros::Publisher pub_remotecontrol_toggle;
    ros::Publisher pub_remotecontrol_input;


    /**
     * @brief receives json payloads from radiointerface, transforms them to messages
     * @param msg message to be put into platoonProtocol messagetype
     */
    void hndl_platooningIn(const platooning::platoonProtocol &msg);

    void hndl_lv_broadcast(const lv_broadcast &msg);

    void hndl_lv_accept(const platooning::lv_accept &msg);

    void hndl_lv_reject(const platooning::lv_reject &msg);

    void hndl_fv_heartbeat(const platooning::fv_heartbeat &msg);

    void hndl_fv_leave(const platooning::fv_leave &msg);

    void hndl_fv_request(const platooning::fv_request &msg);

    void hndl_ui(const platooning::userInterface &msg);
  };


} // namespace platooning

#endif //PLATOONING_MESSAGEDISTRIBUTION_HPP
