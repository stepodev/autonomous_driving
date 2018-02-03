//
// Created by stepo on 12/16/17.
//

/**
 * @file doxygen_c.h
 * @author My Self
 * @date 9 Sep 2012
 * @brief File containing example of doxygen usage for quick reference.
 *
 * Here typically goes a more extensive explanation of what the header
 * defines. Doxygens tags are words preceeded by either a backslash @\
 * or by an at symbol @@.
 * @see http://www.stack.nl/~dimitri/doxygen/docblocks.html
 * @see http://www.stack.nl/~dimitri/doxygen/commands.html
 */

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PLATOONING_PROTOCOL_HPP
#define PLATOONING_PROTOCOL_HPP

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
 * @brief Example showing how to document a function with Doxygen.
 *
 * Description of what the function does. This part may refer to the parameters
 * of the function, like @p param1 or @p param2. A word of code can also be
 * inserted like @c this which is equivalent to <tt>this</tt> and can be useful
 * to say that the function returns a @c void or an @c int. If you want to have
 * more than one word in typewriter font, then just use @<tt@>.
 * We can also include text verbatim,
 * @verbatim like this@endverbatim
 * Sometimes it is also convenient to include an example of usage:
 * @code
 * BoxStruct *out = Box_The_Function_Name(param1, param2);
 * printf("something...\n");
 * @endcode
 * Or,
 * @code{.py}
 * pyval = python_func(arg1, arg2)
 * print pyval
 * @endcode
 * when the language is not the one used in the current source file (but
 * <b>be careful</b> as this may be supported only by recent versions
 * of Doxygen). By the way, <b>this is how you write bold text</b> or,
 * if it is just one word, then you can just do @b this.
 * @param param1 Description of the first parameter of the function.
 * @param param2 The second one, which follows @p param1.
 * @return Describe what the function returns.
 * @see Box_The_Second_Function
 * @see Box_The_Last_One
 * @see http://website/
 * @note Something to note.
 * @warning Warning.
 */

  class MessageDistribution : public nodelet::Nodelet {
  public:
    MessageDistribution();

    ~MessageDistribution();

    void onInit();

  private:
    ros::NodeHandle nh_; /**< Some documentation for the member nh_. */
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
     * @param msg json payload
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

#endif //PLATOONING_PROTOCOL_HPP
