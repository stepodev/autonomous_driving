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
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "platooning/RemoteControl.hpp"
#include "std_msgs/String.h"


namespace platooning
{

/**
 * @brief Nodelet-wrapper of the RemoteController class
 */
    class RemoteControlNodelet : public nodelet::Nodelet
    {
    public:
        RemoteControlNodelet(){};
        ~RemoteControlNodelet(){}

        /**
         * @brief Initialise the nodelet
         *
         * This function is called, when the nodelet manager loads the nodelet.
         */
        virtual void onInit()
        {
            ros::NodeHandle nh = this->getPrivateNodeHandle();

            // resolve node(let) name
            std::string name = nh.getUnresolvedNamespace();
            int pos = name.find_last_of('/');
            name = name.substr(pos + 1);

            NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");

            // Initialises the controller
            if (remoteControl->init())
            {
                NODELET_INFO_STREAM("Nodelet initialised. [" << name << "]");
            }
            else
            {
                NODELET_ERROR_STREAM("Couldn't initialise nodelet! Please restart. [" << name << "]");
            }
        }
    private:
        boost::shared_ptr<RemoteControl> remoteControl;
    };

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::RemoteControlNodelet,
        nodelet::Nodelet);
// %EndTag(FULLTEXT)%