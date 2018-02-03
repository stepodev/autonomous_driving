/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/
#ifndef PLATOONING_USERINTERFACE_HPP
#define PLATOONING_USERINTERFACE_HPP

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <platooning/lv_broadcast.h>
#include <platooning/lv_accept.h>
#include <platooning/lv_reject.h>
#include <platooning/fv_request.h>
#include <platooning/fv_leave.h>
#include <platooning/fv_heartbeat.h>
#include <platooning/platooningToggle.h>
#include <platooning/remotecontrolInput.h>
#include <platooning/remotecontrolToggle.h>
#include <platooning/acceleration.h>
#include <platooning/steeringAngle.h>
#include <platooning/vehiclecontrol.h>
#include <platooning/distanceToObj.h>
#include <platooning/speed.h>
#include <platooning/platooningState.h>

#include "platooning/userInterface.h"
#include "Topics.hpp"

namespace platooning {

  class UserInterface : public nodelet::Nodelet {
  public:
    virtual void onInit();

    UserInterface();

    ~UserInterface();

  private:
    ros::NodeHandle nh_;
    std::string name_ = "UserInterface";
    ros::Publisher pub_userinterface_;

    ros::Subscriber sub_in_lv_broadcast_;
    ros::Subscriber sub_in_lv_accept_;
    ros::Subscriber sub_in_lv_reject_;
    ros::Subscriber sub_out_lv_broadcast_;
    ros::Subscriber sub_out_lv_accept_;
    ros::Subscriber sub_out_lv_reject_;
    ros::Subscriber sub_in_fv_request;
    ros::Subscriber sub_in_fv_heartbeat;
    ros::Subscriber sub_in_fv_leave;
    ros::Subscriber sub_out_fv_request;
    ros::Subscriber sub_out_fv_heartbeat;
    ros::Subscriber sub_out_fv_leave;

    ros::Subscriber sub_toggle_remotecontrol;
    ros::Subscriber sub_toggle_platooning;

    ros::Subscriber sub_remotecontrol_input;
    ros::Subscriber sub_speed;
    ros::Subscriber sub_distance_to_obj;
    ros::Subscriber sub_vehiclecontrol;
    ros::Subscriber sub_steeringangle;
    ros::Subscriber sub_acceleration;
    ros::Subscriber sub_platooning_state;

    boost::shared_ptr<platooning::userInterface> ui_msg_;

    bool spin_report_ = true;
    boost::thread report_spinner_thread_;

    void hndl_in_lv_broadcast( const lv_broadcast &msg );
    void hndl_in_lv_accept( const lv_accept &msg );
    void hndl_in_lv_reject( const lv_reject &msg );
    void hndl_in_fv_request( const fv_request &msg );
    void hndl_in_fv_leave( const fv_leave &msg );
    void hndl_in_fv_heartbeat( const fv_heartbeat &msg );
    void hndl_out_lv_broadcast( const lv_broadcast &msg );
    void hndl_out_lv_accept( const lv_accept &msg );
    void hndl_out_lv_reject( const lv_reject &msg );
    void hndl_out_fv_request( const fv_request &msg );
    void hndl_out_fv_leave( const fv_leave &msg );
    void hndl_out_fv_heartbeat( const fv_heartbeat &msg );

    void hndl_remotecontrol_toggle( const remotecontrolToggle &msg );
    void hndl_platooning_toggle( const platooningToggle &msg );
    void hndl_remotecontrol_input( const remotecontrolInput &msg );
    void hndl_distance_to_obj( const distanceToObj &msg );
    void hndl_speed( const speed &msg );
    void hndl_vehicle_control( const vehiclecontrol &msg );
    void hndl_steering_angle( const steeringAngle &msg );
    void hndl_acceleration( const acceleration &msg );
    void hndl_platooningState( const platooningState &msg );

  };
}

#endif
