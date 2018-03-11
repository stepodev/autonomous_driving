//
// Created by stepo on 2/3/18.
//

#ifndef PROJECT_TOPICS_HPP
#define PROJECT_TOPICS_HPP

#include <string>

namespace topics {
//RadioInterface Topics
const std::string IN_PLATOONING_MSG = "in/platoonProtocol";
const std::string OUT_PLATOONING_MSG = "out/platoonProtocol";

//Userinterface Topics
const std::string USERINTERFACE = "userinterface";

//platooning topics
const std::string IN_LV_BROADCAST = "in/lv_broadcast";
const std::string IN_LV_ACCEPT = "in/lv_accept";
const std::string IN_LV_REJECT = "in/lv_reject";
const std::string OUT_LV_BROADCAST = "out/lv_broadcast";
const std::string OUT_LV_ACCEPT = "out/lv_accept";
const std::string OUT_LV_REJECT = "out/lv_reject";

const std::string IN_FV_REQUEST = "in/fv_request";
const std::string IN_FV_HEARTBEAT = "in/fv_heartbeat";
const std::string IN_FV_LEAVE = "in/fv_leave";
const std::string OUT_FV_REQUEST = "out/fv_request";
const std::string OUT_FV_HEARTBEAT = "out/fv_heartbeat";
const std::string OUT_FV_LEAVE = "out/fv_leave";

const std::string PLATOONINGSTATE = "platooningState";

//remotecontrol topics
const std::string TOGGLE_REMOTECONTROL = "toggle_remotecontrol";
const std::string TOGGLE_PLATOONING = "toggle_platooning";
const std::string REMOTECONTROL_INPUT = "remotecontrolinput";

//control topics
const std::string SENSOR_DISTANCE_TO_OBJ = "sensor_distance_to_obj";
const std::string CURRENT_SPEED = "current_speed";
const std::string VEHICLE_CONTROL = "vehiclecontrol";
const std::string STEERING_ANGLE = "steeringangle";
const std::string ACCELERATION = "acceleration";

//prioritziation topics
const std::string TARGET_SPEED = "target_speed";
const std::string TARGET_DISTANCE = "target_distance";
const std::string TARGET_ANGLE = "target_angle";

//lanedetect
const std::string CAMERA = "camera";

//template
const std::string TEMPLATETOPIC = "templateTopic";

}
#endif //PROJECT_TOPICS_HPP
