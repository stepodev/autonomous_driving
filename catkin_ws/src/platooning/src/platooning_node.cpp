//
// Created by stepo on 12/16/17.
//

#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "platooning_node");
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "platooning/environment_mapping", remap, nargv);
    nodelet.load(nodelet_name, "platooning/remotecontrol", remap, nargv);
    ros::spin();

    return 0;

}