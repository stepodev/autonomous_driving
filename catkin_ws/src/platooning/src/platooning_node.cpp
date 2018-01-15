//
// Created by stepo on 12/16/17.
//

#include <ros/ros.h>
#include <nodelet/loader.h>
#include <boost/thread/thread.hpp>

int main(int argc, char **argv){
    /*

    not working. cant load nodelets like this. use cmdline nodelet manager
    maybe later with system("rosrun ..")


    ros::init(argc, argv, "platooning_node");
    const nodelet::M_string& remap(ros::names::getRemappings());
    nodelet::V_string nargv;

    boost::thread* environnodelet = new boost::thread(
        [&nargv,&remap] {
          nodelet::Loader nodelet;
          nodelet.load("environment_mapping", "platooning/environment_mapping", remap, nargv);} );
    boost::thread* remotenodelet = new boost::thread(
        [&nargv,&remap] {
          nodelet::Loader nodelet;
          nodelet.load("remotecontrol", "platooning/remotecontrol", remap, nargv);} );

    ros::spin();

    environnodelet->join();
    remotenodelet->join();

    return 0;
     */
}