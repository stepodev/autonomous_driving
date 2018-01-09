#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "ros/ros.h"
#include <std_msgs/Float64.h>

namespace platooning
{

    class EnvironmentMapping : public nodelet::Nodelet
    {
    public:
        virtual void onInit();

    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
    };
}
