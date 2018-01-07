#include <pluginlib/class_list_macros.h>
#include "platooning/EnvironmentMapping.hpp"
#include <std_msgs/Float64.h>

namespace platooning
{
    void EnvironmentMapping::onInit()
    {
        NODELET_DEBUG("Initializing nodelet...");

        nh = getNodeHandle();
        pub = nh.advertise<std_msgs::Float64>("distance_to_obj",1000);
        // output from optical capturing
        std_msgs::Float64Ptr output(new std_msgs::Float64());
        output->data = 0.33;
        while (true)
        {
            sleep(1);
            pub.publish(output);
        }
        NODELET_DEBUG("the distance is %f", output->data);
    }
}

PLUGINLIB_EXPORT_CLASS(platooning::EnvironmentMapping, nodelet::Nodelet);
