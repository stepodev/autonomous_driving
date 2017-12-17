#include <pluginlib/class_list_macros.h>
#include "platooning/EnvironmentMapping.hpp"


namespace platooning
{
    void EnvironmentMapping::onInit()
    {
        NODELET_DEBUG("Initializing nodelet...");
    }
}

PLUGINLIB_EXPORT_CLASS(platooning::EnvironmentMapping, nodelet::Nodelet);
