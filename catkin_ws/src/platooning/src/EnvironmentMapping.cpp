#include <pluginlib/class_list_macros.h>
#include "EnvironmentMapping.h"


namespace platooning
{
    void EnvironmentMapping::onInit()
    {
        NODELET_DEBUG("Initializing nodelet...");
    }
}

PLUGINLIB_EXPORT_CLASS(platooning::EnvironmentMapping, nodelet::Nodelet);
