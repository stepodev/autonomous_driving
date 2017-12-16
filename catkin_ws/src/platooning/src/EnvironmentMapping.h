#include <nodelet/nodelet.h>
#include "ros/ros.h"

namespace platooning
{

    class EnvironmentMapping : public nodelet::Nodelet
    {
    public:
        virtual void onInit();
    };
}
