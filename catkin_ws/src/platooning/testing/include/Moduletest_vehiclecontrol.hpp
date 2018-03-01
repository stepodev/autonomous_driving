/**
 * @file doxygen_c.h
 * @author My Self
 * @date 9 Sep 2012
 * @brief File containing example of doxygen usage for quick reference.
 *
 * Here typically goes a more extensive explanation of what the header
 * defines. Doxygens tags are words preceeded by either a backslash @\
 * or by an at symbol @@.
 * @see http://www.stack.nl/~dimitri/doxygen/docblocks.html
 * @see http://www.stack.nl/~dimitri/doxygen/commands.html
 */

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PLATOONING_MODULETEST_MESSAGEDISTRIBUTION_HPP
#define PLATOONING_MODULETEST_MESSAGEDISTRIBUTION_HPP

/*****************************************************************************
** Includes
*****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
<<<<<<< HEAD:catkin_ws/src/platooning/testing/include/Moduletest_messagedistribution.hpp
#include <sstream>

#include "Moduletest.hpp"
#include "Topics.hpp"
#include "MessageTypes.hpp"
=======
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include "platooning/prioritisationDrivingVector.h"
#include "platooning/templateMsg.h" //includes topic aka message
#include <math.h>
>>>>>>> vehicleControl:catkin_ws/src/platooning/include/platooning/LaneKeeping.hpp

namespace platooning {


/**
 * @brief Example showing how to document a function with Doxygen.
 *
 * Description of what the function does. This part may refer to the parameters
 * of the function, like @p param1 or @p param2. A word of code can also be
 * inserted like @c this which is equivalent to <tt>this</tt> and can be useful
 * to say that the function returns a @c void or an @c int. If you want to have
 * more than one word in typewriter font, then just use @<tt@>.
 * We can also include text verbatim,
 * @verbatim like this@endverbatim
 * Sometimes it is also convenient to include an example of usage:
 * @code
 * BoxStruct *out = Box_The_Function_Name(param1, param2);
 * printf("something...\n");
 * @endcode
 * Or,
 * @code{.py}
 * pyval = python_func(arg1, arg2)
 * print pyval
 * @endcode
 * when the language is not the one used in the current source file (but
 * <b>be careful</b> as this may be supported only by recent versions
 * of Doxygen). By the way, <b>this is how you write bold text</b> or,
 * if it is just one word, then you can just do @b this.
 * @param param1 Description of the first parameter of the function.
 * @param param2 The second one, which follows @p param1.
 * @return Describe what the function returns.
 * @see Box_The_Second_Function
 * @see Box_The_Last_One
 * @see http://website/
 * @note Something to note.
 * @warning Warning.
 */

<<<<<<< HEAD:catkin_ws/src/platooning/testing/include/Moduletest_messagedistribution.hpp
  class Moduletest_messagedistribution : public Moduletest {
  public:
    void onInit();

    Moduletest_messagedistribution();

    ~Moduletest_messagedistribution();

  private:
    /**
    * @brief publishes FV_REQUEST on topic in/platoonMsg and expects that message to be translated
    * and published as on the in/fv_request topic
    */
    void pub_in_platoonMsg_recv_fv_request();

    void hndl_pub_in_platoonMsg_recv_fv_request(platooning::fv_request msg);
  };
=======
    class LaneKeeping : public nodelet::Nodelet {
    public:
        virtual void onInit();

        LaneKeeping(ros::NodeHandle &nh, std::string &name);

        LaneKeeping();

        ~LaneKeeping();

    private:
        ros::NodeHandle nh_; /**< Some documentation for the member nh_. */
        std::string name_;
        ros::Subscriber pose_subscriber;
        ros::Publisher pdv_publisher;
        turtlesim::Pose current_position;
        turtlesim::Pose corner;
        turtlesim::Pose point_on_lane;
        turtlesim::Pose upper_left;
        turtlesim::Pose lower_left;
        turtlesim::Pose upper_right;
        turtlesim::Pose lower_right;
        double distance_tolerance;
        platooning::prioritisationDrivingVector current_vector;


        void PoseHandler(const turtlesim::Pose poseMsg);

        void PublishPrioritizationDrivingVector();
>>>>>>> vehicleControl:catkin_ws/src/platooning/include/platooning/LaneKeeping.hpp

        void DriveToNextCorner();

        double GetDistance(turtlesim::Pose goal, turtlesim::Pose current);

        double CalculateSteeringAngle(turtlesim::Pose point);

        void CalculatePointOnLane(double tolerance);



    };
}// namespace platooning

#endif //PLATOONING_MODULETEST_MESSAGEDISTRIBUTION_HPP
