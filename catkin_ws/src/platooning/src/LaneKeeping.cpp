/**
 * @file /platooning/src/platooning.cpp
 *
 * @brief Nodelet implementation of RemoteContol
 *
 * @author stepo
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include <platooning/Topics.hpp>
#include "platooning/LaneKeeping.hpp"

namespace platooning {


/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

    LaneKeeping::LaneKeeping() {};


/*****************************************************************************
** Destructors
*****************************************************************************/

    LaneKeeping::~LaneKeeping() {};


/*****************************************************************************
** Initializers
*****************************************************************************/

    /**
    * Set-up necessary publishers/subscribers
    * @return true, if successful
    */
    void LaneKeeping::onInit() {

        lower_left.x = 2;
        lower_left.y = 2;

        upper_left.x = 2;
        upper_left.y = 8;

        upper_right.x = 8;
        upper_right.y = 8;

        lower_right.x = 8;
        lower_right.y = 2;

        corner = lower_left;
        corner.theta = 0;

        distance_tolerance = 1;
        current_speed.accelleration = 2;

        NODELET_INFO("Creating LaneKeeping Publisher");
	    angle_publisher = nh_.advertise<platooning::steeringAngle>(topics::STEERINGANGLE, 10);
	    accel_publisher = nh_.advertise<platooning::steeringAngle>(topics::ACCELERATION, 10);
        NODELET_INFO("Creating LaneKeeping Subscriber");
        pose_subscriber = nh_.subscribe("turtle1/pose", 10, &LaneKeeping::PoseHandler, this);
    };

    void LaneKeeping::PoseHandler(const turtlesim::Pose poseMsg) {
        NODELET_INFO("Getting a Pose Message");
        current_position = poseMsg;
        PublishPrioritizationDrivingVector();
    }

    /***
     * Publishes a PrioritizationDrivingVector which describes the speed and the steering angle to keep the lane
     */
    void LaneKeeping::PublishPrioritizationDrivingVector(){

        double distance_to_corner = GetDistance(corner, current_position);
        double distance_to_lane_point = GetDistance(point_on_lane, current_position);
        CalculatePointOnLane(1.0);

        if ( distance_to_corner < distance_tolerance){
            DriveToNextCorner();
            current_steering_angle.steering_angle = CalculateSteeringAngle(corner);
        } else {
	        current_steering_angle.steering_angle = CalculateSteeringAngle(point_on_lane);
        }
        angle_publisher.publish(current_steering_angle);
        ROS_DEBUG("distance_corner(%.2f) distance_lane(%.2f) position(%.2f,%.2f) lenkwinkel(%.2f) goal(%.2f,%.2f), lanepoint(%.2f,%.2f)",
                 distance_to_corner, distance_to_lane_point, current_position.x, current_position.y, current_steering_angle.steering_angle, corner.x, corner.y, point_on_lane.x, point_on_lane.y);
    }

    /**
     * Calculates steering angle towards a given point. Always returns a negative value to only make right turns.
     * @param goal_point point to steer to
     * @return angle in gradients
     */
    double LaneKeeping::CalculateSteeringAngle(turtlesim::Pose goal_point){
        double calc_val= 4 * (atan2(goal_point.y - current_position.y, goal_point.x - current_position.x) - current_position.theta);
        double mod_val= std::fmod(calc_val, 6.2831853072);
        ROS_DEBUG("calc_val(%.2f), mod_val(%.2f)", calc_val, mod_val);
        if ( ( calc_val > 0  && mod_val < 0) || ( calc_val < 0  && mod_val > 0) ) {
            ROS_DEBUG("ERROR");
            ROS_DEBUG("ERROR");
            ROS_DEBUG("ERROR");
        }
        return -fabs(mod_val);
    }

    /****
     * Calculates a relatively close (closer than the corners) point on the lane in a given distance. This is necessary to make the turtles drive
     * closer to a imagines lane. Otherwise they making huge turns.
     * @param distance_to_lanepoint distance to the point in driving direction
     */
    void LaneKeeping::CalculatePointOnLane(double tolerance) {
        if (corner.x == lower_left.x && corner.y == lower_left.y) {
            //if car is on lane so let him drive to the corner
            if (lower_left.y + tolerance >= current_position.y &&  lower_left.y - tolerance <= current_position.y){
                point_on_lane = lower_left;
            }else{ //not on lane yet -> steer more to lane
                point_on_lane.x = current_position.x ;
                point_on_lane.y = lower_left.y;
            }
        } else if (corner.x == upper_left.x && corner.y == upper_left.y) {
            //if car is on lane so let him drive to the corner
            if (upper_left.x + tolerance >= current_position.x &&  upper_left.x - tolerance <= current_position.x){
                point_on_lane = upper_left;
            }else { //not on lane yet -> steer more to lane
                point_on_lane.x = upper_left.x;
                point_on_lane.y = current_position.y;
            }
        } else if (corner.x == upper_right.x && corner.y == upper_right.y) {
            //if car is on lane so let him drive to the corner
            if (upper_right.y + tolerance >= current_position.y &&  upper_right.y - tolerance <= current_position.y){
                point_on_lane = upper_right;
            }else { //not on lane yet -> steer more to lane
                point_on_lane.x = current_position.x;
                point_on_lane.y = lower_left.y;
            }
        } else if (corner.x == lower_right.x && corner.y == lower_right.y) {
            //if car is on lane so let him drive to the corner
            if (lower_right.x + tolerance >= current_position.x &&  lower_right.x - tolerance <= current_position.x){
                point_on_lane = lower_right;
            }else { //not on lane yet -> steer more to lane
                point_on_lane.x = upper_left.x;
                point_on_lane.y = current_position.y;
            }
        }
    }

    /***
     * When the car is close to a corner this function is called and selects the next corner the car should drive to
     */
    void LaneKeeping::DriveToNextCorner(){

        if (corner.x == lower_left.x && corner.y == lower_left.y){
            corner = upper_left;
            NODELET_INFO("New goal is upper left");
        } else if ( corner.x == upper_left.x && corner.y == upper_left.y ){
            corner = upper_right;
            NODELET_INFO("New goal is upper right");
        } else if ( corner.x == upper_right.x && corner.y == upper_right.y ){
            corner = lower_right;
            NODELET_INFO("New goal is lower right");
        } else if ( corner.x == lower_right.x && corner.y == lower_right.y ){
            corner = lower_left;
            NODELET_INFO("New goal is lower left");
        }
    }

    /***
     * Calculates the distance between to points
     * @param goal
     * @param current
     * @return
     */
    double LaneKeeping::GetDistance(turtlesim::Pose goal, turtlesim::Pose current){
        return sqrt(pow(goal.x-current.x,2) + pow(goal.y-current.y,2));
    }

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::LaneKeeping, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
