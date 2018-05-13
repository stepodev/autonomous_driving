/**
 * @file include/platooning/LongitudinalProcessing.hpp
 * @author stepo
 * @date 22.03.2018
 * @brief LongitudinalProcessing header file

 */

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PLATOONING_LONGITUDINAL_HPP
#define PLATOONING_LONGITUDINAL_HPP

/*****************************************************************************
** Includes
*****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <boost/thread/recursive_mutex.hpp>
#include <algorithm>
#include <boost/thread/mutex.hpp>

#include "Topics.hpp"
#include "MessageTypes.hpp"
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>

namespace platooning {

/**
 * @class LongitudinalProcessing
 *
 * @brief Receives distance and current speed and uses a PD controller to calculate a new speed that will reduce the
 * current distance of to the desired distance.
 *
 * After receiving target distance and target speed values and two distance measurements, starts calculating the speed
 * required to reduce the distance to target distance. Additionally uses a timer to ensure we have up-to-date speed
 * and distance measurements.
 *
 * Creates distances as "distance from point 0" where 0 is an object ahead. Thus distances given as negative values to
 * the PD controller. Ex: The measurement from the sensor is "5", gets interpreted as "-5 units from the object ahead".
 * Target distance could be "1", so "-1 unit from the object ahead".
 *
 * @bugs Suspected cause of random segfault crashes. See issue 72. Seems to be "distance" gets deconstructed sometimes?
 *
 */

#define RANGE_DATA_CHECK (unsigned int)1
#define VELOCITY_DATA_CHECK (unsigned int)2

using TimeStep = float;
using Distance = float;
using Velocity = float;

class LongitudinalProcessing : public nodelet::Nodelet {
  public:
	void onInit();

	LongitudinalProcessing();

	~LongitudinalProcessing();

  private:
	/**
	 * @class PDController
	 *
	 * @brief PD Controller calculates the error between a value representing a desired state and the actual state.
	 * Returns another value to get to that desired state.
	 *
	 * See http://robotic-controls.com/learn/programming/pd-feedback-control-introduction
	 *
	 * We could try and see how a PID performs.
 	 */
	class PDController {
	  public:
		PDController();

		void set_target_position(const Distance &target_postion);

		float calulate_velocity(const Distance &current_position,
		                        const Velocity &current_velocity);

		float get_target_position() { return target_relative_position_; }

	  private:
		float kp_ = 0.8; /**< proportional value. The higher the error, respond proportinally */
		float kd_ = 0.4; /**< derivative value. The closer to error 0, reduce response */
		float target_relative_position_ = -1;
	};

	ros::NodeHandle nh_; /**< Some documentation for the member nh_. */
	std::string name_ = "LongitudinalProcessing";

	ros::Subscriber sub_distance_to_obj_; /**< Subscriber to distance sensor data */
	ros::Subscriber sub_target_velocity_;
	ros::Subscriber sub_current_velocity_;
	ros::Subscriber sub_target_distance_;

	ros::Publisher pub_velocity_;

	boost::recursive_mutex calc_mutex_; /**< mutex to be used before changing or updateing distance and velocity values */
	PDController pd_controller_;

	Distance current_distance_ = 0;
	Distance previous_distance_ = 0;
	float target_velocity_ = 0;
	float current_velocity_ = 0;

	boost::posix_time::ptime previous_distance_timestamp_ = boost::posix_time::min_date_time;
	boost::posix_time::ptime current_distance_timestamp_ = boost::posix_time::min_date_time;

	void hndl_distance_from_sensor(const platooning::distance &msg);
	void hndl_target_distance(const platooning::targetDistance &msg);
	void hndl_current_velocity(const platooning::speed &msg);
	void hndl_targetSpeed(const platooning::targetSpeed &msg);

	void update_velocity();

	//Timer facilites
	boost::posix_time::milliseconds SOURCECHECK_FREQ = boost::posix_time::milliseconds(1000);
	boost::asio::io_service io_service_;
	boost::asio::io_service::work io_worker_;
	boost::asio::deadline_timer detect_dead_datasource_timer;
	boost::thread_group thread_pool_;
	bool dead_data_reminder_warn_ = true;
	void check_dead_datasrc(const boost::system::error_code &e);
	unsigned short data_src_flags = 0;
};

} // namespace platooning

#endif //PLATOONING_LONGITUDINAL_HPP
