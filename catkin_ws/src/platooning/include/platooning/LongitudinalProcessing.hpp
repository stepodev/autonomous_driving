//
// Created by stepo on 12/16/17.
//

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

#ifndef PLATOONING_LONGITUDINAL_HPP
#define PLATOONING_LONGITUDINAL_HPP

/*****************************************************************************
** Includes
*****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <algorithm>
#include <boost/thread/mutex.hpp>

#include "Topics.hpp"
#include "MessageTypes.hpp"
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>

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

#define DEFAULT_SPRING_CONSTANT 0.01 //the lower, the slower we reach target distance, the slower we brake
#define RANGE_DATA_CHECK 1
#define VELOCITY_DATA_CHECK 2

using SpringConstant = float;
using TimeStep = float;
using Distance = float;
using Velocity = float;

class LongitudinalProcessing : public nodelet::Nodelet {
  public:
	void onInit();

	LongitudinalProcessing();

	~LongitudinalProcessing();

  private:

	class CritiallyDampenedSpring
	{

	  public:
		CritiallyDampenedSpring();

		void set_target_position(const Distance &target_postion) { target_relative_position_ = target_postion; };

		float calulate_velocity(const Distance &current_position,
				                        const Velocity &relative_velocity,
				                        const float &time_step);

		const float& get_target_position() { return target_relative_position_; }

	  private:
		float spring_constant_ = DEFAULT_SPRING_CONSTANT;
		float time_step_ = 1;
		float target_relative_position_ = -1;
	};

	ros::NodeHandle nh_; /**< Some documentation for the member nh_. */
	std::string name_ = "LongitudinalProcessing";

	ros::Subscriber sub_distance_to_obj_;
	ros::Subscriber sub_target_speed_;
	ros::Subscriber sub_current_speed_;
	ros::Subscriber sub_target_distance_;

	ros::Publisher pub_velocity_;

	CritiallyDampenedSpring spring_;

	Distance current_distance_ = 0;
	Distance previous_distance_ = 0;

	float target_velocity_ = 0;
	float current_velocity_ = 0;

	boost::posix_time::ptime previous_distance_timestamp_ = boost::posix_time::min_date_time;
	boost::posix_time::ptime current_distance_timestamp_ = boost::posix_time::min_date_time;

	boost::mutex calc_mutex_;

	/**
	 * @brief to achieve X does Y
	 * @param msg incoming topic message
	 */
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
	void check_dead_datasrc(const boost::system::error_code &e);
	unsigned short data_src_flags = 0;

	int ix = 0;
};

} // namespace platooning

#endif //PLATOONING_LONGITUDINAL_HPP
