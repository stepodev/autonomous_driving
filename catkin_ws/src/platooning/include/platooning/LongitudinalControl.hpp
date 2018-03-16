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

#include "Topics.hpp"
#include "MessageTypes.hpp"

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

#define DEFAULT_TIMESTEP 0.3 //i think this is Hz
#define DEFAULT_SPRING_CONSTANT 20 //the dice said that

using SpringConstant = float;
using TimeStep = float;
using Distance = float;
using Speed = float;

class LongitudinalControl : public nodelet::Nodelet {
  public:
	void onInit();

	LongitudinalControl();

	~LongitudinalControl();

  private:

	class CritiallyDampenedSpring
	{

	  public:
		CritiallyDampenedSpring( SpringConstant spring_constant,
		                         TimeStep time_step,
		                         Distance target_distance);

		void set_target_distance( const Distance& target_distance ) { target_distance_ = target_distance; };

		float calulate_velocity( const Distance& current_distance, const Speed& current_speed, const Speed& target_speed );

	  private:
		float spring_constant_;
		float time_step_;
		float target_distance_;
	};

	ros::NodeHandle nh_; /**< Some documentation for the member nh_. */
	std::string name_ = "LongitudinalControl";

	ros::Subscriber sub_distance_to_obj_;
	ros::Subscriber sub_target_speed_;
	ros::Subscriber sub_current_speed_;
	ros::Subscriber sub_target_distance_;
	ros::Subscriber sub_critically_dampened_spring_params_;

	ros::Publisher pub_acceleration_;

	CritiallyDampenedSpring spring_;

	float current_distance_;
	float target_distance_;
	float target_speed_;
	float current_speed_;
	boost::mutex calc_mutex_;

	/**
	 * @brief to achieve X does Y
	 * @param msg incoming topic message
	 */
	void hndl_distance_from_sensor(const platooning::distance &msg);
	void hndl_target_distance(const platooning::targetDistance &msg);
	void hndl_current_speed(const platooning::speed &msg);
	void hndl_targetSpeed(const platooning::targetSpeed &msg);
	void hndl_spring_update( const platooning::criticallyDampenedSpring &msg );


	void update_speed();

	void set_spring(const SpringConstant &spring_constant,
	                const TimeStep &time_step,
	                const Distance &target_distance);

};

} // namespace platooning

#endif //PLATOONING_LONGITUDINAL_HPP
