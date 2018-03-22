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

#ifndef PLATOONING_PRIORITIZATION_HPP
#define PLATOONING_PRIORITIZATION_HPP


/*****************************************************************************
** Includes
*****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

#include "MessageTypes.hpp"
#include "Topics.hpp"
#include "Services.hpp"
#include "ServiceTypes.hpp"

#define LOWEST_SPEED -5.0f
#define HIGHEST_SPEED 7.0f

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

enum class PrioritizationMode {
	NONE,
	REMOTECONTROL,
	PLATOONING,
};

std::string to_string( const PrioritizationMode& e ) {
	switch ( e ) {
		case PrioritizationMode::NONE:
			return "NONE";
		case PrioritizationMode::REMOTECONTROL:
			return "REMOTECONTROL";
		case PrioritizationMode::PLATOONING:
			return "PLATOONING";
	}

	return "";
}

class Prioritization : public nodelet::Nodelet {
  public:
	virtual void onInit();

	Prioritization();

	~Prioritization();

  private:
	ros::NodeHandle nh_; /**< Some documentation for the member nh_. */
	std::string name_ = "Prioritization";
	float target_speed_;
	float current_speed_;
	float target_distance_;
	float target_angle_;
	uint32_t vehicle_id_ = 1;
	PrioritizationMode mode_;
	platooningState platooning_state_;
	boost::thread_group thread_pool_;

	//Subscribers
	ros::Subscriber sub_remotecontrolInput_;
	ros::Subscriber sub_remotecontrolToggle_;
	ros::Subscriber sub_platooningToggle_;
	ros::Subscriber sub_platooningState_;
	ros::Subscriber sub_sensor_velocity_;
	ros::Subscriber sub_calc_velocity_;

	//Publishers
	ros::Publisher pub_targetAngle_;
	ros::Publisher pub_targetSpeed_;
	ros::Publisher pub_targetDistance_;
	ros::Publisher pub_vehicleControl_;

	/**
	 * @brief to achieve X does Y
	 * @param msg incoming topic message
	 */

	void hndl_remotecontrolInput(const remotecontrolInput &);
	void hndl_remotecontrolToggle(const remotecontrolToggle &);
	void hndl_platooningToggle(const platooningToggle &);
	void hndl_platooningState(const platooningState &);
	void hndl_current_speed(const speed&);
	void hndl_calc_velocity(const speed&);

};

} // namespace platooning

#endif //PLATOONING_PRIORITIZATION_HPP
