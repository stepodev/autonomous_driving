/**
 * @file testing/src/Prioritization.hpp
 * @author stepo
 * @date 22,03,2018
 * @brief Contains header of Prioritization class
 *
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

#define LOWEST_SPEED -5.0f /**< lower bound of velocity our car can go */
#define HIGHEST_SPEED 7.0f /**< upper bound of velocity our car can go */

namespace platooning {


/**
 * @class PrioritizationMode
 * @brief helper class containing all possible state the nodelet can be in
 */
enum class PrioritizationMode {
	NONE,
	REMOTECONTROL,
	PLATOONING,
};

/**
 * @brief helper function to convert state to printable string
 * @param pm state to be translated
 * @return string of state
 */
std::string to_string( const PrioritizationMode& pm ) {
	switch ( pm ) {
		case PrioritizationMode::NONE:
			return "NONE";
		case PrioritizationMode::REMOTECONTROL:
			return "REMOTECONTROL";
		case PrioritizationMode::PLATOONING:
			return "PLATOONING";
	}

	//to remove warning
	return "";
}


/**
 * @class Prioritization
 *
 * @brief Listens to data sent by longitudinal and lateral processing nodes, the platooning node, the input from
 * the controller PC and decides which data to forward to the car's contoller.
 *
 * @bugs no known
 */
class Prioritization : public nodelet::Nodelet {
  public:
	virtual void onInit();

	Prioritization();

	~Prioritization();

  private:
	ros::NodeHandle nh_;
	std::string name_ = "Prioritization";
	float target_speed_;
	float current_speed_;
	float target_distance_;
	float target_angle_;
	uint32_t vehicle_id_ = -1;
	boost::thread_group thread_pool_;

	PrioritizationMode mode_; /**< saves prioritization state we are in */
	platooningState platooning_state_; /**< saves platooning state we received */

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


	void hndl_remotecontrolInput(const remotecontrolInput &);
	void hndl_remotecontrolToggle(const remotecontrolToggle &);
	void hndl_platooningToggle(const platooningToggle &);
	void hndl_platooningState(const platooningState &);
	void hndl_current_speed(const speed&);
	void hndl_calc_velocity(const speed&);

};

} // namespace platooning

#endif //PLATOONING_PRIORITIZATION_HPP
