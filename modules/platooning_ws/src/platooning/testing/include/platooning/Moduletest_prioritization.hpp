/**
 * @file testing/include/platooning/Moduletest_prioritization.hpp
 * @author stepo
 * @date 22,03,2018
 * @brief Contains header of Moduletest_longitudinalprocessing class
 *
 */

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PLATOONING_MODULETEST_PRIORITIZATION_HPP
#define PLATOONING_MODULETEST_PRIORITIZATION_HPP

/*****************************************************************************
** Includes
*****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "Moduletest.hpp"
#include "platooning/Topics.hpp"
#include "platooning/MessageTypes.hpp"
#include "platooning/ServiceTypes.hpp"
#include "platooning/Services.hpp"

namespace platooning {

/**
 * @class Moduletest_prioritization
 * @brief Tests prioritization nodelet
 */

class Moduletest_prioritization : public Moduletest {
  public:
	void onInit();

	Moduletest_prioritization();

	~Moduletest_prioritization();

  private:
	/**
	 * @brief toggles remotecontrol, sends speed, expects to receive vehicleControl, toggle remotecontrol, send speed
	 * and dont receive vehiclecontrol
	 */
	boost::mutex mtx;
	platooning::vehicleControl vc_msg;
	bool vehiclecontrol_received = false;
	void test_remotecontrol_toggle_and_speed_recv_vehiclecontrol();
	void hndl_test_remotecontrol_toggle_and_speed_recv_vehiclecontrol(const platooning::vehicleControl &msg);

	/**
	 * @brief toggle platooning, send targetspeed and targetdistance, send distance and speed, receive calc speed
	 * toggle platooning receive nothing
	 */
	void test_platooning_toggle_and_speed_recv_vehiclecontrol();
	void hndl_callback_test_platooning_toggle_and_speed_recv_vehiclecontrol(const platooning::vehicleControl &msg);

	//provide vehicle id to prio nodelet
	ros::ServiceServer vehicle_id_server_;
	bool provide_vehicle_id(platooning::getVehicleId::Request &req,
	                                    platooning::getVehicleId::Response &res);
};

} // namespace platooning

#endif //PLATOONING_MODULETEST_PRIORITIZATION_HPP
