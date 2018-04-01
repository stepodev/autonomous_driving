/**
 * @file testing/src/Moduletest_longitudinalprocessing.hpp
 * @author stepo
 * @date 22,03,2018
 * @brief Contains header of Moduletest_longitudinalprocessing class
 *
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PLATOONING_MODULETEST_PLATOONING_HPP
#define PLATOONING_MODULETEST_PLATOONING_HPP

/*****************************************************************************
** Includes
*****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <algorithm>

#include "Moduletest.hpp"
#include "platooning/Topics.hpp"
#include "platooning/MessageTypes.hpp"

namespace platooning {

/**
 * @class Moduletest_longitudinalprocessing
 *
 * @brief Moduletest nodelet for LongitudinalProcessing class.
 *
 */

class Moduletest_longitudinalprocessing : public Moduletest {
  public:
	void onInit();

	Moduletest_longitudinalprocessing();

	~Moduletest_longitudinalprocessing();

  private:
	/**
	 * @brief Sends target and current distance, target and current velocity, and expects to receive new velocity on
	 * topics::CALCULATED_VELOCITY
	 */
	void test_send_new_data_recv_velocity();
	void hndl_test_send_new_data_recv_velocity(const speed &);

	/**
	 * @brief Times out distance and velo data, excepts velocity 0 on
	 * topics::CALCULATED_VELOCITY
	 */
	void test_send_timeout_distance_velo_recv_velocity();
	void hndl_test_send_timeout_distance_velo_recv_velocity(const speed &);

	/**
	 * @brief Simulates object ahead increasing velocity, expects returned velocities to catch this car up
	 *
	 * Simulates two cars starting at same speed and at correct distance. Changes LV speed up and down. Expects distance
	 * to never go below 0 and to be around the desired distance at the end
	 */

	float fv_pos = 0;
	float lv_pos = 1;
	float fv_velo = 0;
	float lv_velo = 2;
	boost::posix_time::ptime start_time;
	void test_change_velocity_keep_up();
	void hndl_test_change_velocity_keep_up_velo(const platooning::speed &);
	void hndl_test_change_velocity_keep_up_timer(const boost::system::error_code &e);
	void hndl_test_change_velocity_keep_up_scenario_over();

	boost::posix_time::milliseconds TIMER_FREQ = boost::posix_time::milliseconds(20);
	boost::asio::io_service io_service_;
	boost::asio::io_service::work io_worker_;
	boost::asio::deadline_timer update_timer;
	boost::thread_group thread_pool_;
};

} // namespace platooning

#endif //PLATOONING_MODULETEST_PLATOONING_HPP
