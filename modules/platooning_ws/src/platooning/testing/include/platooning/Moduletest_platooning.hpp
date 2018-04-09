/**
 * @file testing/include/platooning/Moduletest_platooning.hpp
 * @author stepo
 * @date 24.03.2018
 * @brief Header for platooning modultest
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
#include <nodelet/NodeletUnload.h>
#include <nodelet/NodeletLoad.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

#include "Moduletest.hpp"
#include "platooning/Topics.hpp"
#include "platooning/MessageTypes.hpp"

namespace platooning {

/**
 * @class Moduletest_platooning
 * @brief class that tests platooning module
 */

class Moduletest_platooning : public Moduletest {
  public:
	void onInit();

	Moduletest_platooning();

	~Moduletest_platooning();

  private:

	/**
	 * @brief tests receipt of platooningtoggle and resulting state change into creating
	 */
	void test_send_platoontoggle_recv_platoonstate_creating();
	void hndl_test_send_platoontoggle_recv_platoonstate_creating(platooningState msg);

	/**
	 * @brief tests receipt of platooningtoggle but staying in CREATING as LV when another vehicle is present
	 */
	void send_platoontoggle_stay_creating_lv();

	/**
	 * @brief sends fv heartbeats to lv and expects platooning not to timeout
	 */
	void send_heartbeats_dont_recv_platoonstate_timeout();

	/**
	 * @brief sends lv broadcasts to fv and expects platooning not to timeout
	 */
	void send_broadcast_dont_recv_platoonstate_timeout();

	/**
	 * @brief stops sending lv broadcast to fv and expects platooning to return to IDLE
	 */
	void lv_broadcast_timeout_recv_platoonstate_idle();

	/**
	 * @brief stops sending fv heartbeat without a member and expects platooning to go CREATING
	 */
	void fv_heartbeat_timeout_recv_platoonstate_without_member_timedout();

	/**
	 * @brief sends fv_request while lv is CREATING and expects lv_accept
	 */
	void test_send_fv_request_recv_lv_accept();
	void hndl_tc_send_fv_request_recv_lv_accept( lv_accept msg );

	/**
	 * @brief sends fv_request while lv is RUNNING and expects lv_accept
	 */
	void send_fv_request_recv_platoonstate_running();

	/**
	 * @brief sends fv_request while lv is RUNNING and expects lv_broadcast
	 */
	void send_fv_request_recv_broadcast();

	/**
	 * @brief sends lv_accept and expects fv_heartbeat
	 */
	void send_lv_accept_recv_heartbeat();

	/**
	 * @brief sends lv_broadcast to lv, expects to receive nothing
	 */
	void send_broadcast_to_lv_recv_nothing();

	/**
	 * @brief sends fv_heartbeat to fv, expects to receive nothing
	 */
	void send_heartbeat_to_fv_recv_nothing();

	/**
	 * @brief sends fv_heartbeat to IDLE, expects to receive nothing
	 */
	void send_heartbeat_to_idle_recv_nothing();

	/**
	 * @brief sends fv_request to IDLE, expects to receive nothing
	 */
	void send_fv_request_to_idle_recv_nothing();

	/**
	 * @brief sends fv_leave to IDLE, expects to receive nothing
	 */
	void send_fv_leave_to_idle_recv_nothing();

	/**
	 * @brief sends fv_heartbeat to IDLE, expects to receive nothing
	 */
	void send_fv_heartbeat_to_idle_recv_nothing();

	/**
	 * @brief sends lv_accept to IDLE, expects to receive nothing
	 */
	void send_lv_accept_to_idle_recv_nothing();

	/**
	 * @brief sends lv_reject to IDLE, expects to receive nothing
	 */
	void send_lv_reject_to_idle_recv_nothing();

	/**
	 * @brief sends lv_broadcast to IDLE, expects to receive nothing
	 */
	void send_lv_broadcast_to_idle_recv_nothing();

	/**
	 * @brief sends fv_leave lv with wrong source_id, expects no change
	 */
	void send_fv_leave_non_member_recv_nothing();

	/**
	 * @brief sends two lv_accept expects no change
	 */
	void send_fv_request_receive_two_lv_accept();


	uint32_t vehicle_id_ = 0;
	boost::thread_group thread_pool_;
};

} // namespace platooning

#endif //PLATOONING_MODULETEST_PLATOONING_HPP
