/**
 * @file /testing/include/platooning/Moduletest_messagedistribution.hpp
 *
 * @brief Header file for Moduletest_messagedistribution class
 *
 * @author stepo
 *
 * @date 22.03.2018
 **/

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
#include <sstream>

#include "Moduletest.hpp"
#include "platooning/Topics.hpp"
#include "platooning/MessageTypes.hpp"

namespace platooning {

/**
 * @class Moduletest_messagedistribution
 *
 * @brief Moduletest nodelet for messagedistribution class.
 *
 * Testcases don't need to properly fill and test message fields, as that is covered in the
 * messagetypes tests
 */

class Moduletest_messagedistribution : public Moduletest {
  public:
	void onInit();

	Moduletest_messagedistribution();

	~Moduletest_messagedistribution();

  private:

	std::string handlers_hit_ = "";

	/**
	* @brief publishes LV_BROADCAST on topic IN_PLATOONING_MSG and expects that message to be translated
	* and published as on the IN_LV_BROADCAST topic
	*/
	void test_pub_in_platoonMsg_recv_lv_broadcast();
	void hndl_test_pub_in_platoonMsg_recv_lv_broadcast(platooning::lv_broadcast msg);

	/**
	* @brief publishes lv_broadcast on topic OUT_LV_BROADCAST and expects that message to be translated
	* and published as on the LV_BROADCAST on OUT_PLATOONING_MSG topic
	*/
	void test_pub_in_lv_broadcast_recv_platoonMsg();
	void hndl_test_pub_in_lv_broadcast_recv_platoonMsg(platooning::platoonProtocol msg);

	/**
	* @brief publishes LV_ACCEPT on topic IN_PLATOONING_MSG and expects that message to be translated
	* and published as on the IN_LV_ACCEPT topic
	*/
	void test_pub_in_platoonMsg_recv_lv_accept();
	void hndl_test_pub_in_platoonMsg_recv_lv_accept(platooning::lv_accept msg);

	/**
	* @brief publishes lv_accept on topic OUT_LV_ACCEPT and expects that message to be translated
	* and published as on the LV_ACCEPT on OUT_PLATOONING_MSG topic
	*/
	void test_pub_in_lv_accept_recv_platoonMsg();
	void hndl_test_pub_in_lv_accept_recv_platoonMsg(platooning::platoonProtocol msg);

	/**
	* @brief publishes LV_REJECT on topic IN_PLATOONING_MSG and expects that message to be translated
	* and published as on the IN_LV_REJECT topic
	*/
	void test_pub_in_platoonMsg_recv_lv_reject();
	void hndl_test_pub_in_platoonMsg_recv_lv_reject(platooning::lv_reject msg);

	/**
	* @brief publishes lv_reject on topic OUT_LV_REJECT and expects that message to be translated
	* and published as on the LV_REJECT on OUT_PLATOONING_MSG topic
	*/
	void test_pub_in_lv_reject_recv_platoonMsg();
	void hndl_test_pub_in_lv_reject_recv_platoonMsg(platooning::platoonProtocol msg);

	/**
	* @brief publishes FV_HEARTBEAT on topic IN_PLATOONING_MSG and expects that message to be translated
	* and published as on the IN_FV_HEARTBEAT topic
	*/
	void test_pub_in_platoonMsg_recv_fv_heartbeat();
	void hndl_test_pub_in_platoonMsg_recv_fv_heartbeat(platooning::fv_heartbeat msg);

	/**
	* @brief publishes fv_heartbeat on topic OUT_FV_HEARTBEAT and expects that message to be translated
	* and published as on the FV_HEARTBEAT on OUT_PLATOONING_MSG topic
	*/
	void test_pub_in_fv_heartbeat_recv_platoonMsg();
	void hndl_test_pub_in_fv_heartbeat_recv_platoonMsg(platooning::platoonProtocol msg);

	/**
	* @brief publishes FV_LEAVE on topic IN_PLATOONING_MSG and expects that message to be translated
	* and published as on the IN_FV_LEAVE topic
	*/
	void test_pub_in_platoonMsg_recv_fv_leave();
	void hndl_test_pub_in_platoonMsg_recv_fv_leave(platooning::fv_leave msg);

	/**
	* @brief publishes fv_leave on topic OUT_FV_LEAVE and expects that message to be translated
	* and published as on the FV_LEAVE on OUT_PLATOONING_MSG topic
	*/
	void test_pub_in_fv_leave_recv_platoonMsg();
	void hndl_test_pub_in_fv_leave_recv_platoonMsg(platooning::platoonProtocol msg);

	/**
	* @brief publishes FV_REQUEST on topic IN_PLATOONING_MSG and expects that message to be translated
	* and published as on the IN_FV_REQUEST topic
	*/
	void test_pub_in_platoonMsg_recv_fv_request();
	void hndl_test_pub_in_platoonMsg_recv_fv_request(platooning::fv_request msg);

	/**
	* @brief publishes fv_request on topic OUT_FV_REQUEST and expects that message to be translated
	* and published as on the FV_REQUEST on OUT_PLATOONING_MSG topic
	*/

	void test_pub_in_fv_request_recv_platoonMsg();
	void hndl_test_pub_in_fv_request_recv_platoonMsg(platooning::platoonProtocol msg);

	/**
	* @brief publishes REMOTE_CONTROLINPUT on topic IN_PLATOONING_MSG and expects that message to be translated
	* and published as on the REMOTECONTROL_INPUT topic
	*/
	void test_pub_in_platoonMsg_recv_remotecontrolInput();
	void hndl_test_pub_in_platoonMsg_recv_remotecontrolInput(platooning::remotecontrolInput msg);

	/**
	* @brief publishes REMOTE_CONTROLTOGGLE on topic IN_PLATOONING_MSG and expects that message to be translated
	* and published as on the TOGGLE_PLATOONING topic
	*/
	void test_pub_in_platoonMsg_recv_remotecontrolToggle();
	void hndl_test_pub_in_platoonMsg_recv_remotecontrolToggle(platooning::remotecontrolToggle msg);

	/**
	* @brief publishes REMOTE_PLATOONINGTOGGLE on topic IN_PLATOONING_MSG and expects that message to be translated
	* and published as on the TOGGLE_REMOTECONTROL topic
	*/
	void test_pub_in_platoonMsg_recv_platooningToggle();
	void hndl_test_pub_in_platoonMsg_recv_platooningToggle(platooning::platooningToggle msg);

	/**
	* @brief publishes REMOTE_USERINTERFACE on topic IN_PLATOONING_MSG and expects that message to be translated
	* and published as on the USERINTERFACE topic
	*/
	void test_pub_in_platoonMsg_recv_userInterface();
	void hndl_test_pub_in_platoonMsg_recv_userInterface(platooning::userInterface msg);
	void callback_test_pub_in_platoonMsg_recv_userInterface();

	/**
	 * @brief publishes all messages on all topics and expects to hit every handler for them
	 */
	void test_stresstest_all_messages();

};

} // namespace platooning

#endif //PLATOONING_MODULETEST_MESSAGEDISTRIBUTION_HPP
