/**
 * @file testing/include/platooning/Integrationtest_platooning.hpp
 * @author stepo
 * @date 22,03,2018
 * @brief Contains header of Integrationtest_platooning class
 *
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PLATOONING_INTEGRATIONTEST_PLATOONING_HPP
#define PLATOONING_INTEGRATIONTEST_PLATOONING_HPP

/*****************************************************************************
** Includes
*****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <list>

#include "Integrationtest.hpp"
#include "platooning/Topics.hpp"
#include "platooning/MessageTypes.hpp"
#include "platooning/UdpServer.hpp"
#include "platooning/Platooning.hpp"

const boost::posix_time::time_duration HEARTBEAT_FREQ(boost::posix_time::milliseconds(200));
const boost::posix_time::time_duration HEARTBEAT_TIMEOUT(boost::posix_time::milliseconds(1000));
const boost::posix_time::time_duration BROADCAST_FREQ(boost::posix_time::milliseconds(50));
const boost::posix_time::time_duration BROADCAST_TIMEOUT(boost::posix_time::milliseconds(1000));

namespace platooning {

/**
 * @class Moduletest_longitudinalprocessing
 *
 * @brief Moduletest nodelet for LongitudinalProcessing class.
 *
 */

class Integrationtest_platooning : public Integrationtest {
  public:
	void onInit();

	Integrationtest_platooning();

	~Integrationtest_platooning() override;

  private:
	/**
	 * @brief sends platooningToggle via udp to start as LV, adds a FV, expects lv_broadcast with correct vehicle_id
	 */
	void test_send_platooningToggle_recv_heartbeats_and_broadcast();
	void cleanup_test_send_platooningToggle_recv_heartbeats_and_broadcast();

	/**
	 * @brief starts platooning for all members and expects platooning data to be in lv_broadcast
	 */
	void test_send_platooningToggle_recv_heartbeat_data_and_broadcast_data();
	void cleanup_test_send_platooningToggle_recv_heartbeat_data_and_broadcast_data();


	/**
	 * @brief starts local LV, toggles FVs, sends broadcast updates, expects ui to have these updates
	 */

	void test_send_updated_broadcast_receive_userinterface();
	void cleanup_test_send_updated_broadcast_receive_userinterface();

	std::list<std::pair<boost::posix_time::ptime, platooning::lv_broadcast>> bclist;
	std::list<std::pair<boost::posix_time::ptime,platooning::fv_heartbeat>> fv2hb;
	std::list<std::pair<boost::posix_time::ptime,platooning::fv_heartbeat>> fv3hb;
	std::list<std::pair<boost::posix_time::ptime,platooning::userInterface>> lvui;
	std::list<std::pair<boost::posix_time::ptime,platooning::userInterface>> fv2ui;
	std::list<std::pair<boost::posix_time::ptime,platooning::userInterface>> fv3ui;

	void hndl_lv_broadcast( const lv_broadcast& msg );
	void hndl_fv_heartbeat( const fv_heartbeat& msg );

	void receive_udp_message( const boost::shared_ptr<std::pair<std::string, uint32_t>>& msgpair );

	boost::thread_group thread_pool_;

	//heartbeat timers
	boost::asio::io_service io_service_;
	boost::asio::io_service::work io_worker_;
	boost::asio::deadline_timer fv_heartbeat_sender_;
	boost::asio::deadline_timer fv_heartbeat_checker_;
	boost::asio::deadline_timer lv_broadcast_sender_;
	boost::asio::deadline_timer lv_broadcast_checker_;

	//follower list and timeouts;
	std::unordered_map<uint32_t, boost::posix_time::ptime> fv_heartbeat_timeout_tracker_;
	std::pair<uint32_t, boost::posix_time::ptime> lv_broadcast_timeout_tracker_;

	/**
	 * @brief resets all timers and trackers to initial state
	 */
	void reset();

	std::unique_ptr<UdpServer> server_ptr_;

};
} // namespace platooning

#endif //PLATOONING_INTEGRATIONTEST_PLATOONING_HPP
