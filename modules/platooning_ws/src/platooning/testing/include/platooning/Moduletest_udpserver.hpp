/**
 * @file testing/include/Moduletest_udpserver.hpp
 * @author stepo
 * @date 23.03.2018
 * @brief Header file Moduletest_udpserver class
 *
 */

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PLATOONING_MODULETEST_UDPSERVER_HPP
#define PLATOONING_MODULETEST_UDPSERVER_HPP

/*****************************************************************************
** Includes
*****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include "Moduletest.hpp"
#include "platooning/Topics.hpp"
#include "platooning/MessageTypes.hpp"
#include "platooning/UdpServer.hpp"


namespace platooning {

/**
 * @class Moduletest_udpserver
 *
 * @brief Moduletest for UdpServer.
 *
 */

class Moduletest_udpserver : public Moduletest {
  public:
	void onInit();

	Moduletest_udpserver();

	~Moduletest_udpserver();

  private:
	/**
	* @brief two threads send for 10 seconds, udpserver should receive all
	*/

	boost::mutex send_mtx_;
	int send_counter = 0;
	boost::mutex recv_mtx_;
	int recv_counter = 0;
	void test_stresstest_send_and_receive();
	void hndl_recv_upd( boost::shared_ptr<std::pair<std::string, uint32_t>> msg );

};

} // namespace platooning

#endif //PLATOONING_MODULETEST_UDPSERVER_HPP
