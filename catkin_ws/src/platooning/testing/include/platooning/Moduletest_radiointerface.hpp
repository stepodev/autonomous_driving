/**
 * @file testing/src/Moduletest_radiointerface.hpp
 * @author stepo
 * @date 22,03,2018
 * @brief Contains header of Moduletest_radiointerface class
 *
 */

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PLATOONING_MODULETEST_RADIOINTERFACE_HPP
#define PLATOONING_MODULETEST_RADIOINTERFACE_HPP

/*****************************************************************************
** Includes
*****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <sstream>

#include "platooning/Topics.hpp"
#include "platooning/MessageTypes.hpp"
#include "platooning/UdpServer.hpp"
#include "Moduletest.hpp"

using boost::asio::ip::udp;

namespace platooning {

/**
 * @brief Moduletest Nodelet for RadioInterface.
 */

class Moduletest_radiointerface : public Moduletest {
  public:
	virtual void onInit();

	Moduletest_radiointerface();

	~Moduletest_radiointerface();

  private:
	ros::NodeHandle nh_;
	std::string name_ = "Moduletest_radiointerface";
	std::unique_ptr<UdpServer> server_;

	/**
	 * @brief sends upd datagram, expects to get a platoonprotocol on IN_PLATOON_MSG topic
	 */
	void test_send_udp_recv_protocolIn();
	void hndl_recv_udp_dummy(boost::shared_ptr<std::pair<std::string, uint32_t>> msg);
	void hndl_recv_in_protocol(platooning::platoonProtocol msg);

	/**
	 * @brief sends platoonProtocol message on OUT_PLATOON_MSG topic and expects to receive the udp datagram
	 */
	void test_send_protocolOut_recv_udp();
	void handl_test_udp_recvd(boost::shared_ptr<std::pair<std::string, uint32_t>> msg);

	/**
	 * @brief sends 100 upd datagram, expects to get a platoonprotocol on IN_PLATOON_MSG topic
	 */
	int send_counter = 0;
	int recv_counter = 0;
	void test_stresstest_send_udp_recv_protocolIn();
	void hdnl_test_stresstest_send_udp_recv_protocolIn(boost::shared_ptr<std::pair<std::string, uint32_t>> msg);


	/**
	 * @brief sends 100 platoonProtocol message on OUT_PLATOON_MSG topic and expects to receive the udp datagrams
	 */
	void test_stresstest_send_protocolOut_recv_upd();
	void handl_test_stresstest_send_protocolOut_recv_upd(boost::shared_ptr<std::pair<std::string, uint32_t>> msg);
};

} // namespace platooning

#endif //PLATOONING_MODULETEST_RADIOINTERFACE_HPP
