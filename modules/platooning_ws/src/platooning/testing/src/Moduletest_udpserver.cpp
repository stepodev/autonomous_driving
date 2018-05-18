/**
 * @file src/UdpServer.cpp
 * @author stepo
 * @date 23.03.2018
 * @brief Implementation of Moduletest_udpserver class
 *
 */

/*****************************************************************************
** Includes
*****************************************************************************/

#include "platooning/Moduletest_udpserver.hpp"

// %Tag(FULLTEXT)%
namespace platooning {

/*****************************************************************************
** Constructors
*****************************************************************************/


Moduletest_udpserver::Moduletest_udpserver() = default;

/*****************************************************************************
** Destructors
*****************************************************************************/

Moduletest_udpserver::~Moduletest_udpserver() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/

/**
* Set-up necessary publishers/subscribers
* @return true, if successful
*/
void Moduletest_udpserver::onInit() {

	name_ = "Moduletest_udpserver";

	register_testcases(boost::bind(&Moduletest_udpserver::test_stresstest_send_and_receive, this));

	NODELET_INFO("[%s ] init done", name_.c_str());

	start_tests();
}

/*****************************************************************************
** Testcases
*****************************************************************************/

void Moduletest_udpserver::test_stresstest_send_and_receive() {

	set_current_test("test_stresstest_send_and_receive");
	NODELET_INFO("[%s] started testcase %s ", name_.c_str(), get_current_test().c_str());

	set_timeout(boost::posix_time::time_duration(boost::posix_time::seconds(20)));

	boost::function<void(boost::shared_ptr<std::pair<std::string, uint32_t>>)>
		cbfun(boost::bind(boost::mem_fn(&Moduletest_udpserver::hndl_recv_upd), this, _1));

	auto server = boost::shared_ptr<UdpServer>(new UdpServer(
		cbfun,
		udp::endpoint(udp::v4(), 10000),
		udp::endpoint(boost::asio::ip::address_v4::broadcast(), 10000)));
	server->set_filter_own_broadcasts(false);

	boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();

	boost::thread t1 = boost::thread([this, server, start] {
		while ((boost::posix_time::microsec_clock::local_time() - start).total_seconds() <= 10) {
			server->start_send("a", 1);

			send_mtx_.lock();
			send_counter++;
			send_mtx_.unlock();
		}

	});

	boost::thread t2 = boost::thread([this, server, start] {
		while ((boost::posix_time::microsec_clock::local_time() - start).total_seconds() <= 10) {
			server->start_send("a", 1);

			send_mtx_.lock();
			send_counter++;
			send_mtx_.unlock();
		}

	});

	t1.join();

	TestResult res;
	res.success = true;
	res.comment = "sent packets " + std::to_string(send_counter) + " recvd " + std::to_string(recv_counter);

	if (send_counter != recv_counter) {

		res.success = false;
		res.comment = "sent packets " + std::to_string(send_counter) + " recvd "
			+ std::to_string(recv_counter);

	}

	finalize_test(res);

}

void Moduletest_udpserver::hndl_recv_upd(boost::shared_ptr<std::pair<std::string, uint32_t>> msg) {

	recv_mtx_.lock();
	recv_counter += msg->second;
	recv_mtx_.unlock();

}

} // namespace platooning


PLUGINLIB_EXPORT_CLASS(platooning::Moduletest_udpserver, nodelet::Nodelet);
// %EndTag(FULLTEXT)%



