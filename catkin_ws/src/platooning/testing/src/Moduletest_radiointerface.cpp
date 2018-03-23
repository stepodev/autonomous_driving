//
// Created by stepo on 12/16/17.
//


/**
 * @file /testing/platooning/Moduletest_radiointerface.cpp
 *
 * @brief Tests RadioInterface class
 *
 * @author stepo
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "platooning/Moduletest_radiointerface.hpp"

// %Tag(FULLTEXT)%
namespace platooning {

/*****************************************************************************
** Constructors
*****************************************************************************/

Moduletest_radiointerface::Moduletest_radiointerface() = default;

/*****************************************************************************
** Destructors
*****************************************************************************/

Moduletest_radiointerface::~Moduletest_radiointerface() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/
void Moduletest_radiointerface::onInit() {



	register_testcases(boost::bind(&Moduletest_radiointerface::test_send_udp_recv_protocolIn, this));
	register_testcases(boost::bind(&Moduletest_radiointerface::test_send_protocolOut_recv_udp, this));
	register_testcases(boost::bind(&Moduletest_radiointerface::test_stresstest_send_udp_recv_protocolIn, this));
	register_testcases(boost::bind(&Moduletest_radiointerface::test_stresstest_send_protocolOut_recv_upd, this));

	NODELET_INFO("[%s] init done", name_.c_str());

	start_tests();
};


/*****************************************************************************
** Testcases
*****************************************************************************/

/**
* @brief tests radiointerface to receive an udp datagram and convert and publish that as a message
*/
void Moduletest_radiointerface::test_send_udp_recv_protocolIn() {

	set_current_test("test_send_udp_recv_protocolIn");

	NODELET_INFO("[%s] test_send_udp_recv_protocolIn", name_.c_str());

	//prepare subscriber
	sub_map_.clear();
	sub_map_.emplace(topics::IN_PLATOONING_MSG, ros::Subscriber());
	sub_map_[topics::IN_PLATOONING_MSG] = nh_.subscribe(topics::IN_PLATOONING_MSG, 1,
	                                                    &Moduletest_radiointerface::hndl_recv_in_protocol,
	                                                    this);

	while (sub_map_[topics::IN_PLATOONING_MSG].getNumPublishers() == 0) {
		boost::this_thread::sleep_for(boost::chrono::seconds(1));
	}

	//setup server to send udp
	try {
		boost::function<void(boost::shared_ptr<std::pair<std::string, uint32_t>>)>
			cbfun(boost::bind(boost::mem_fn(&Moduletest_radiointerface::hndl_recv_udp_dummy), this, _1));

		server_ = std::unique_ptr<UdpServer>(
			new UdpServer(
				cbfun,
				udp::endpoint(udp::v4(), 10001),
				udp::endpoint(boost::asio::ip::address_v4::broadcast(), 10000)));
		server_->set_filter_own_broadcasts(false);
	} catch (std::exception &e) {
		TestResult res;
		res.success = false;
		res.comment = std::string("udpserver init failed ") + e.what();

		finalize_test(res);
	}

	//send msg through our server
	try {
		platooning::platoonProtocol outmsg;
		outmsg.message_type = FV_REQUEST;
		outmsg.payload = get_current_test();

		server_->start_send(outmsg.payload, outmsg.message_type);

	} catch (std::exception &e) {
		TestResult res;
		res.success = false;
		res.comment = std::string("udpsudpserver start_send failed ") + e.what();

		finalize_test(res);
	}
}

void Moduletest_radiointerface::hndl_recv_udp_dummy(boost::shared_ptr<std::pair<std::string, uint32_t>> msg) {
	NODELET_WARN("[%s][hndl_recv_udp_dummy] unexpectedly received msg\ntype %s\npayload %s",
	             name_.c_str(), std::to_string(msg->second).c_str(),
		         msg->first.c_str());
}

void Moduletest_radiointerface::hndl_recv_in_protocol(platooning::platoonProtocol msg) {

	TestResult res;
	res.success = true;
	res.comment = "initial";

	if (msg.payload == get_current_test() && msg.message_type == FV_REQUEST) {
		res.success = true;
		res.comment = msg.payload;
	}

	if (msg.payload != get_current_test()) {
		res.comment = std::string("[ test_send_udp_recv_protocolIn ] payload mismatch.\nought:\"")
			+ get_current_test() + "\"\nwas   \"" + msg.payload + "\"";
		res.success = false;
	}

	if (msg.message_type != FV_REQUEST) {
		res.comment += std::string("\n[ test_send_udp_recv_protocolIn ] message type mismatch. ought:\"")
			+ std::to_string(FV_REQUEST) + "\"\nwas\"" + std::to_string(msg.message_type) + "\"";
		res.success = false;
	}

	finalize_test(res);
}

/**
* @brief tests radiointerface to receive a message and send as udp datagram
*/

void Moduletest_radiointerface::test_send_protocolOut_recv_udp() {

	set_current_test("test_send_protocolOut_recv_udp");
	set_timeout(boost::posix_time::seconds(5));

	//prepare server
	try {
		boost::function<void(boost::shared_ptr<std::pair<std::string, uint32_t>>)>
			cbfun(boost::bind(boost::mem_fn(&Moduletest_radiointerface::handl_test_udp_recvd), this, _1));

		server_ = std::unique_ptr<UdpServer>(new UdpServer(
			cbfun, udp::endpoint(udp::v4(), 10000), udp::endpoint(boost::asio::ip::address_v4::broadcast(), 10000)));
		server_->set_filter_own_broadcasts(false);
	} catch (std::exception &e) {
		TestResult res;
		res.success = false;
		res.comment = std::string("udpserver init failed ") + e.what();

		finalize_test(res);
		return;
	}

	//prepare publisher
	pub_map_.clear();
	pub_map_.emplace(topics::OUT_PLATOONING_MSG, ros::Publisher());
	pub_map_[topics::OUT_PLATOONING_MSG] = nh_.advertise<platooning::platoonProtocol>(topics::OUT_PLATOONING_MSG, 10);

	while (pub_map_[topics::OUT_PLATOONING_MSG].getNumSubscribers() == 0) {
		boost::this_thread::sleep_for(boost::chrono::seconds(1));
	}

	if (pub_map_[topics::OUT_PLATOONING_MSG].getNumSubscribers() == 0) {
		TestResult res;
		res.success = false;
		res.comment = "test_send_protocolOut_recv_udp: no subscribers to topics::OUT_PLATOONING_MSG";
		finalize_test(res);
		return;
	}

	boost::shared_ptr<platooning::platoonProtocol>
		msg = boost::shared_ptr<platooning::platoonProtocol>(new platooning::platoonProtocol);

	msg->payload = get_current_test();
	msg->message_type = FV_LEAVE;

	pub_map_[topics::OUT_PLATOONING_MSG].publish(msg);
}

void Moduletest_radiointerface::handl_test_udp_recvd(boost::shared_ptr<std::pair<std::string, uint32_t>> msg_pair) {

	TestResult res;

	if (msg_pair->first == get_current_test() && msg_pair->second == FV_LEAVE) {
		res.success = true;
	} else if (msg_pair->first != get_current_test()) {
		res.comment = std::string(" payload mismatch.\nought:\"")
			+ get_current_test() + "\"\nwas   \"" + msg_pair->first + "\"";

		res.success = false;
	} else if (msg_pair->second != FV_LEAVE) {
		res.comment += std::string("\nmessage_type mismatch.\nought:\"")
			+ std::to_string(FV_LEAVE)
			+ "\"\nwas   \"" + std::to_string(msg_pair->second) + "\"";
		res.success = false;
	} else {
		res.success = false;
		res.comment = "handl_test_udp_recvd: unknown failure";
	}
	server_->shutdown();

	finalize_test(res);
}
void Moduletest_radiointerface::test_stresstest_send_udp_recv_protocolIn() {
	set_current_test("test_stresstest_send_udp_recv_protocolIn");
	set_timeout(boost::posix_time::seconds(8));

	//prepare server
	try {

		server_->shutdown();

		boost::function<void(boost::shared_ptr<std::pair<std::string, uint32_t>>)>
			cbfun(boost::bind(boost::mem_fn(&Moduletest_radiointerface::hdnl_test_stresstest_send_udp_recv_protocolIn), this, _1));

		server_ = std::unique_ptr<UdpServer>(new UdpServer(
			cbfun, udp::endpoint(udp::v4(), 10000), udp::endpoint(boost::asio::ip::address_v4::broadcast(), 10000)));
		server_->set_filter_own_broadcasts(false);
	} catch (std::exception &e) {
		TestResult res;
		res.success = false;
		res.comment = std::string("udpserver init failed ") + e.what();

		finalize_test(res);
		return;
	}

	//prepare subscriber
	ros::Subscriber sub_= nh_.subscribe(topics::IN_PLATOONING_MSG, 100,
	                                     &Moduletest_radiointerface::hndl_recv_in_protocol,
	                                     this);

	//wait for publishers to be online
	int ix = 0;
	while (sub_.getNumPublishers() == 0 && ix++ < 5) {
		boost::this_thread::sleep_for(boost::chrono::seconds(1));
	}


	if (sub_.getNumPublishers() == 0) {
		TestResult res;
		res.success = false;
		res.comment = "test_stresstest_protocolOut_recv_udp: no publishers to topics::IN_PLATOONING_MSG";
		finalize_test(res);
		return;
	}

	send_counter = 0;
	recv_counter = 0;
	while( send_counter <= 99 ) {

		server_->start_send(get_current_test(), FV_LEAVE);

		send_counter++;

	}

	while( recv_counter != send_counter ) {
		boost::this_thread::sleep_for(boost::chrono::seconds(1));
	}

	TestResult res;

	if (recv_counter == send_counter && recv_counter == 100 ) {
		res.success = true;
	} else {
		res.success = false;
		res.comment = "test_stresstest_protocolOut_recv_udp: sent " + std::to_string(send_counter)
			+ " recv: " + std::to_string(recv_counter);
	}
	server_->shutdown();

	finalize_test(res);

}

void Moduletest_radiointerface::hdnl_test_stresstest_send_udp_recv_protocolIn(boost::shared_ptr<std::pair<std::string,
                                                                                                 uint32_t>> msg) {

	if (msg->first == get_current_test() && msg->second == FV_LEAVE) {
		recv_counter++;
	}

}

void Moduletest_radiointerface::test_stresstest_send_protocolOut_recv_upd() {
	set_current_test("test_stresstest_send_protocolOut_recv_upd");
	set_timeout(boost::posix_time::seconds(8));

	//prepare server
	try {

		server_->shutdown();

		boost::function<void(boost::shared_ptr<std::pair<std::string, uint32_t>>)>
			cbfun(boost::bind(boost::mem_fn(&Moduletest_radiointerface::hdnl_test_stresstest_send_udp_recv_protocolIn), this, _1));

		server_ = std::unique_ptr<UdpServer>(new UdpServer(
			cbfun, udp::endpoint(udp::v4(), 10000), udp::endpoint(boost::asio::ip::address_v4::broadcast(), 10000)));
		server_->set_filter_own_broadcasts(false);
	} catch (std::exception &e) {
		TestResult res;
		res.success = false;
		res.comment = std::string("udpserver init failed ") + e.what();

		finalize_test(res);
		return;
	}

	ros::Publisher pub_ = nh_.advertise<platooning::platoonProtocol>(topics::OUT_PLATOONING_MSG, 100);

	int ix = 0;
	while (pub_.getNumSubscribers() == 0 && ix++ < 5) {
		boost::this_thread::sleep_for(boost::chrono::seconds(1));
	}


	if (pub_.getNumSubscribers() == 0) {
		TestResult res;
		res.success = false;
		res.comment = "test_stresstest_send_protocolOut_recv_upd: no subscribers to topics::OUT_PLATOONING_MSG";
		finalize_test(res);
		return;
	}

	send_counter = 0;
	recv_counter = 0;
	while( send_counter <= 99 ) {

		auto msg = boost::shared_ptr<platooning::platoonProtocol>( new platooning::platoonProtocol() );
		msg->message_type = FV_LEAVE;
		msg->payload = get_current_test();
		pub_.publish(msg);

		send_counter++;

	}

	while( recv_counter != send_counter ) {
		boost::this_thread::sleep_for(boost::chrono::seconds(1));
	}

	TestResult res;

	if (recv_counter == send_counter && recv_counter == 100 ) {
		res.success = true;
	} else {
		res.success = false;
		res.comment = "test_stresstest_send_protocolOut_recv_upd: sent " + std::to_string(send_counter)
			+ " recv: " + std::to_string(recv_counter);
	}
	server_->shutdown();

	finalize_test(res);


}

void Moduletest_radiointerface::handl_test_stresstest_send_protocolOut_recv_upd(boost::shared_ptr<std::pair<std::string,
                                                                                                            uint32_t>> msg) {
	if( msg->second == FV_LEAVE && msg->first == get_current_test() ) {
		recv_counter++;
	}

}
} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Moduletest_radiointerface, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
