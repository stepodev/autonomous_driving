/**
 * @file testing/src/Moduletest_radiointerface.hpp
 * @author stepo
 * @date 22,03,2018
 * @brief Contains header of Moduletest_longitudinalprocessing class
 *
 */

#include <platooning/Platooning.hpp>
#include "platooning/Integrationtest_platooning.hpp"

// %Tag(FULLTEXT)%
namespace platooning {

/*****************************************************************************
** Constructors
*****************************************************************************/

Integrationtest_platooning::Integrationtest_platooning()
	: io_worker_(io_service_),
	  fv_heartbeat_sender_(io_service_),
	  fv_heartbeat_checker_(io_service_),
	  lv_broadcast_sender_(io_service_),
	  lv_broadcast_checker_(io_service_) {

	thread_pool_.create_thread([this] { io_service_.run(); });

}

/*****************************************************************************
** Destructors
*****************************************************************************/

Integrationtest_platooning::~Integrationtest_platooning() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/

/**
* Set-up necessary publishers/subscribers
*/
void Integrationtest_platooning::onInit() {

	name_ = "Integrationtest_platooning";
	nh_ = getMTNodeHandle();

	int testcaseid;
	nh_.getParam("testcase", testcaseid);

	if( !nh_.hasParam("testcase")){
		NODELET_FATAL("[%s] testcase param missing", name_.c_str());
		exit(-1);
	}

	NODELET_INFO("[%s] selected testcase %i", name_.c_str(), testcaseid);

	if (testcaseid == 1) {
		register_testcases(boost::bind(&Integrationtest_platooning::test_send_platooningToggle_recv_heartbeats_and_broadcast,
		                               this));
	}

	if (testcaseid == 2) {
		register_testcases(boost::bind(&Integrationtest_platooning::test_send_platooningToggle_recv_heartbeat_data_and_broadcast_data,
		                               this));
	}

	if (testcaseid == 3) {
		register_testcases(boost::bind(&Integrationtest_platooning::test_send_updated_broadcast_receive_userinterface,
		                               this));
	}

	if (testcaseid <= 0) {
		NODELET_INFO("[%s] no testcase selected. running all", name_.c_str());

		register_testcases(boost::bind(&Integrationtest_platooning::test_send_platooningToggle_recv_heartbeats_and_broadcast,
		                               this));
		register_testcases(boost::bind(&Integrationtest_platooning::test_send_platooningToggle_recv_heartbeat_data_and_broadcast_data,
		                               this));
		register_testcases(boost::bind(&Integrationtest_platooning::test_send_updated_broadcast_receive_userinterface,
		                               this));
	}

	auto recv_udp_msg_cb = boost::bind(boost::mem_fn(&Integrationtest_platooning::receive_udp_message), this, _1);

	server_ptr_ = std::unique_ptr<UdpServer>(new UdpServer(recv_udp_msg_cb,
	                                                       udp::endpoint(udp::v4(), 13500),
	                                                       udp::endpoint(boost::asio::ip::address_v4::broadcast(),
	                                                                     13500)));

	NODELET_INFO("[%s] init done", name_.c_str());

	thread_pool_.create_thread([this] {
		start_tests();
	});
}

/*****************************************************************************
** Testcases
*****************************************************************************/

void Integrationtest_platooning::test_send_platooningToggle_recv_heartbeats_and_broadcast() {

	set_timeout(boost::posix_time::time_duration(boost::posix_time::seconds(20)));
	set_current_test("test_send_platooningToggle_recv_heartbeats_and_broadcast");

	boost::function<void()> cbfun(boost::bind(boost::mem_fn(
		&Integrationtest_platooning::cleanup_test_send_platooningToggle_recv_heartbeats_and_broadcast), this));

	register_timeout_callback(cbfun);

	//mockup publishers
	pub_map_.emplace(topics::OUT_PLATOONING_MSG,
	                 nh_.advertise<platooning::platoonProtocol>(topics::OUT_PLATOONING_MSG, 1));

	while (pub_map_[topics::OUT_PLATOONING_MSG].getNumSubscribers() < 1) {
		boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
	}

	//mockup subscriber to velocity to catch calculated velocity
	sub_map_.emplace(topics::IN_FV_HEARTBEAT, nh_.subscribe(topics::IN_FV_HEARTBEAT, 1,
	                                                        &Integrationtest_platooning::hndl_fv_heartbeat,
	                                                        this));
	sub_map_.emplace(topics::IN_LV_BROADCAST, nh_.subscribe(topics::IN_LV_BROADCAST, 1,
	                                                        &Integrationtest_platooning::hndl_lv_broadcast,
	                                                        this));

	//create toggle messages for all three vehicles, change only relevant fields
	//VEHICLE 1
	platooningToggle toggle_msg;
	toggle_msg.vehicle_id = 1;
	toggle_msg.lvfv = "LV";
	toggle_msg.enable_platooning = true;
	toggle_msg.inner_platoon_distance = 1;
	toggle_msg.platoon_speed = 5;

	auto v1 = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	v1->message_type = REMOTE_PLATOONINGTOGGLE;
	v1->payload = MessageTypes::encode_message(toggle_msg);

	pub_map_[topics::OUT_PLATOONING_MSG].publish(v1);

	boost::this_thread::sleep_for(boost::chrono::seconds(1));

	//VEHICLE 2
	toggle_msg.vehicle_id = 2;
	toggle_msg.lvfv = "FV";

	auto v2 = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	v2->message_type = REMOTE_PLATOONINGTOGGLE;
	v2->payload = MessageTypes::encode_message(toggle_msg);

	pub_map_[topics::OUT_PLATOONING_MSG].publish(v2);

	boost::this_thread::sleep_for(boost::chrono::seconds(1));

	//VEHICLE 3
	auto v3 = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	v3->message_type = REMOTE_PLATOONINGTOGGLE;
	v3->payload = MessageTypes::encode_message(toggle_msg);

	pub_map_[topics::OUT_PLATOONING_MSG].publish(v3);

	//wait for slowpokes
	boost::this_thread::sleep_for(boost::chrono::seconds(5));

	TestResult res;
	res.success = true;
	res.comment = "";

	if (bclist.empty()) {
		res.success = false;
		res.comment += "no broadcast from lv with vehicle id 1 received\n";
	}

	if (fv2hb.empty()) {
		res.success = false;
		res.comment += "no heartbeat from fv with vehicle id 2 received\n";
	}

	if (fv2hb.empty()) {
		res.success = false;
		res.comment += "no heartbeat from fv with vehicle id 3 received\n";
	}

	//cleanup
	cleanup_test_send_platooningToggle_recv_heartbeats_and_broadcast();

	finalize_test(res);
}

void Integrationtest_platooning::cleanup_test_send_platooningToggle_recv_heartbeats_and_broadcast() {

	//send togglemessages in reverse order

	platooningToggle toggle_msg;

	toggle_msg.platoon_speed = 5;
	toggle_msg.vehicle_id = 2;
	toggle_msg.lvfv = "FV";
	toggle_msg.enable_platooning = false;
	toggle_msg.inner_platoon_distance = 1;
	toggle_msg.platoon_speed = 5;

	auto v2 = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	v2->message_type = REMOTE_PLATOONINGTOGGLE;
	v2->payload = MessageTypes::encode_message(toggle_msg);

	pub_map_[topics::OUT_PLATOONING_MSG].publish(v2);

	toggle_msg.vehicle_id = 3;
	auto v3 = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	v3->message_type = REMOTE_PLATOONINGTOGGLE;
	v3->payload = MessageTypes::encode_message(toggle_msg);
	pub_map_[topics::OUT_PLATOONING_MSG].publish(v3);

	boost::this_thread::sleep_for(boost::chrono::seconds(2));

	toggle_msg.vehicle_id = 1;
	toggle_msg.lvfv = "LV";

	auto v1 = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	v1->message_type = REMOTE_PLATOONINGTOGGLE;
	v1->payload = MessageTypes::encode_message(toggle_msg);
	pub_map_[topics::OUT_PLATOONING_MSG].publish(v1);

	boost::this_thread::sleep_for(boost::chrono::seconds(2));

	//cleanup local state
	reset();
}

void Integrationtest_platooning::test_send_platooningToggle_recv_heartbeat_data_and_broadcast_data() {
	set_timeout(boost::posix_time::time_duration(boost::posix_time::seconds(20)));
	set_current_test("test_send_platooningToggle_recv_heartbeat_data_and_broadcast_data");

	boost::function<void()> cbfun(boost::bind(boost::mem_fn(
		&Integrationtest_platooning::cleanup_test_send_platooningToggle_recv_heartbeat_data_and_broadcast_data), this));

	register_timeout_callback(cbfun);

	//mockup publishers
	pub_map_.emplace(topics::OUT_PLATOONING_MSG,
	                 nh_.advertise<platooning::platoonProtocol>(topics::OUT_PLATOONING_MSG, 1));

	while (pub_map_[topics::OUT_PLATOONING_MSG].getNumSubscribers() < 1) {
		boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
	}

	//mockup subscriber to velocity to catch calculated velocity
	sub_map_.emplace(topics::IN_FV_HEARTBEAT, nh_.subscribe(topics::IN_FV_HEARTBEAT, 1,
	                                                        &Integrationtest_platooning::hndl_fv_heartbeat,
	                                                        this));
	sub_map_.emplace(topics::IN_LV_BROADCAST, nh_.subscribe(topics::IN_LV_BROADCAST, 1,
	                                                        &Integrationtest_platooning::hndl_lv_broadcast,
	                                                        this));

	//create toggle messages for all three vehicles, change only relevant fields
	//VEHICLE 1
	platooningToggle toggle_msg;
	toggle_msg.vehicle_id = 1;
	toggle_msg.lvfv = "LV";
	toggle_msg.enable_platooning = true;
	toggle_msg.inner_platoon_distance = 10.0f;
	toggle_msg.platoon_speed = 1.0f;

	auto v1 = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	v1->message_type = REMOTE_PLATOONINGTOGGLE;
	v1->payload = MessageTypes::encode_message(toggle_msg);

	pub_map_[topics::OUT_PLATOONING_MSG].publish(v1);

	boost::this_thread::sleep_for(boost::chrono::seconds(1));

	//VEHICLE 2
	toggle_msg.vehicle_id = 2;
	toggle_msg.lvfv = "FV";

	auto v2 = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	v2->message_type = REMOTE_PLATOONINGTOGGLE;
	v2->payload = MessageTypes::encode_message(toggle_msg);

	pub_map_[topics::OUT_PLATOONING_MSG].publish(v2);

	boost::this_thread::sleep_for(boost::chrono::seconds(1));

	//VEHICLE 3
	auto v3 = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	v3->message_type = REMOTE_PLATOONINGTOGGLE;
	v3->payload = MessageTypes::encode_message(toggle_msg);

	pub_map_[topics::OUT_PLATOONING_MSG].publish(v3);

	//wait for slowpokes
	boost::this_thread::sleep_for(boost::chrono::seconds(5));

	TestResult res;
	res.success = true;
	res.comment = "";

	//broadcasts
	if (bclist.empty()) {
		res.success = false;
		res.comment += "no broadcast from lv with vehicle id 1 received\n";
	}

	// heartbeats
	if (fv2hb.empty()) {
		res.success = false;
		res.comment += "no heartbeat from fv with vehicle id 2 received\n";
	}

	if (fv3hb.empty()) {
		res.success = false;
		res.comment += "no heartbeat from fv with vehicle id 3 received\n";
	}

	//lv ui
	if (lvui.empty()) {
		res.success = false;
		res.comment += "no ui message from lv with vehicle id 1 received\n";
	}

	bool running_found = false;
	bool member2_found = false;
	bool member3_found = false;
	bool ipd10_found = false;
	bool ps1_found = false;
	for (auto &item : lvui) {
		if (item.second.platooning_state == "RUNNING") {
			running_found = true;
		}

		if (std::find(item.second.platoon_members.begin(), item.second.platoon_members.end(), 2)
			!= item.second.platoon_members.end()) {
			member2_found = true;
		}

		if (std::find(item.second.platoon_members.begin(), item.second.platoon_members.end(), 3)
			!= item.second.platoon_members.end()) {
			member3_found = true;
		}
	}

	if (!running_found) {
		res.success = false;
		res.comment += "lv not RUNNING. lv with vehicle id 1\n";
	}

	if (!member2_found) {
		res.success = false;
		res.comment += "lv doesnt have follower 2. lv with vehicle id 1\n";
	}

	if (!member3_found) {
		res.success = false;
		res.comment += "lv doesnt have follower 3. lv with vehicle id 1\n";
	}

	//fv 2 ui
	if (fv2ui.empty()) {
		res.success = false;
		res.comment += "no ui message from fv with vehicle id 2 received\n";
	}

	running_found = false;
	member2_found = false;
	member3_found = false;
	ipd10_found = false;
	ps1_found = false;
	for (auto &item : fv2ui) {
		if (item.second.platooning_state == "RUNNING") {
			running_found = true;
		}

		if (std::find(item.second.platoon_members.begin(), item.second.platoon_members.end(), 2)
			!= item.second.platoon_members.end()) {
			member2_found = true;
		}

		if (std::find(item.second.platoon_members.begin(), item.second.platoon_members.end(), 3)
			!= item.second.platoon_members.end()) {
			member3_found = true;
		}

		if (item.second.platoon_speed == 10.0f) {
			ipd10_found = true;
		}

		if (item.second.inner_platoon_distance == 1.0f) {
			ps1_found = true;
		}
	}

	if (!running_found) {
		res.success = false;
		res.comment += "fv not RUNNING. fv with vehicle id 2\n";
	}

	if (!member2_found) {
		res.success = false;
		res.comment += "fv doesnt have follower 2. fv with vehicle id 2\n";
	}

	if (!member3_found) {
		res.success = false;
		res.comment += "fv doesnt have follower 3. fv with vehicle id 2\n";
	}

	if (!ipd10_found) {
		res.success = false;
		res.comment += "fv doesnt have correct ipd. fv with vehicle id 2\n";
	}

	if (!ps1_found) {
		res.success = false;
		res.comment += "fv doesnt have correct ps. fv with vehicle id 2\n";
	}

	//fv3 ui
	if (fv3ui.empty()) {
		res.success = false;
		res.comment += "no ui message from fv with vehicle id 2 received\n";
	}

	running_found = false;
	member2_found = false;
	member3_found = false;
	ipd10_found = false;
	ps1_found = false;
	for (auto &item : fv3ui) {
		if (item.second.platooning_state == "RUNNING") {
			running_found = true;
		}

		if (std::find(item.second.platoon_members.begin(), item.second.platoon_members.end(), 2)
			!= item.second.platoon_members.end()) {
			member2_found = true;
		}

		if (std::find(item.second.platoon_members.begin(), item.second.platoon_members.end(), 3)
			!= item.second.platoon_members.end()) {
			member3_found = true;
		}

		if (item.second.platoon_speed == 10.0f) {
			ipd10_found = true;
		}

		if (item.second.inner_platoon_distance == 1.0f) {
			ps1_found = true;
		}
	}

	if (!running_found) {
		res.success = false;
		res.comment += "fv not RUNNING. fv with vehicle id 3\n";
	}

	if (!member2_found) {
		res.success = false;
		res.comment += "fv doesnt have follower 2. fv with vehicle id 3\n";
	}

	if (!member3_found) {
		res.success = false;
		res.comment += "fv doesnt have follower 3. fv with vehicle id 3\n";
	}

	if (!ipd10_found) {
		res.success = false;
		res.comment += "fv doesnt have correct ipd. fv with vehicle id 3\n";
	}

	if (!ps1_found) {
		res.success = false;
		res.comment += "fv doesnt have correct ps. fv with vehicle id 3\n";
	}

	//cleanup
	cleanup_test_send_platooningToggle_recv_heartbeat_data_and_broadcast_data();

	finalize_test(res);
}

void Integrationtest_platooning::cleanup_test_send_platooningToggle_recv_heartbeat_data_and_broadcast_data() {

	platooningToggle toggle_msg;

	//send togglemessages in reverse order
	toggle_msg.vehicle_id = 2;
	toggle_msg.lvfv = "FV";
	toggle_msg.enable_platooning = false;
	toggle_msg.inner_platoon_distance = 1;
	toggle_msg.platoon_speed = 5;

	auto v2 = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	v2->message_type = REMOTE_PLATOONINGTOGGLE;
	v2->payload = MessageTypes::encode_message(toggle_msg);

	pub_map_[topics::OUT_PLATOONING_MSG].publish(v2);

	toggle_msg.vehicle_id = 3;

	auto v3 = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	v3->message_type = REMOTE_PLATOONINGTOGGLE;
	v3->payload = MessageTypes::encode_message(toggle_msg);
	pub_map_[topics::OUT_PLATOONING_MSG].publish(v3);

	boost::this_thread::sleep_for(boost::chrono::seconds(2));

	toggle_msg.vehicle_id = 1;
	toggle_msg.lvfv = "LV";

	auto v1 = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	v1->message_type = REMOTE_PLATOONINGTOGGLE;
	v1->payload = MessageTypes::encode_message(toggle_msg);
	pub_map_[topics::OUT_PLATOONING_MSG].publish(v1);

	reset();

	boost::this_thread::sleep_for(boost::chrono::seconds(2));
}

void Integrationtest_platooning::test_send_updated_broadcast_receive_userinterface() {
	set_timeout(boost::posix_time::time_duration(boost::posix_time::seconds(20)));
	set_current_test("test_send_updated_broadcast_receive_userinterface");

	boost::function<void()> cbfun(boost::bind(boost::mem_fn(
		&Integrationtest_platooning::cleanup_test_send_updated_broadcast_receive_userinterface), this));

	register_timeout_callback(cbfun);

	//mockup publishers
	pub_map_.emplace(topics::OUT_PLATOONING_MSG, nh_.advertise<platoonProtocol>(topics::OUT_PLATOONING_MSG, 1));
	pub_map_.emplace(topics::TOGGLE_PLATOONING, nh_.advertise<platooningToggle>(topics::TOGGLE_PLATOONING, 1));

	while (pub_map_[topics::OUT_PLATOONING_MSG].getNumSubscribers() < 1) {
		boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
	}

	//mockup subscriber to velocity to catch calculated velocity
	sub_map_.emplace(topics::IN_FV_HEARTBEAT, nh_.subscribe(topics::IN_FV_HEARTBEAT, 1,
	                                                        &Integrationtest_platooning::hndl_fv_heartbeat,
	                                                        this));

	//setup platooningnode to LV as vehicle 1
	nh_.setParam("/vehicle_id", 1);
	Platooning platooningnode;
	platooningnode.onInit();

	//create toggle messages for all three vehicles, change only relevant fields
	//VEHICLE 1
	platooningToggle toggle_msg;
	toggle_msg.vehicle_id = 1;
	toggle_msg.lvfv = "LV";
	toggle_msg.enable_platooning = true;
	toggle_msg.inner_platoon_distance = 10.0f;
	toggle_msg.platoon_speed = 1.0f;

	auto v1 = boost::make_shared<platooningToggle>(toggle_msg);
	pub_map_[topics::TOGGLE_PLATOONING].publish(v1);

	boost::this_thread::sleep_for(boost::chrono::seconds(1));

	//VEHICLE 2
	toggle_msg.vehicle_id = 2;
	toggle_msg.lvfv = "FV";

	auto v2 = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	v2->message_type = REMOTE_PLATOONINGTOGGLE;
	v2->payload = MessageTypes::encode_message(toggle_msg);

	pub_map_[topics::OUT_PLATOONING_MSG].publish(v2);

	boost::this_thread::sleep_for(boost::chrono::seconds(1));

	//VEHICLE 3
	auto v3 = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	v3->message_type = REMOTE_PLATOONINGTOGGLE;
	v3->payload = MessageTypes::encode_message(toggle_msg);

	boost::this_thread::sleep_for(boost::chrono::seconds(1));

	//send updated platooningtoggle every 50ms
	toggle_msg.vehicle_id = 1;
	toggle_msg.lvfv = "LV";

	int i = 0;
	while (i++ <= 20) {
		toggle_msg.inner_platoon_distance += 1.0f;
		toggle_msg.platoon_speed += 0.1f;
		pub_map_[topics::TOGGLE_PLATOONING].publish(toggle_msg);
		boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
	}

	//check whether uimsg from fv2 and fv3 have the updated targetspeeds by counting different speeds.
	std::set<float> different_ps_fv2;
	std::set<float> different_ipd_fv2;
	std::list<std::pair<boost::posix_time::ptime, platooning::userInterface>> fv2cp = fv2ui;

	for (auto &x : fv2cp) {
		different_ps_fv2.insert(x.second.platoon_speed);
		different_ipd_fv2.insert(x.second.inner_platoon_distance);
	}

	std::set<float> different_ps_fv3;
	std::set<float> different_ipd_fv3;
	std::list<std::pair<boost::posix_time::ptime, platooning::userInterface>> fv3cp = fv3ui;

	for (auto &x : fv3cp) {
		different_ps_fv3.insert(x.second.platoon_speed);
		different_ipd_fv3.insert(x.second.inner_platoon_distance);
	}

	TestResult res;
	res.success = true;
	res.comment = "";

	if (different_ps_fv2.size() < 20) {
		res.success = false;
		res.comment = "fv2 reported less than 20 different ps values. values were ";

		for (auto &x : different_ps_fv2) {
			res.comment += std::to_string(x) + ",";
		}
		res.comment += "\n";
	}

	if (different_ipd_fv2.size() < 20) {
		res.success = false;
		res.comment = "fv2 reported less than 20 different ipd values. values were ";

		for (auto &x : different_ipd_fv2) {
			res.comment += std::to_string(x) + ",";
		}
		res.comment += "\n";
	}

	if (different_ps_fv3.size() < 20) {
		res.success = false;
		res.comment = "fv3 reported less than 20 different ps values. values were ";

		for (auto &x : different_ps_fv3) {
			res.comment += std::to_string(x) + ",";
		}
		res.comment += "\n";
	}

	if (different_ipd_fv3.size() < 20) {
		res.success = false;
		res.comment = "fv3 reported less than 20 different ipd values. values were ";

		for (auto &x : different_ipd_fv3) {
			res.comment += std::to_string(x) + ",";
		}
		res.comment += "\n";
	}

	cleanup_test_send_updated_broadcast_receive_userinterface();

	finalize_test(res);

}

void Integrationtest_platooning::cleanup_test_send_updated_broadcast_receive_userinterface() {
	platooningToggle toggle_msg;

	//send togglemessages in reverse order
	toggle_msg.vehicle_id = 2;
	toggle_msg.lvfv = "FV";
	toggle_msg.enable_platooning = false;
	toggle_msg.inner_platoon_distance = 1;
	toggle_msg.platoon_speed = 5;

	auto v2 = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	v2->message_type = REMOTE_PLATOONINGTOGGLE;
	v2->payload = MessageTypes::encode_message(toggle_msg);

	pub_map_[topics::OUT_PLATOONING_MSG].publish(v2);

	toggle_msg.vehicle_id = 3;

	auto v3 = boost::shared_ptr<platoonProtocol>(new platoonProtocol);
	v3->message_type = REMOTE_PLATOONINGTOGGLE;
	v3->payload = MessageTypes::encode_message(toggle_msg);
	pub_map_[topics::OUT_PLATOONING_MSG].publish(v3);

	boost::this_thread::sleep_for(boost::chrono::seconds(2));

	nh_.setParam("/vehicle_id", 0);
	reset();
}

void Integrationtest_platooning::hndl_lv_broadcast(const lv_broadcast &msg) {
	std::cout << "fv1bc" << std::endl;

	if (msg.src_vehicle == 1) {
		std::cout << "fv1bc" << std::endl;
		std::pair<boost::posix_time::ptime, lv_broadcast> tmp(boost::posix_time::microsec_clock::local_time(), msg);
		bclist.push_back(tmp);
	}
}

void Integrationtest_platooning::hndl_fv_heartbeat(const fv_heartbeat &msg) {
	if (msg.src_vehicle == 2) {
		std::cout << "fv2hb" << std::endl;

		auto tmp =
			std::pair<boost::posix_time::ptime, fv_heartbeat>(boost::posix_time::microsec_clock::local_time(), msg);
		fv2hb.push_back(tmp);
	}

	if (msg.src_vehicle == 3) {
		std::cout << "fv3hb" << std::endl;

		auto tmp =
			std::pair<boost::posix_time::ptime, fv_heartbeat>(boost::posix_time::microsec_clock::local_time(), msg);
		fv3hb.push_back(tmp);
	}
}

void Integrationtest_platooning::reset() {

	bclist.clear();
	fv2hb.clear();
	fv3hb.clear();
	lvui.clear();
	fv2ui.clear();
	fv3ui.clear();

	//heartbeat timer
	fv_heartbeat_sender_.cancel();
	fv_heartbeat_checker_.cancel();
	lv_broadcast_sender_.cancel();
	lv_broadcast_checker_.cancel();

	io_service_.stop();

	thread_pool_.interrupt_all();

	thread_pool_.join_all();

	thread_pool_.create_thread([this] { io_service_.run(); });

	//follower list and timeouts;
	fv_heartbeat_timeout_tracker_.clear();
	lv_broadcast_timeout_tracker_.first = 0;
	lv_broadcast_timeout_tracker_.second = boost::posix_time::microsec_clock::local_time();
}

void Integrationtest_platooning::receive_udp_message(const boost::shared_ptr<std::pair<std::string,
                                                                                       uint32_t>> &msgpair) {

	if (msgpair->second == REMOTE_USERINTERFACE) {
		userInterface msg = MessageTypes::decode_json<userInterface>(msgpair->first);
		if (msg.src_vehicle == 1) {
			auto tmp =
				std::pair<boost::posix_time::ptime, userInterface>(boost::posix_time::microsec_clock::local_time(),
				                                                   msg);
			lvui.push_back(tmp);
		}

		if (msg.src_vehicle == 2) {
			auto tmp =
				std::pair<boost::posix_time::ptime, userInterface>(boost::posix_time::microsec_clock::local_time(),
				                                                   msg);
			fv2ui.push_back(tmp);
		}

		if (msg.src_vehicle == 3) {
			auto tmp =
				std::pair<boost::posix_time::ptime, userInterface>(boost::posix_time::microsec_clock::local_time(),
				                                                   msg);
			fv3ui.push_back(tmp);
		}
	}

}

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Integrationtest_platooning, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
