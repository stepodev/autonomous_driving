//
// Created by stepo on 3/4/18.
//

#include <platooning_sim/topics.hpp>
#include "platooning_sim/gazeboadapter.hpp"

namespace platooning_sim {

gazeboadapter::gazeboadapter() :
	gazupdate_send_timer_(io_service_) {

}

gazeboadapter::~gazeboadapter() {

}

void gazeboadapter::onInit() {

	p1gazupdate.id = 1;
	p2gazupdate.id = 2;
	p3gazupdate.id = 3;

	pub_p1_control_ = nh_.advertise<prius_msgs::Control>(topics::p1_control, 1);
	pub_p2_control_ = nh_.advertise<prius_msgs::Control>(topics::p2_control, 1);
	pub_p3_control_ = nh_.advertise<prius_msgs::Control>(topics::p3_control, 1);

	sub_p1_speed_ = nh_.subscribe(topics::p1_speed,1,&gazeboadapter::hndl_p1_speed, this);
	sub_p2_speed_ = nh_.subscribe(topics::p2_speed,1,&gazeboadapter::hndl_p2_speed, this);
	sub_p3_speed_ = nh_.subscribe(topics::p3_speed,1,&gazeboadapter::hndl_p3_speed, this);

	sub_p1_sonar_front_ = nh_.subscribe(topics::p1_front_sonar_left_middle_range,1,&gazeboadapter::hndl_p1_sonar,this);
	sub_p2_sonar_front_ = nh_.subscribe(topics::p2_front_sonar_left_middle_range,1,&gazeboadapter::hndl_p2_sonar,this);
	sub_p3_sonar_front_ = nh_.subscribe(topics::p3_front_sonar_left_middle_range,1,&gazeboadapter::hndl_p3_sonar,this);

	sub_p1_camera_front_ = nh_.subscribe(topics::p1_front_camera_image_raw,1,&gazeboadapter::hndl_p1_camera,this);
	sub_p2_camera_front_ = nh_.subscribe(topics::p2_front_camera_image_raw,1,&gazeboadapter::hndl_p2_camera,this);
	sub_p3_camera_front_ = nh_.subscribe(topics::p3_front_camera_image_raw,1,&gazeboadapter::hndl_p3_camera,this);

	boost::function<void(std::pair<std::string, uint32_t>)>
		cbfun(boost::bind(boost::mem_fn(&gazeboadapter::hndl_recv_udp), this, _1));

	server_ = std::unique_ptr<UdpServer>(
		new UdpServer(
			cbfun, udp::endpoint(udp::v4(), 13000), udp::endpoint(boost::asio::ip::address_v4::broadcast(), 13001)));
	server_->set_filter_own_broadcasts(false);


	keep_spinning_ = true;

	gazupdate_send_timer_.expires_from_now(BROADCAST_FREQ);
	gazupdate_send_timer_.async_wait(boost::bind(&gazeboadapter::send_gazupdate, this,
	                                             boost::asio::placeholders::error));

	thread_pool_.create_thread([this] {
		while (this->keep_spinning_) {
			io_service_.run();
		}
	});

	ROS_WARN("init done");
}

void gazeboadapter::hndl_recv_udp(std::pair<std::string, uint32_t> packet) {

	platooning::stmupdate msg;

	switch (packet.second) {
		case STMSIM_UPDATE: platooning::MessageTypes::decode_json(packet.first, msg);
			process_stmsim( msg );
			break;
		default: break;
	}

}
void gazeboadapter::hndl_p1_sonar(const sensor_msgs::Range &msg) {
	p1gazupdate.distance = msg.range;
}

void gazeboadapter::hndl_p1_camera(const sensor_msgs::Image &msg) {

}

void gazeboadapter::hndl_p1_speed(const prius_msgs::Speed &msg) {
	p1gazupdate.speed = msg.speed;
}

void gazeboadapter::hndl_p2_sonar(const sensor_msgs::Range &msg) {
	p2gazupdate.distance = msg.range;
}

void gazeboadapter::hndl_p2_camera(const sensor_msgs::Image &msg) {

}

void gazeboadapter::hndl_p2_speed(const prius_msgs::Speed &msg) {
	p2gazupdate.speed = msg.speed;
}

void gazeboadapter::hndl_p3_sonar(const sensor_msgs::Range &msg) {
	p3gazupdate.distance = msg.range;
}

void gazeboadapter::hndl_p3_camera(const sensor_msgs::Image &msg) {

}

void gazeboadapter::hndl_p3_speed(const prius_msgs::Speed &msg) {
	p3gazupdate.speed = msg.speed;
}

void gazeboadapter::send_gazupdate(const boost::system::error_code &e) {

	ROS_WARN("send_gazupdate");

	if (boost::asio::error::operation_aborted == e) {
		return;
	}

	std::string msg = platooning::MessageTypes::encode_message(p1gazupdate);
	server_->start_send( msg, GAZ_UPDATE );

	msg = platooning::MessageTypes::encode_message(p2gazupdate);
	server_->start_send( msg, GAZ_UPDATE );

	msg = platooning::MessageTypes::encode_message(p3gazupdate);
	server_->start_send( msg, GAZ_UPDATE );

	gazupdate_send_timer_.expires_from_now(BROADCAST_FREQ);
	gazupdate_send_timer_.async_wait(boost::bind(&gazeboadapter::send_gazupdate, this,
	                                            boost::asio::placeholders::error));

	if (io_service_.stopped()) {
		thread_pool_.create_thread([this] {
			while (this->keep_spinning_) {
				io_service_.run();
			}
		});
	}
}
void gazeboadapter::process_stmsim(const platooning::stmupdate &stmupdate) {

	auto c = boost::shared_ptr<prius_msgs::Control>( new prius_msgs::Control);

	//we dont reverse
	c->shift_gears = prius_msgs::Control::FORWARD;

	//maybe between +-0.87? handwheelhigh +-7.85?
	c->steer = stmupdate.steeringAngle;

	//assumes accel between -1 and 1
	if( stmupdate.acceleration < 0 ) {
		c->brake = stmupdate.acceleration * -1;
	} else if( stmupdate.acceleration == 0 ) {
		//dont do nothin
	} else if( stmupdate.acceleration > 0){
		c->throttle = stmupdate.acceleration;
	}

	if( stmupdate.id == 1 ) {
		pub_p1_control_.publish(c);
	}

	if( stmupdate.id == 2 ) {
		pub_p2_control_.publish(c);
	}

	if( stmupdate.id == 3 ) {
		pub_p3_control_.publish(c);
	}

}

}

PLUGINLIB_EXPORT_CLASS(platooning_sim::gazeboadapter, nodelet::Nodelet);
// %EndTag(FULLTEXT)%