//
// Created by stepo on 12/16/17.
//


/**
 * @file /platooning/src/platooning.cpp
 *
 * @brief Nodelet implementation of RemoteContol
 *
 * @author stepo
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include "platooning/StmSim.hpp"

namespace platooning {


/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

StmSim::StmSim() {};

/*****************************************************************************
** Destructors
*****************************************************************************/

StmSim::~StmSim() {};


/*****************************************************************************
** Initializers
*****************************************************************************/

/**
* Set-up necessary publishers/subscribers
*/
void StmSim::onInit() {

	sub_steering_ = nh_.subscribe(topics::STEERING_ANGLE, 1,
	                              &StmSim::hndl_steeringAngle, this);

	sub_accel_ = nh_.subscribe(topics::ACCELERATION, 1,
	                           &StmSim::hndl_acceleration, this);

	pub_current_speed_ = nh_.advertise<platooning::speed>(topics::CURRENT_SPEED, 1, true);
	pub_distanceToObj_ = nh_.advertise<platooning::distance>(topics::SENSOR_DISTANCE_TO_OBJ, 1, true);

	try {
		boost::function<void(boost::shared_ptr<std::pair<std::string, uint32_t>>)> cbfun(boost::bind(boost::mem_fn(
			&StmSim::hndl_gazupdate), this, _1));

		server_ptr_ = std::unique_ptr<UdpServer>(
			new UdpServer(
				cbfun,
				udp::endpoint(udp::v4(), 13001),
				udp::endpoint(boost::asio::ip::address_v4::broadcast(), 13000)));
		server_ptr_->set_filter_own_broadcasts(false);

	} catch (std::exception &e) {
		NODELET_FATAL("[%s] udpserver init failed with %s", name_.c_str(), e.what());
	}

	steering_angle_ = 0;
	acceleration_ = 0;
	vehicle_id_ = 1;

	thread_pool_.create_thread([this] {

		try {
			//Services
			ros::ServiceClient
				srv_client_ = nh_.serviceClient<platooning::getVehicleId>(platooning_services::VEHICLE_ID);

			ros::Duration sec;
			sec.sec = 20;
			if (srv_client_.waitForExistence(ros::Duration(sec))) {

				platooning::getVehicleId::Request req;
				platooning::getVehicleId::Response res;

				if (srv_client_.call(req, res)) {
					this->vehicle_id_ = res.vehicle_id;
				}
			}
		}
		catch (std::exception &ex) {
			NODELET_ERROR("[%s] hndl_gazupdate failed with %s ", name_.c_str(), ex.what());
		}

	});

	NODELET_INFO("[%s] init done", name_.c_str());

}

/*****************************************************************************
** Handlers
*****************************************************************************/
void StmSim::hndl_gazupdate(boost::shared_ptr<std::pair<std::string, uint32_t>> message_pair) {

	try {

		if (message_pair->second == GAZ_UPDATE) {

			platooning::gazupdate gazmsg;
			MessageTypes::decode_json(message_pair->first, gazmsg);

			if (gazmsg.id == vehicle_id_) {
				auto speedmsg = boost::shared_ptr<platooning::speed>(new platooning::speed);
				speedmsg->speed = gazmsg.speed;
				pub_current_speed_.publish(speedmsg);

				auto distancemsg = boost::shared_ptr<platooning::distance>(new platooning::distance);
				distancemsg->distance = gazmsg.distance;
				pub_distanceToObj_.publish(distancemsg);
			}
		}
	}
	catch (std::exception &ex) {
		NODELET_ERROR("[%s] hndl_gazupdate failed with %s ", name_.c_str(), ex.what());
	}

}
void StmSim::hndl_acceleration(const platooning::acceleration &msg) {
	try {
		acceleration_ = msg.accelleration;

		platooning::stmupdate outmsg;
		outmsg.id = vehicle_id_;
		outmsg.steeringAngle = steering_angle_;
		outmsg.acceleration = acceleration_;

		std::string msgstr = MessageTypes::encode_message(outmsg);

		server_ptr_->start_send(msgstr, STMSIM_UPDATE);

	} catch (std::exception &ex) {
		NODELET_ERROR("[%s] hndl_targetSpeed failed with %s ", name_.c_str(), ex.what());
	}

}

void StmSim::hndl_steeringAngle(const platooning::steeringAngle &msg) {
	try {

		steering_angle_ = msg.steering_angle;

		platooning::stmupdate outmsg;
		outmsg.id = vehicle_id_;
		outmsg.steeringAngle = steering_angle_;
		outmsg.acceleration = acceleration_;

		std::string msgstr = MessageTypes::encode_message(outmsg);

		server_ptr_->start_send(msgstr, STMSIM_UPDATE);
	} catch (std::exception &ex) {
		NODELET_ERROR("[%s] hndl_targetAngle failed with %s ", name_.c_str(), ex.what());
	}
}

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::StmSim, nodelet::Nodelet);
// %EndTag(FULLTEXT)%