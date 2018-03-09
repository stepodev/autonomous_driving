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
#include <platooning/distanceToObj.h>
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

	sub_targetAngle_ = nh_.subscribe(topics::TARGET_ANGLE, 1,
	                                 &StmSim::hndl_targetAngle, this);

	sub_targetSpeed_ = nh_.subscribe(topics::TARGET_SPEED, 1,
	                                 &StmSim::hndl_targetSpeed, this);

	pub_current_speed_ = nh_.advertise<platooning::speed>(topics::SPEED, 1);
	pub_distanceToObj_ = nh_.advertise<platooning::distance>(topics::DISTANCE_TO_OBJECT, 1);


	try {
		boost::function<void(std::pair<std::string, uint32_t>)> cbfun(boost::bind(boost::mem_fn(
			&StmSim::hndl_gazupdate), this, _1));

		server_ptr_ = std::unique_ptr<UdpServer>(
			new UdpServer(
				cbfun, udp::endpoint(udp::v4(), 13001), udp::endpoint(boost::asio::ip::address_v4::broadcast(), 13000)));
		server_ptr_->set_filter_own_broadcasts(false);

	} catch (std::exception &e) {
		NODELET_FATAL(std::string("[" + name_ + "] udpserver init failed\n" + std::string(e.what())).c_str());
	}

	target_angle_ = 0;
	target_speed_ = 0;
	vehicle_id_ = 1;
}


/*****************************************************************************
** Handlers
*****************************************************************************/
void StmSim::hndl_gazupdate(std::pair<std::string, uint32_t> message_pair) {

	if( message_pair.second == GAZ_UPDATE ) {

		platooning::gazupdate gazmsg;
		decode_json(message_pair.first, gazmsg);

		if( gazmsg.id == vehicle_id_ ) {
			auto speedmsg = boost::shared_ptr<platooning::speed>( new platooning::speed );
			speedmsg->speed = gazmsg.speed;
			pub_current_speed_.publish(speedmsg);

			auto distancemsg = boost::shared_ptr<platooning::distanceToObj>( new platooning::distanceToObj );
			distancemsg->distance_to_obj = gazmsg.distance;
			pub_distanceToObj_.publish(distancemsg);
		}
	}

}
void StmSim::hndl_targetSpeed(const platooning::targetSpeed &msg) {
	if( msg.target_speed != target_speed_ ) {
		platooning::stmupdate outmsg;
		outmsg.id = vehicle_id_;
		outmsg.steeringAngle = target_angle_;
		outmsg.acceleration = target_speed_;

		std::string msgstr = encode_message( outmsg );

		server_ptr_->start_send(msgstr, STMSIM_UPDATE );
	}

}

void StmSim::hndl_targetAngle(const platooning::targetAngle &msg) {
	if( msg.steering_angle != target_angle_ ) {
		platooning::stmupdate outmsg;
		outmsg.id = vehicle_id_;
		outmsg.steeringAngle = target_angle_;
		outmsg.acceleration = target_speed_;

		std::string msgstr = encode_message( outmsg );

		server_ptr_->start_send(msgstr, STMSIM_UPDATE );
	}
}

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::StmSim, nodelet::Nodelet);
// %EndTag(FULLTEXT)%