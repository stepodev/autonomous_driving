/**
 * @file /testing/src/Moduletest_messagetypes.cpp
 *
 * @brief Moduletest nodelet for messagetypes
 *
 * @author stepo
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "platooning/Moduletest_messagetypes.hpp"

// %Tag(FULLTEXT)%
namespace platooning {

/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

Moduletest_messagetypes::Moduletest_messagetypes() = default;

/*****************************************************************************
** Destructors
*****************************************************************************/

Moduletest_messagetypes::~Moduletest_messagetypes() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/

/**
* Set-up necessary publishers/subscribers
* @return true, if successful
*/
void Moduletest_messagetypes::onInit() {

	name_ = "Moduletest_messagetypes";

	register_testcases(boost::bind(&Moduletest_messagetypes::test_encode_decode_messages, this));

	NODELET_INFO("[%s ] init done", name_.c_str());

	start_tests();
}

/*****************************************************************************
** Testcases
*****************************************************************************/

void Moduletest_messagetypes::test_encode_decode_messages() {

	set_current_test("test_encode_decode_messages");

	TestResult res;
	res.success = true;
	res.comment = "";

	try {
		platooning::remotecontrolToggle origmsg;
		origmsg.vehicle_id = 3;
		origmsg.enable_remotecontrol = true;

		std::string json = MessageTypes::encode_message(origmsg);
		platooning::remotecontrolToggle decmsg = MessageTypes::decode_json<platooning::remotecontrolToggle>(json);

		if (origmsg.vehicle_id != decmsg.vehicle_id) {
			res.success = false;
			res.comment += "remotecontrolToggle vehicle_id decode or encode wrong\n";
		}
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "remotecontrolToggle threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	try {
		platooning::userInterface origmsg;
		origmsg.remotecontrol_enabled = false;
		origmsg.leading_vehicle = false;
		origmsg.following_vehicle = false;
		origmsg.potential_following_vehicle = false;
		origmsg.platooning_state = "IDLE";
		origmsg.src_vehicle = 3;
		origmsg.platoon_size = 3;
		origmsg.inner_platoon_distance = 0;
		origmsg.platoon_speed = 0;
		origmsg.actual_distance = 0;
		origmsg.speed = 0;
		origmsg.platoon_id = 0;
		origmsg.platoon_members = {1, 2, 3};

		std::string json = MessageTypes::encode_message(origmsg);
		auto decmsg = MessageTypes::decode_json<platooning::userInterface>(json);

		if (origmsg.remotecontrol_enabled != decmsg.remotecontrol_enabled) {
			res.success = false;
			res.comment += "userInterface remotecontrol_enabled decode or encode wrong\n";
		}
		if (origmsg.leading_vehicle != decmsg.leading_vehicle) {
			res.success = false;
			res.comment += "userInterface leading_vehicle decode or encode wrong\n";
		}
		if (origmsg.following_vehicle != decmsg.following_vehicle) {
			res.success = false;
			res.comment += "userInterface following_vehicle decode or encode wrong\n";
		}
		if (origmsg.potential_following_vehicle != decmsg.potential_following_vehicle) {
			res.success = false;
			res.comment += "userInterface potential_following_vehicle decode or encode wrong\n";
		}
		if (origmsg.platooning_state != decmsg.platooning_state) {
			res.success = false;
			res.comment += "userInterface platooning_state decode or encode wrong\n";
		}
		if (origmsg.platoon_size != decmsg.platoon_size) {
			res.success = false;
			res.comment += "userInterface platoon_size decode or encode wrong\n";
		}
		if (origmsg.inner_platoon_distance != decmsg.inner_platoon_distance) {
			res.success = false;
			res.comment += "userInterface inner_platoon_distance decode or encode wrong\n";
		}
		if (origmsg.platoon_speed != decmsg.platoon_speed) {
			res.success = false;
			res.comment += "userInterface platoon_speed decode or encode wrong\n";
		}
		if (origmsg.actual_distance != decmsg.actual_distance) {
			res.success = false;
			res.comment += "userInterface actual_distance decode or encode wrong\n";
		}
		if (origmsg.speed != decmsg.speed) {
			res.success = false;
			res.comment += "userInterface speed decode or encode wrong\n";
		}
		if (origmsg.platoon_id != decmsg.platoon_id) {
			res.success = false;
			res.comment += "userInterface platoon_id decode or encode wrong\n";
		}

		for (auto x : origmsg.platoon_members) {
			if (std::find(decmsg.platoon_members.begin(), decmsg.platoon_members.end(), x)
				== decmsg.platoon_members.end()) {
				res.success = false;
				res.comment += "userInterface platoon_members from orig not found in dec.  decode or encode wrong\n";
			}
		}

		for (auto x : decmsg.platoon_members) {
			if (std::find(origmsg.platoon_members.begin(), origmsg.platoon_members.end(), x)
				== origmsg.platoon_members.end()) {
				res.success = false;
				res.comment += "userInterface platoon_members from dec not found in orig.decode or encode wrong\n";
			}
		}

	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "userInterface threw test encode/decode with " + std::string(ex.what()) + "\n";
	}
	try {
		platooning::lv_broadcast origmsg;
		origmsg.src_vehicle = 1;
		origmsg.platoon_id = 2;
		origmsg.ps = 3;
		origmsg.ipd = 4;
		origmsg.followers = {1, 2, 3};

		std::string json = MessageTypes::encode_message(origmsg);
		auto decmsg = MessageTypes::decode_json<platooning::lv_broadcast>(json);

		if (origmsg.src_vehicle != decmsg.src_vehicle) {
			res.success = false;
			res.comment += "lv_broadcast src_vehicle decode or encode wrong\n";
		}

		if (origmsg.platoon_id != decmsg.platoon_id) {
			res.success = false;
			res.comment += "lv_broadcast platoon_id decode or encode wrong\n";
		}

		if (origmsg.ps != decmsg.ps) {
			res.success = false;
			res.comment += "lv_broadcast ps decode or encode wrong\n";
		}

		if (origmsg.ipd != decmsg.ipd) {
			res.success = false;
			res.comment += "lv_broadcast ipd decode or encode wrong\n";
		}

		for (auto x : origmsg.followers) {
			if (std::find(decmsg.followers.begin(), decmsg.followers.end(), x) == decmsg.followers.end()) {
				res.success = false;
				res.comment += "lv_broadcast followers from orig not found in dec. decode or encode wrong\n";
			}
		}

		for (auto x : decmsg.followers) {
			if (std::find(origmsg.followers.begin(), origmsg.followers.end(), x) == origmsg.followers.end()) {
				res.success = false;
				res.comment += "lv_broadcast followers from dec not found in orig. decode or encode wrong\n";
			}
		}
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "lv_broadcast threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	try {
		platooning::lv_accept origmsg;
		origmsg.platoon_id = 1;
		origmsg.dst_vehicle = 2;
		origmsg.src_vehicle = 3;

		std::string json = MessageTypes::encode_message(origmsg);
		auto decmsg = MessageTypes::decode_json<platooning::lv_accept>(json);

		if (origmsg.src_vehicle != decmsg.src_vehicle) {
			res.success = false;
			res.comment += "lv_accept src_vehicle decode or encode wrong\n";
		}

		if (origmsg.dst_vehicle != decmsg.dst_vehicle) {
			res.success = false;
			res.comment += "lv_accept dst_vehicle decode or encode wrong\n";
		}

		if (origmsg.platoon_id != decmsg.platoon_id) {
			res.success = false;
			res.comment += "lv_accept platoon_id decode or encode wrong\n";
		}
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "lv_accept threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	try {
		platooning::lv_reject origmsg;
		origmsg.platoon_id = 1;
		origmsg.dst_vehicle = 2;
		origmsg.src_vehicle = 3;

		std::string json = MessageTypes::encode_message(origmsg);
		auto decmsg = MessageTypes::decode_json<platooning::lv_reject>(json);

		if (origmsg.src_vehicle != decmsg.src_vehicle) {
			res.success = false;
			res.comment += "lv_reject src_vehicle decode or encode wrong\n";
		}

		if (origmsg.dst_vehicle != decmsg.dst_vehicle) {
			res.success = false;
			res.comment += "lv_reject dst_vehicle decode or encode wrong\n";
		}

		if (origmsg.platoon_id != decmsg.platoon_id) {
			res.success = false;
			res.comment += "lv_reject platoon_id decode or encode wrong\n";
		}
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "lv_reject threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	try {
		platooning::fv_request origmsg;
		origmsg.src_vehicle = 3;

		std::string json = MessageTypes::encode_message(origmsg);
		auto decmsg = MessageTypes::decode_json<platooning::fv_request>(json);

		if (origmsg.src_vehicle != decmsg.src_vehicle) {
			res.success = false;
			res.comment += "fv_request src_vehicle decode or encode wrong\n";
		}
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "lv_reject threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	try {
		platooning::fv_heartbeat origmsg;
		origmsg.src_vehicle = 3;
		origmsg.platoon_id = 2;

		std::string json = MessageTypes::encode_message(origmsg);
		auto decmsg = MessageTypes::decode_json<platooning::fv_heartbeat>(json);

		if (origmsg.src_vehicle != decmsg.src_vehicle) {
			res.success = false;
			res.comment += "fv_heartbeat src_vehicle decode or encode wrong\n";
		}

		if (origmsg.platoon_id != decmsg.platoon_id) {
			res.success = false;
			res.comment += "fv_heartbeat platoon_id decode or encode wrong\n";
		}
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "fv_heartbeat threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	try {
		platooning::fv_leave origmsg;
		origmsg.src_vehicle = 3;
		origmsg.platoon_id = 2;

		std::string json = MessageTypes::encode_message(origmsg);
		auto decmsg = MessageTypes::decode_json<platooning::fv_leave>(json);

		if (origmsg.src_vehicle != decmsg.src_vehicle) {
			res.success = false;
			res.comment += "fv_leave src_vehicle decode or encode wrong\n";
		}

		if (origmsg.platoon_id != decmsg.platoon_id) {
			res.success = false;
			res.comment += "fv_leave platoon_id decode or encode wrong\n";
		}
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "fv_leave threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	try {
		platooning::platooningToggle origmsg;
		origmsg.vehicle_id = 3;
		origmsg.enable_platooning = true;
		origmsg.platoon_speed = 1;
		origmsg.inner_platoon_distance = 5;
		origmsg.lvfv = "LV";

		std::string json = MessageTypes::encode_message(origmsg);
		auto decmsg = MessageTypes::decode_json<platooning::platooningToggle>(json);

		if (origmsg.vehicle_id != decmsg.vehicle_id) {
			res.success = false;
			res.comment += "platooningToggle vehicle_id decode or encode wrong\n";
		}

		if (origmsg.enable_platooning != decmsg.enable_platooning) {
			res.success = false;
			res.comment += "platooningToggle enable_platooning decode or encode wrong\n";
		}

		if (origmsg.platoon_speed != decmsg.platoon_speed) {
			res.success = false;
			res.comment += "platooningToggle platoon_speed decode or encode wrong\n";
		}

		if (origmsg.inner_platoon_distance != decmsg.inner_platoon_distance) {
			res.success = false;
			res.comment += "platooningToggle inner_platoon_distance decode or encode wrong\n";
		}

		if (origmsg.lvfv != decmsg.lvfv) {
			res.success = false;
			res.comment += "platooningToggle lvfv decode or encode wrong\n";
		}
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "platooningToggle threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	try {
		platooning::platooningState origmsg;
		origmsg.vehicle_id = 1;
		origmsg.platoon_id = 2;
		origmsg.platooning_state = "IDLE";
		origmsg.ipd = 3;
		origmsg.ps = 4;
		origmsg.i_am_FV = false;
		origmsg.i_am_LV = false;
		//no encoders or decoders yet. internal msg
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "platooningState threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	try {
		platooning::remotecontrolInput origmsg;
		origmsg.vehicle_id = 1;
		origmsg.emergency_stop = true;
		origmsg.remote_angle = 1.3f;
		origmsg.remote_speed = 23.f;

		std::string json = MessageTypes::encode_message(origmsg);
		auto decmsg = MessageTypes::decode_json<platooning::remotecontrolInput>(json);

		if (origmsg.vehicle_id != decmsg.vehicle_id) {
			res.success = false;
			res.comment += "remotecontrolInput vehicle_id decode or encode wrong\n";
		}

		if (origmsg.emergency_stop != decmsg.emergency_stop) {
			res.success = false;
			res.comment += "remotecontrolInput emergency_stop decode or encode wrong\n";
		}

		if (origmsg.remote_angle != decmsg.remote_angle) {
			res.success = false;
			res.comment += "remotecontrolInput remote_angle decode or encode wrong\n";
		}

		if (origmsg.remote_speed != decmsg.remote_speed) {
			res.success = false;
			res.comment += "remotecontrolInput remote_speed decode or encode wrong\n";
		}
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "remotecontrolInput threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	try {
		platooning::acceleration origmsg;
		origmsg.accelleration = 2.1f;

		std::string json = MessageTypes::encode_message(origmsg);
		auto decmsg = MessageTypes::decode_json<platooning::acceleration>(json);

		if (origmsg.accelleration != decmsg.accelleration) {
			res.success = false;
			res.comment += "acceleration accelleration decode or encode wrong\n";
		}
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "acceleration threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	try {
		platooning::steeringAngle origmsg;
		origmsg.steering_angle = 2.1f;

		std::string json = MessageTypes::encode_message(origmsg);
		auto decmsg = MessageTypes::decode_json<platooning::steeringAngle>(json);

		if (origmsg.steering_angle != decmsg.steering_angle) {
			res.success = false;
			res.comment += "steeringAngle steering_angle decode or encode wrong\n";
		}
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "steeringAngle threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	try {
		platooning::gazupdate origmsg;
		origmsg.speed = 2.1f;
		origmsg.id = 5;
		origmsg.distance = 123.0f;

		std::string json = MessageTypes::encode_message(origmsg);
		auto decmsg = MessageTypes::decode_json<platooning::gazupdate>(json);

		if (origmsg.speed != decmsg.speed) {
			res.success = false;
			res.comment += "gazupdate speed decode or encode wrong\n";
		}

		if (origmsg.id != decmsg.id) {
			res.success = false;
			res.comment += "gazupdate speed id or encode wrong\n";
		}

		if (origmsg.distance != decmsg.distance) {
			res.success = false;
			res.comment += "gazupdate distance id or encode wrong\n";
		}
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "gazupdate threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	try {
		platooning::stmupdate origmsg;
		origmsg.id = 5;
		origmsg.acceleration = 1.2f;
		origmsg.steeringAngle = 1.5f;

		std::string json = MessageTypes::encode_message(origmsg);
		auto decmsg = MessageTypes::decode_json<platooning::stmupdate>(json);

		if (origmsg.id != decmsg.id) {
			res.success = false;
			res.comment += "stmupdate id decode or encode wrong\n";
		}

		if (origmsg.acceleration != decmsg.acceleration) {
			res.success = false;
			res.comment += "stmupdate acceleration decode or encode wrong\n";
		}

		if (origmsg.steeringAngle != decmsg.steeringAngle) {
			res.success = false;
			res.comment += "stmupdate steeringAngle decode or encode wrong\n";
		}
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "stmupdate threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	try {
		platooning::targetSpeed origmsg;
		origmsg.target_speed = 5.5f;

		//no encode or decode, internal
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "targetSpeed threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	try {
		platooning::targetAngle origmsg;
		origmsg.steering_angle = 5.5f;
		//no encode or decode, internal
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "targetAngle threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	try {
		platooning::targetDistance origmsg;
		origmsg.distance = 324.4f;
		//no encode or decode, internal
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "targetDistance threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	try {
		platooning::speed origmsg;
		origmsg.speed = 12.0f;
		//no encode or decode, internal
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "origmsg threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	try {
		platooning::distance origmsg;
		origmsg.distance = 12.0f;
		//no encode or decode, internal
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "origmsg threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	try {
		platooning::vehicleControl origmsg;
		//no encode or decode, internal
	} catch (std::exception &ex) {
		res.success = false;
		res.comment += "vehicleControl threw test encode/decode with " + std::string(ex.what()) + "\n";
	}

	finalize_test(res);

}

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Moduletest_messagetypes, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
