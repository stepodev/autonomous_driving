#include "platooning/MessageTypes.hpp"

namespace pt = boost::property_tree;

namespace platooning {

template<typename T>
std::vector<T>
MessageTypes::json_as_vector(boost::property_tree::ptree const &pt, boost::property_tree::ptree::key_type const &key) {
	std::vector<T> r;
	for (auto &item : pt.get_child(key))
		r.push_back(item.second.get_value<T>());
	return r;
}

template<typename T>
boost::property_tree::ptree
MessageTypes::vector_as_json(std::vector<T> const &v) {
	boost::property_tree::ptree root;

	for (auto &i : v) {
		boost::property_tree::ptree leaf;
		leaf.put("", i);
		root.push_back(std::make_pair("", leaf));
	}

	return root;
}

void MessageTypes::decode_json(const std::string &json, lv_broadcast& message) {
	std::stringstream ss(json);

	pt::ptree root;
	pt::read_json(ss, root);

	message.src_vehicle = root.get<uint32_t>("src_vehicle");
	message.platoon_id = root.get<uint32_t>("platoon_id");
	message.ipd = root.get<float>("ipd");
	message.ps = root.get<float>("ps");
	message.followers = MessageTypes::json_as_vector<uint32_t>(root, "followers");
}

void MessageTypes::decode_json(const std::string &json, lv_accept &message) {
	std::stringstream ss(json);

	pt::ptree root;
	pt::read_json(ss, root);

	message.src_vehicle = root.get<uint32_t>("src_vehicle");
	message.platoon_id = root.get<uint32_t>("platoon_id");
	message.dst_vehicle = root.get<uint32_t>("dst_vehicle");

}

void MessageTypes::decode_json(const std::string &json, lv_reject &message) {
	std::stringstream ss(json);

	pt::ptree root;
	pt::read_json(ss, root);

	message.src_vehicle = root.get<uint32_t>("src_vehicle");
	message.platoon_id = root.get<uint32_t>("platoon_id");
	message.dst_vehicle = root.get<uint32_t>("dst_vehicle");

}

void MessageTypes::decode_json(const std::string &json, fv_request &message) {
	std::stringstream ss(json);

	pt::ptree root;
	pt::read_json(ss, root);

	message.src_vehicle = root.get<uint32_t>("src_vehicle");
	//msgfields.ipd = root.get<float>("ipd");
	//msgfields.ps = root.get<float>("ps");

}

void MessageTypes::decode_json(const std::string &json, fv_heartbeat &message) {
	std::stringstream ss(json);

	pt::ptree root;
	pt::read_json(ss, root);

	message.src_vehicle = root.get<uint32_t>("src_vehicle");
	message.platoon_id = root.get<uint32_t>("platoon_id");

}

void MessageTypes::decode_json(const std::string &json, fv_leave &message) {
	std::stringstream ss(json);

	pt::ptree root;
	pt::read_json(ss, root);

	message.src_vehicle = root.get<uint32_t>("src_vehicle");
	message.platoon_id = root.get<uint32_t>("platoon_id");

}

void MessageTypes::decode_json(const std::string &json, remotecontrolInput &message) {
	std::stringstream ss(json);

	pt::ptree root;
	pt::read_json(ss, root);

	message.emergency_stop = root.get<bool>("emergency_stop");
	message.remote_angle = root.get<float>("remote_angle");
	message.remote_speed = root.get<float>("remote_speed");
	message.vehicle_id = root.get<uint32_t>("vehicle_id");
}

void MessageTypes::decode_json(const std::string &json, remotecontrolToggle &message) {
	std::stringstream ss(json);

	pt::ptree root;
	pt::read_json(ss, root);

	message.enable_remotecontrol = root.get<bool>("enable_remotecontrol");
	message.vehicle_id = root.get<uint32_t>("vehicle_id");

}

void MessageTypes::decode_json(const std::string &json, platooningToggle &message) {

	std::stringstream ss(json);

	pt::ptree root;
	pt::read_json(ss, root);

	message.enable_platooning = root.get<bool>("enable_platooning");
	message.vehicle_id = root.get<uint32_t>("vehicle_id");
	message.inner_platoon_distance = root.get<float>("inner_platoon_distance");
	message.platoon_speed = root.get<float>("platoon_speed");
	message.lvfv = root.get<std::string>("lvfv");

}

void MessageTypes::decode_json(const std::string &json, userInterface &message) {

	std::stringstream ss(json);

	pt::ptree root;
	pt::read_json(ss, root);

	message.remotecontrol_enabled = root.get<bool>("remotecontrol_enabled");
	message.leading_vehicle = root.get<bool>("leading_vehicle");
	message.following_vehicle = root.get<bool>("following_vehicle");
	message.potential_following_vehicle = root.get<bool>("potential_following_vehicle");
	message.platooning_state = root.get<std::string>("platooning_state");
	message.src_vehicle = root.get<uint32_t>("src_vehicle");
	message.platoon_members = MessageTypes::json_as_vector<uint32_t>(root, "platoon_members");
	message.platoon_size = message.platoon_members.size();
	message.inner_platoon_distance = root.get<float>("inner_platoon_distance");
	message.platoon_speed = root.get<float>("platoon_speed");
	message.actual_distance = root.get<float>("actual_distance");
	message.speed = root.get<float>("speed");
	message.platoon_id = root.get<uint32_t>("platoon_id");
}

void MessageTypes::decode_json(const std::string &json, stmupdate &message) {

	std::stringstream ss(json);

	pt::ptree root;
	pt::read_json(ss, root);

	message.id = root.get<uint32_t>("id");
	message.steeringAngle = root.get<float>("steeringAngle");
	message.acceleration = root.get<float>("acceleration");

}

void MessageTypes::decode_json(const std::string &json, gazupdate &message) {

	std::stringstream ss(json);

	pt::ptree root;
	pt::read_json(ss, root);

	message.id = root.get<uint32_t>("id");
	message.speed = root.get<float>("speed");
	message.distance = root.get<float>("distance");

}

void MessageTypes::decode_json(const std::string &json, steeringAngle &message) {

	std::stringstream ss(json);

	pt::ptree root;
	pt::read_json(ss, root);

	message.steering_angle = root.get<float>("steering_angle");
}

void MessageTypes::decode_json(const std::string &json, acceleration &message) {
	std::stringstream ss(json);

	pt::ptree root;
	pt::read_json(ss, root);

	message.accelleration = root.get<float>("accelleration");
}

void MessageTypes::decode_json(const std::string &json, vehicleControl &message) {
	std::stringstream ss(json);

	pt::ptree root;
	pt::read_json(ss, root);

	message.steering_angle = root.get<float>("steering_angle");
	message.velocity = root.get<float>("velocity");
}

std::string MessageTypes::encode_message(const lv_broadcast &message) {
	pt::ptree root;

	root.put("src_vehicle", message.src_vehicle);
	root.add_child("followers", MessageTypes::vector_as_json<uint32_t>(message.followers));
	root.put("ps", message.ps);
	root.put("ipd", message.ipd);
	root.put("platoon_id", message.platoon_id);

	std::stringstream ss;
	boost::property_tree::write_json(ss, root, false);

	return ss.str();
}

std::string MessageTypes::encode_message(const lv_accept &message) {
	pt::ptree root;

	root.put("platoon_id", message.platoon_id);
	root.put("src_vehicle", message.src_vehicle);
	root.put("dst_vehicle", message.dst_vehicle);

	std::stringstream ss;
	boost::property_tree::write_json(ss, root, false);

	return ss.str();
}

std::string MessageTypes::encode_message(const lv_reject &message) {
	pt::ptree root;

	root.put("platoon_id", message.platoon_id);
	root.put("src_vehicle", message.src_vehicle);
	root.put("dst_vehicle", message.dst_vehicle);

	std::stringstream ss;
	boost::property_tree::write_json(ss, root, false);

	return ss.str();
}

std::string MessageTypes::encode_message(const fv_heartbeat &message) {
	pt::ptree root;

	root.put("platoon_id", message.platoon_id);
	root.put("src_vehicle", message.src_vehicle);

	std::stringstream ss;
	boost::property_tree::write_json(ss, root, false);

	return ss.str();
}

std::string MessageTypes::encode_message(const fv_leave &message) {
	pt::ptree root;

	root.put("platoon_id", message.platoon_id);
	root.put("src_vehicle", message.src_vehicle);

	std::stringstream ss;
	boost::property_tree::write_json(ss, root, false);

	return ss.str();
}

std::string MessageTypes::encode_message(const fv_request &message) {
	pt::ptree root;

	root.put("src_vehicle", message.src_vehicle);

	std::stringstream ss;
	boost::property_tree::write_json(ss, root, false);

	return ss.str();
}

std::string MessageTypes::encode_message(const remotecontrolInput &message) {
	pt::ptree root;

	root.put("vehicle_id", message.vehicle_id);
	root.put("remote_angle", message.remote_angle);
	root.put("remote_speed", message.remote_speed);
	root.put("emergency_stop", message.emergency_stop);

	std::stringstream ss;
	boost::property_tree::write_json(ss, root, false);

	return ss.str();
}

std::string MessageTypes::encode_message(const remotecontrolToggle &message) {
	pt::ptree root;

	root.put("enable_remotecontrol", message.enable_remotecontrol);
	root.put("vehicle_id", message.vehicle_id);

	std::stringstream ss;
	boost::property_tree::write_json(ss, root, false);

	return ss.str();
}

std::string MessageTypes::encode_message(const platooningToggle &message) {
	pt::ptree root;

	root.put("vehicle_id", message.vehicle_id);
	root.put("enable_platooning", message.enable_platooning);
	root.put("inner_platoon_distance", message.inner_platoon_distance);
	root.put("platoon_speed", message.platoon_speed);
	root.put("lvfv", message.lvfv);

	std::stringstream ss;
	boost::property_tree::write_json(ss, root, false);

	return ss.str();
}

std::string MessageTypes::encode_message(const stmupdate &message) {
	pt::ptree root;

	root.put("id", message.id);
	root.put("acceleration", message.acceleration);
	root.put("steeringAngle", message.steeringAngle);

	std::stringstream ss;
	boost::property_tree::write_json(ss, root, false);

	return ss.str();
}

std::string MessageTypes::encode_message(const gazupdate &message) {
	pt::ptree root;

	root.put("id", message.id);
	root.put("distance", message.distance);
	root.put("speed", message.speed);

	std::stringstream ss;
	boost::property_tree::write_json(ss, root, false);

	return ss.str();
}

std::string MessageTypes::encode_message(const steeringAngle &message) {
	pt::ptree root;

	root.put("steering_angle", message.steering_angle);

	std::stringstream ss;
	boost::property_tree::write_json(ss, root, false);

	return ss.str();
}
std::string MessageTypes::encode_message(const acceleration &message) {
	pt::ptree root;

	root.put("accelleration", message.accelleration);

	std::stringstream ss;
	boost::property_tree::write_json(ss, root, false);

	return ss.str();
}

std::string MessageTypes::encode_message(const userInterface &message) {
	pt::ptree root;

	root.put("leading_vehicle", message.leading_vehicle);
	root.put("following_vehicle", message.following_vehicle);
	root.put("potential_following_vehicle", message.potential_following_vehicle);

	root.put("inner_platoon_distance", message.inner_platoon_distance);
	root.put("actual_distance", message.actual_distance);
	root.put("platoon_speed", message.platoon_speed);
	root.put("speed", message.speed);

	root.put("platooning_state", message.platooning_state);

	root.put("src_vehicle", message.src_vehicle);
	root.put("platoon_size", message.platoon_size);
	root.add_child("platoon_members", vector_as_json<uint32_t>(message.platoon_members));
	root.put("remotecontrol_enabled", message.remotecontrol_enabled);

	root.put("platoon_id", message.platoon_id);

	std::stringstream ss;
	boost::property_tree::write_json(ss, root, false);

	return ss.str();
}

std::string MessageTypes::encode_message(const vehicleControl &message) {
	pt::ptree root;

	root.put("velocity", message.velocity);
	root.put("steering_angle", message.steering_angle);

	std::stringstream ss;
	boost::property_tree::write_json(ss, root, false);

	return ss.str();

}

}