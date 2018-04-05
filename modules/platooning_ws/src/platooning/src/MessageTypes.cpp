#include "platooning/MessageTypes.hpp"

namespace pt = boost::property_tree;
using json = nlohmann::json;

namespace platooning {

template <>
lv_broadcast MessageTypes::decode_json<lv_broadcast>(const std::string &jsonstr) {
	json root = json::parse(jsonstr);

	lv_broadcast message;

	message.src_vehicle = root["src_vehicle"].get<uint32_t>();
	message.platoon_id = root["platoon_id"].get<uint32_t>();
	message.ipd = root["ipd"].get<float>();
	message.ps = root["ps"].get<float>();
	message.followers = root["followers"].get<std::vector<uint32_t>>();

	return message;
}

template <>
lv_accept MessageTypes::decode_json(const std::string &jsonstr) {
	json root = json::parse(jsonstr);

	lv_accept message;

	message.src_vehicle = root["src_vehicle"].get<uint32_t>();
	message.platoon_id = root["platoon_id"].get<uint32_t>();
	message.dst_vehicle = root["dst_vehicle"].get<uint32_t>();

	return message;

}

template <>
lv_reject MessageTypes::decode_json(const std::string &jsonstr) {
	json root = json::parse(jsonstr);

	lv_reject message;

	message.src_vehicle = root["src_vehicle"].get<uint32_t>();
	message.platoon_id = root["platoon_id"].get<uint32_t>();
	message.dst_vehicle = root["dst_vehicle"].get<uint32_t>();

	return message;

}

template <>
fv_request MessageTypes::decode_json(const std::string &jsonstr) {
	json root = json::parse(jsonstr);

	fv_request message;

	message.src_vehicle = root["src_vehicle"].get<uint32_t>();
	//msgfields.ipd = root.get<float>("ipd");
	//msgfields.ps = root.get<float>("ps");

	return message;

}

template <>
fv_heartbeat MessageTypes::decode_json(const std::string &jsonstr) {
	json root = json::parse(jsonstr);

	fv_heartbeat message;

	message.src_vehicle = root["src_vehicle"].get<uint32_t>();
	message.platoon_id = root["platoon_id"].get<uint32_t>();

	return message;

}

template <>
fv_leave MessageTypes::decode_json(const std::string &jsonstr) {
	json root = json::parse(jsonstr);

	fv_leave message;

	message.src_vehicle = root["src_vehicle"].get<uint32_t>();
	message.platoon_id = root["platoon_id"].get<uint32_t>();

	return message;

}

template <>
remotecontrolInput MessageTypes::decode_json(const std::string &jsonstr) {
	json root = json::parse(jsonstr);

	remotecontrolInput message;

	message.emergency_stop = root["emergency_stop"].get<bool>();
	message.remote_angle = root["remote_angle"].get<float>();
	message.remote_speed = root["remote_speed"].get<float>();
	message.vehicle_id = root["vehicle_id"].get<uint32_t>();

	return message;

}

template <>
remotecontrolToggle MessageTypes::decode_json(const std::string &jsonstr) {
	json root = json::parse(jsonstr);

	remotecontrolToggle message;

	message.enable_remotecontrol = root["enable_remotecontrol"].get<bool>();
	message.vehicle_id = root["vehicle_id"].get<uint32_t>();

	return message;

}

template <>
platooningToggle MessageTypes::decode_json(const std::string &jsonstr) {
	json root = json::parse(jsonstr);

	platooningToggle message;

	message.enable_platooning = root["enable_platooning"].get<bool>();
	message.vehicle_id = root["vehicle_id"].get<uint32_t>();
	message.inner_platoon_distance = root["inner_platoon_distance"].get<float>();
	message.platoon_speed = root["platoon_speed"].get<float>();
	message.lvfv = root["lvfv"].get<std::string>();

	return message;

}

template <>
userInterface MessageTypes::decode_json(const std::string &jsonstr) {
	json root = json::parse(jsonstr);

	userInterface message;

	message.remotecontrol_enabled = root["remotecontrol_enabled"].get<bool>();
	message.leading_vehicle = root["leading_vehicle"].get<bool>();
	message.following_vehicle = root["following_vehicle"].get<bool>();
	message.potential_following_vehicle = root["potential_following_vehicle"].get<bool>();
	message.platooning_state = root["platooning_state"].get<std::string>();
	message.src_vehicle = root["src_vehicle"].get<uint32_t>();
	message.platoon_members = root["platoon_members"].get<std::vector<uint32_t>>();
	message.platoon_size = message.platoon_members.size();
	message.inner_platoon_distance = root["inner_platoon_distance"].get<float>();
	message.platoon_speed = root["platoon_speed"].get<float>();
	message.actual_distance = root["actual_distance"].get<float>();
	message.speed = root["speed"].get<float>();
	message.platoon_id = root["platoon_id"].get<uint32_t>();

	return message;

}

template <>
stmupdate MessageTypes::decode_json(const std::string &jsonstr) {
	json root = json::parse(jsonstr);
	stmupdate message;

	message.id = root["id"].get<uint32_t>();
	message.steeringAngle = root["steeringAngle"].get<float>();
	message.acceleration = root["acceleration"].get<float>();

	return message;

}

template <>
gazupdate MessageTypes::decode_json(const std::string &jsonstr) {
	json root = json::parse(jsonstr);

	gazupdate message;

	message.id = root["id"].get<uint32_t>();
	message.speed = root["speed"].get<float>();
	message.distance = root["distance"].get<float>();

	return message;

}

template <>
steeringAngle MessageTypes::decode_json(const std::string &jsonstr) {
	json root = json::parse(jsonstr);

	steeringAngle message;

	message.steering_angle = root["steering_angle"].get<float>();

	return message;

}

template <>
acceleration MessageTypes::decode_json(const std::string &jsonstr) {
	json root = json::parse(jsonstr);

	acceleration message;

	message.accelleration = root["accelleration"].get<float>();

	return message;

}

template <>
vehicleControl MessageTypes::decode_json(const std::string &jsonstr) {
	json root = json::parse(jsonstr);

	vehicleControl message;

	message.steering_angle = root["steering_angle"].get<float>();
	message.velocity = root["velocity"].get<float>();

	return message;
}

template<typename T>
std::string MessageTypes::encode_message(const T &message) {
	return "unknown type";
}

template <>
std::string MessageTypes::encode_message(const lv_broadcast &message) {
	json root;

	root["src_vehicle"] =  message.src_vehicle;
	root["followers"] = message.followers;
	root["ps"] = message.ps;
	root["ipd"] = message.ipd;
	root["platoon_id"] = message.platoon_id;

	return root.dump();
}

template <>
std::string MessageTypes::encode_message(const lv_accept &message) {
	json root;

	root["platoon_id"] = message.platoon_id;
	root["src_vehicle"] = message.src_vehicle;
	root["dst_vehicle"] = message.dst_vehicle;

	return root.dump();
}

template <>
std::string MessageTypes::encode_message(const lv_reject &message) {
	json root;

	root["platoon_id"] = message.platoon_id;
	root["src_vehicle"] = message.src_vehicle;
	root["dst_vehicle"] = message.dst_vehicle;

	return root.dump();
}

template <>
std::string MessageTypes::encode_message(const fv_heartbeat &message) {
	json root;

	root["platoon_id"] = message.platoon_id;
	root["src_vehicle"] = message.src_vehicle;

	return root.dump();
}

template <>
std::string MessageTypes::encode_message(const fv_leave &message) {
	json root;

	root["platoon_id"] = message.platoon_id;
	root["src_vehicle"] = message.src_vehicle;

	return root.dump();
}

template <>
std::string MessageTypes::encode_message(const fv_request &message) {
	json root;

	root["src_vehicle"] = message.src_vehicle;

	return root.dump();
}

template <>
std::string MessageTypes::encode_message(const remotecontrolInput &message) {
	json root;

	root["vehicle_id"] = message.vehicle_id;
	root["remote_angle"] = message.remote_angle;
	root["remote_speed"] = message.remote_speed;
	root["emergency_stop"] = (bool)message.emergency_stop;

	return root.dump();
}

template <>
std::string MessageTypes::encode_message(const remotecontrolToggle &message) {
	json root;

	root["enable_remotecontrol"] = (bool)message.enable_remotecontrol;
	root["vehicle_id"] = message.vehicle_id;

	std::cout << root.dump() << std::endl;

	return root.dump();
}

template <>
std::string MessageTypes::encode_message(const platooningToggle &message) {
	json root;

	root["vehicle_id"] = message.vehicle_id;
	root["enable_platooning"] = (bool)message.enable_platooning;
	root["inner_platoon_distance"] = message.inner_platoon_distance;
	root["platoon_speed"] = message.platoon_speed;
	root["lvfv"] = message.lvfv;

	return root.dump();
}

template <>
std::string MessageTypes::encode_message(const stmupdate &message) {
	json root;

	root["id"] = message.id;
	root["acceleration"] = message.acceleration;
	root["steeringAngle"] = message.steeringAngle;

	return root.dump();
}

template <>
std::string MessageTypes::encode_message(const gazupdate &message) {
	json root;

	root["id"] = message.id;
	root["distance"] = message.distance;
	root["speed"] = message.speed;

	return root.dump();
}

template <>
std::string MessageTypes::encode_message(const steeringAngle &message) {
	json root;

	root["steering_angle"] = message.steering_angle;

	return root.dump();
}

template <>
std::string MessageTypes::encode_message(const acceleration &message) {
	json root;

	root["accelleration"] = message.accelleration;

	return root.dump();
}

template <>
std::string MessageTypes::encode_message(const userInterface &message) {
	json root;

	root["leading_vehicle"] = (bool)message.leading_vehicle;
	root["following_vehicle"] = (bool)message.following_vehicle;
	root["potential_following_vehicle"] = (bool)message.potential_following_vehicle;

	root["inner_platoon_distance"] = message.inner_platoon_distance;
	root["actual_distance"] = message.actual_distance;
	root["platoon_speed"] = message.platoon_speed;
	root["speed"] = message.speed;

	root["platooning_state"] = message.platooning_state;

	root["src_vehicle"] = message.src_vehicle;
	root["platoon_size"] = message.platoon_size;
	root["platoon_members"] = message.platoon_members;
	root["remotecontrol_enabled"] = (bool)message.remotecontrol_enabled;

	root["platoon_id"] = message.platoon_id;

	return root.dump();
}

template <>
std::string MessageTypes::encode_message(const vehicleControl &message) {
	json root;

	root["velocity"] = message.velocity;
	root["steering_angle"] = message.steering_angle;

	return root.dump();

}

}