#include <platooning_sim/gazebo_camera_follow.hpp>

namespace gazebo {

CameraFollowPlugin::CameraFollowPlugin() {

}
void CameraFollowPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
	// Store the pointer to the model
	this->model_ = _parent;

	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		std::bind(&CameraFollowPlugin::OnUpdate, this));

}

void CameraFollowPlugin::OnUpdate() {
	//gazebo::gui has qtcore include problem even though it exists. fuck cmake or me
	//rendering::UserCameraPtr cam = gui::get_active_camera();

	//ignition::math::Pose3d model_pose = model_->WorldPose();

	//set usercamera to offset of model
	//model_pose.Pos().X( model_pose.Pos().X() - 10 );

	//cam->SetWorldPose( model_pose );

}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(CameraFollowPlugin)

}