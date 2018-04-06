#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
//#include <gazebo/gui/gui.hh>
#include <ignition/math/Pose3.hh>
#include <ros/ros.h>

#ifndef PLATOONING_SIM_GAZEBO_CAMERA_FOLLOW_HPP
#define PLATOONING_SIM_GAZEBO_CAMERA_FOLLOW_HPP

namespace gazebo
{
class CameraFollowPlugin : public ModelPlugin
{
  public:
	CameraFollowPlugin();

	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

	// Called by the world update start event
    public: void OnUpdate();

	// Pointer to the model
  private: physics::ModelPtr model_;

	// Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;

};
}

#endif //PLATOONING_SIM_GAZEBO_CAMERA_FOLLOW_HPP
