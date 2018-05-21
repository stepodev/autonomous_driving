//
// Created by Paul Waljaew on 17.01.18.
//

#ifndef ROS1_PLATOON_MASTER_EVENTS_HPP
#define ROS1_PLATOON_MASTER_EVENTS_HPP

//******************************************************************
// EVENTS
//******************************************************************

#include <boost/statechart/event.hpp>
#include <boost/statechart/transition.hpp>

namespace platoon {
namespace master {

template <typename Self>
using Event = boost::statechart::event<Self>;

template <typename From, typename To>
using Transition = boost::statechart::transition<From, To>;

//******************************************************************
// Commands/Events EgoVehicle
//******************************************************************
// EgoIdle | EgoSpeedAdj
struct AdjustSpeedCmd : Event<AdjustSpeedCmd> {};

// EgoSteering
struct AngleAdjDone : Event<AngleAdjDone> {};

// EgoIdle
struct SteerCmd : Event<SteerCmd> {};

// EgoSpeedAdj
struct VehicleIsMoving : Event<VehicleIsMoving> {};  // Velocity > 0
struct VelocityIsZero : Event<VelocityIsZero> {};    // Velocity == 0

//******************************************************************
// Commands/Events PlatoonMain
//******************************************************************

// Init
struct PreConfigDone : Event<PreConfigDone> {};
struct ShutdownSystemCmd : Event<ShutdownSystemCmd> {};

// PlatoonRequest
struct RequestPlatoonCmd : Event<RequestPlatoonCmd> {};
struct RequestUnsuccesfull : Event<RequestUnsuccesfull> {};
struct IsFollower : Event<IsFollower> {};
struct IsLeader : Event<IsLeader> {};

// PlatoonJoining/PlatoonCreating
struct JoiningDone : Event<JoiningDone> {};    // implies Request succeeded
struct CreatingDone : Event<CreatingDone> {};  // implies Request succeeded

//******************************************************************
// Commands/Events PlatoonLeaderMain
//******************************************************************

// PlatoonLeader
struct HeartbeatLoss : Event<HeartbeatLoss> {};
struct ChangeIpdCmd : Event<ChangeIpdCmd> {};
struct ObstacleRecognized : Event<ObstacleRecognized> {
};  // Response Action -> EmergencyBcCmd TODO
struct EmergencyBcCmd : Event<EmergencyBcCmd> {};  // ACTION TODO
struct After10 : Event<After10> {
};  // Trigger Event each time 10ms expired //TODO

// PlatoonBroadcasting
struct BroadcastSended : Event<BroadcastSended> {};

// ObstacleAviodance
struct EmergencyBreakCmd : Event<EmergencyBreakCmd> {};

// PlatoonReconfig
struct PlatoonSizeOK : Event<PlatoonSizeOK> {};            // PlatoonSize > 1
struct PlatoonSizeToSmall : Event<PlatoonSizeToSmall> {};  // PlatoonSize == 1

// PlatoonDisolve
struct DisolvePlatoonCmd : Event<DisolvePlatoonCmd> {};

//******************************************************************
// Commands/Events PlatoonFollowerMain
//******************************************************************

// PlatoonFollower
struct BroadcastReceived : Event<BroadcastReceived> {};
struct LeavePlatoonCmd : Event<LeavePlatoonCmd> {};

// PlatoonLeave
struct LastPlatoonFollower : Event<LastPlatoonFollower> {};
struct NotLastPlatoonFollower : Event<NotLastPlatoonFollower> {};

// UpdateData
struct UpdateDone : Event<UpdateDone> {};

//******************************************************************
// Commands/Events PlatoonFollowerMain AND PlatoonLeaderMain
//******************************************************************

// AdjustmentLVSpeed | AdjustmentFVSpeed
struct AdjustDone : Event<AdjustDone> {};

}  // namespace master
}  // namespace platoon

#endif  // ROS1_PLATOON_EVENTS_HPP