//******************************************************************
// file:       platoon/master/states.hpp
// purpose:    Platoon state type
// license:    ?
// maintainer:    Paul Waljaew - waljaewp@informatik.hu-berlin.de
//                Bruno Lemke - lemkebru@informatik.hu-berlin.de
//
// created:    2017/04/02
//******************************************************************

#ifndef ROS1_PLATOON_MASTER_STATES_HPP
#define ROS1_PLATOON_MASTER_STATES_HPP

//******************************************************************
// STATES
//******************************************************************

#include <boost/mpl/list.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state_machine.hpp>

#include <master/events.hpp>

namespace platoon {
namespace master {

template <typename Self, typename... Other>
using State = boost::statechart::simple_state<Self, Other...>;

template <typename Self, typename Start>
using Stm = boost::statechart::state_machine<Self, Start>;

//******************************************************************
// State Forward Declarations
//******************************************************************

struct VehicleStatechart;

struct PlatoonLeaderMain;
struct PlatoonLeader;
struct PlatoonBroadcasting;
struct PlatoonReconfig;
struct PlatoonDisolve;
struct ObstacleAvoidance;
struct AdjustementLVSpeed;
struct EgoLVIdle;
struct EgoLVSpeedAdj;
struct EgoLVMoving;
struct EgoLVSteering;

struct PlatoonFollowerMain;
struct PlatoonFollower;
struct PlatoonLeaving;
struct UpdateData;
struct AdjustementFVSpeed;
struct EgoFVIdle;
struct EgoFVSpeedAdj;
struct EgoFVMoving;
struct EgoFVSteering;

struct PlatoonMain;
struct Init;
struct Idle;
struct PlatoonRequest;
struct PlatoonJoining;
struct PlatoonCreating;
struct Final;

//******************************************************************
// VehicleStatechart
//******************************************************************

struct VehicleStatechart : Stm<VehicleStatechart, PlatoonMain> {
 public:
  VehicleStatechart() = default;
};

//******************************************************************

//******************************************************************
// Leading Vehicle States
//******************************************************************

//******************************************************************
// PlatoonLeaderMain
//******************************************************************

struct PlatoonLeaderMain : State<PlatoonLeaderMain, VehicleStatechart,
                                 boost::mpl::list<PlatoonLeader, EgoLVIdle>> {
  PlatoonLeaderMain();

  ~PlatoonLeaderMain();
};

//******************************************************************
// PlatoonLeader
//******************************************************************

struct PlatoonLeader : State<PlatoonLeader, PlatoonLeaderMain::orthogonal<0>> {
  typedef boost::mpl::list<Transition<AdjustSpeedCmd, PlatoonBroadcasting>,
                           Transition<HeartbeatLoss, PlatoonReconfig>,
                           Transition<ChangeIpdCmd, PlatoonBroadcasting>,
                           Transition<ObstacleRecognized,
                                      ObstacleAvoidance>  // Action
                           // EmergencyBcCmd/ TODO
                           >
      reactions;

  PlatoonLeader();

  ~PlatoonLeader();
};

//******************************************************************
// PlatoonBroadcasting
//******************************************************************

struct PlatoonBroadcasting
    : State<PlatoonBroadcasting, PlatoonLeaderMain::orthogonal<0>> {
  typedef Transition<BroadcastSended, PlatoonLeader> reactions;

  PlatoonBroadcasting();

  ~PlatoonBroadcasting();
};

//******************************************************************
// PlatoonReconfig
//******************************************************************

struct PlatoonReconfig
    : State<PlatoonReconfig, PlatoonLeaderMain::orthogonal<0>> {
  typedef boost::mpl::list<Transition<PlatoonSizeOK, PlatoonLeader>,
                           Transition<PlatoonSizeToSmall, PlatoonDisolve>>
      reactions;

  PlatoonReconfig();

  ~PlatoonReconfig();
};

//******************************************************************
// PlatoonDisolve
//******************************************************************

struct PlatoonDisolve
    : State<PlatoonDisolve, PlatoonLeaderMain::orthogonal<0>> {
  typedef Transition<DisolvePlatoonCmd, Idle> reactions;

  PlatoonDisolve();

  ~PlatoonDisolve();
};

//******************************************************************
// ObstacleAvoidance
//******************************************************************

struct ObstacleAvoidance
    : State<ObstacleAvoidance, PlatoonLeaderMain::orthogonal<0>> {
  typedef Transition<EmergencyBreakCmd, Idle> reactions;

  ObstacleAvoidance();

  ~ObstacleAvoidance();
};

//******************************************************************
// AdjustmentLVSpeed
//******************************************************************

struct AdjustmentLVSpeed
    : State<AdjustmentLVSpeed, PlatoonLeaderMain::orthogonal<0>> {
  typedef Transition<AdjustDone, PlatoonLeader> reactions;

  AdjustmentLVSpeed();

  ~AdjustmentLVSpeed();
};

//******************************************************************
// EgoLVIdle:
// Power on the Vehicle
//******************************************************************

struct EgoLVIdle : State<EgoLVIdle, PlatoonLeaderMain::orthogonal<1>> {
  typedef Transition<AdjustSpeedCmd, EgoLVSpeedAdj> reactions;

  EgoLVIdle();

  ~EgoLVIdle();
};

//******************************************************************
// EgoLVSpeedAdj:
// Adjustment speed velocity ( +  | - )
//******************************************************************

struct EgoLVSpeedAdj : State<EgoLVSpeedAdj, PlatoonLeaderMain::orthogonal<1>> {
  typedef boost::mpl::list<Transition<VehicleIsMoving, EgoLVMoving>,
                           Transition<VelocityIsZero, EgoLVIdle>>
      reactions;

  EgoLVSpeedAdj();

  ~EgoLVSpeedAdj();
};

//******************************************************************
// EgoLVMoving:
// Vehicle's velocity greater ZERO
//******************************************************************

struct EgoLVMoving : State<EgoLVMoving, PlatoonLeaderMain::orthogonal<1>> {
  typedef boost::mpl::list<Transition<SteerCmd, EgoLVSteering>,
                           Transition<AdjustSpeedCmd, EgoLVSpeedAdj>>
      reactions;
  // Transition back to PlatoonStates??

  EgoLVMoving();

  ~EgoLVMoving();
};

//******************************************************************
// EgoSteering:
// Adjustment wheel angle
//******************************************************************

struct EgoLVSteering : State<EgoLVSteering, PlatoonLeaderMain::orthogonal<1>> {
  typedef Transition<AngleAdjDone, EgoLVMoving> reactions;

  EgoLVSteering();

  ~EgoLVSteering();
};

//******************************************************************
// Following Vehicle States
//******************************************************************

//******************************************************************
// PlatoonFollowerMain
//******************************************************************

struct PlatoonFollowerMain
    : State<PlatoonFollowerMain, VehicleStatechart,
            boost::mpl::list<PlatoonFollower, EgoFVIdle>> {
  PlatoonFollowerMain();

  ~PlatoonFollowerMain();
};

//******************************************************************
// PlatoonFollower
//******************************************************************

struct PlatoonFollower
    : State<PlatoonFollower, PlatoonFollowerMain::orthogonal<0>> {
  typedef boost::mpl::list<Transition<BroadcastReceived, UpdateData>,
                           Transition<LeavePlatoonCmd, PlatoonLeaving>,
                           Transition<After10, AdjustementFVSpeed>>
      reactions;

  PlatoonFollower();

  ~PlatoonFollower();
};

//******************************************************************
// PlatoonLeaving
//******************************************************************

struct PlatoonLeaving
    : State<PlatoonLeaving, PlatoonFollowerMain::orthogonal<0>> {
  typedef boost::mpl::list<Transition<LastPlatoonFollower,
                                      Idle>,  // Action Platoon disolve? TODO
                           Transition<NotLastPlatoonFollower, PlatoonFollower>>
      reactions;

  PlatoonLeaving();

  ~PlatoonLeaving();
};

//******************************************************************
// UpdateData
//******************************************************************

struct UpdateData : State<UpdateData, PlatoonFollowerMain::orthogonal<0>> {
  typedef Transition<UpdateDone, PlatoonFollower> reactions;

  UpdateData();

  ~UpdateData();
};

//******************************************************************
// AdjustementFVSpeed
//******************************************************************

struct AdjustementFVSpeed
    : State<AdjustementFVSpeed, PlatoonFollowerMain::orthogonal<0>> {
  typedef Transition<AdjustDone, PlatoonFollower> reactions;

  AdjustementFVSpeed();

  ~AdjustementFVSpeed();
};

//******************************************************************
// EgoFVIdle:
// Power on the Vehicle
//******************************************************************

struct EgoFVIdle : State<EgoFVIdle, PlatoonFollowerMain::orthogonal<1>> {
  typedef Transition<AdjustSpeedCmd, EgoFVSpeedAdj> reactions;

  EgoFVIdle();

  ~EgoFVIdle();
};

//******************************************************************
// EgoFVSpeedAdj:
// Adjustment speed velocity ( +  | - )
//******************************************************************

struct EgoFVSpeedAdj
    : State<EgoFVSpeedAdj, PlatoonFollowerMain::orthogonal<1>> {
  typedef boost::mpl::list<Transition<VehicleIsMoving, EgoFVMoving>,
                           Transition<VelocityIsZero, EgoFVIdle>>
      reactions;

  EgoFVSpeedAdj();

  ~EgoFVSpeedAdj();
};

//******************************************************************
// EgoFVMoving:
// Vehicle's velocity greater ZERO
//******************************************************************

struct EgoFVMoving : State<EgoFVMoving, PlatoonFollowerMain::orthogonal<1>> {
  typedef boost::mpl::list<Transition<SteerCmd, EgoFVSteering>,
                           Transition<AdjustSpeedCmd, EgoFVSpeedAdj>>
      reactions;
  // Transition back to PlatoonStates??

  EgoFVMoving();

  ~EgoFVMoving();
};

//******************************************************************
// EgoFVSteering:
// Adjustment wheel angle
//******************************************************************

struct EgoFVSteering
    : State<EgoFVSteering, PlatoonFollowerMain::orthogonal<1>> {
  typedef Transition<AngleAdjDone, EgoFVMoving> reactions;

  EgoFVSteering();

  ~EgoFVSteering();
};

//******************************************************************
// Main states
//******************************************************************

//******************************************************************
// PlatoonMain
// Point of origin for the two other sub-state-machines
// PlatoonFollowerMain | PlatoonLeaderMain
//******************************************************************

struct PlatoonMain : State<PlatoonMain, VehicleStatechart, Init> {
  PlatoonMain();

  ~PlatoonMain();
};

template <typename Self>
using MainState = State<Self, PlatoonMain>;

//******************************************************************
// Init
// The initial vehicle state for pre-configuration
//******************************************************************

struct Init : MainState<Init> {
  typedef Transition<PreConfigDone, Idle> reactions;

  Init();

  ~Init();
};

//******************************************************************
// Idle
//******************************************************************

struct Idle : MainState<Idle> {
  typedef boost::mpl::list<Transition<RequestPlatoonCmd, PlatoonRequest>,
                           Transition<ShutdownSystemCmd, Final>>
      reactions;

  Idle();

  ~Idle();
};

//******************************************************************
// PlatoonRequest
//******************************************************************

struct PlatoonRequest : MainState<PlatoonRequest> {
  typedef boost::mpl::list<Transition<RequestUnsuccesfull, Idle>,
                           Transition<IsFollower, PlatoonJoining>,
                           Transition<IsLeader, PlatoonCreating>>
      reactions;

  PlatoonRequest();

  ~PlatoonRequest();
};

//******************************************************************
// PlatoonJoining
//******************************************************************

struct PlatoonJoining : MainState<PlatoonJoining> {
  typedef Transition<JoiningDone, PlatoonFollowerMain> reactions;

  PlatoonJoining();

  ~PlatoonJoining();
};

//******************************************************************
// PlatoonCreating
//******************************************************************

struct PlatoonCreating : MainState<PlatoonCreating> {
  typedef Transition<CreatingDone, PlatoonLeaderMain> reactions;

  PlatoonCreating();

  ~PlatoonCreating();
};

//******************************************************************
// PlatoonFinal
// The final vehicle state for shutdown
//******************************************************************

struct Final : MainState<Final> {
  Final();

  ~Final();
};

}  // namespace master
}  // namespace platoon

#endif  // ROS1_PLATOON_MASTER_STATES_HPP
