//
// Created by Paul Waljaew on 18.01.18.
//

#include <iostream>

#include <master/states.hpp>

using namespace platoon::master;

//******************************************************************
// PlatoonLeaderMain
//******************************************************************

PlatoonLeaderMain::PlatoonLeaderMain() {
  std::cout << "Entering PlatoonLeaderMain\n\n";
}

PlatoonLeaderMain::~PlatoonLeaderMain() {
  std::cout << "\nExiting PlatoonLeaderMain\n";
  std::cout << "---------------------------------------------------\n";
}

//******************************************************************
// PlatoonLeader
//******************************************************************

PlatoonLeader::PlatoonLeader() { std::cout << "Entering PlatoonLeader\n"; }

PlatoonLeader::~PlatoonLeader() { std::cout << "Exiting PlatoonLeader\n"; }

//******************************************************************
// PlatoonDisolve
//******************************************************************

PlatoonDisolve::PlatoonDisolve() { std::cout << "Entering PlatoonDisolve\n"; }

PlatoonDisolve::~PlatoonDisolve() { std::cout << "Exiting PlatoonDisolve\n"; }

//******************************************************************
// PlatoonReconfig
//******************************************************************

PlatoonReconfig::PlatoonReconfig() {
  std::cout << "Entering PlatoonReconfig\n";
}

PlatoonReconfig::~PlatoonReconfig() {
  std::cout << "Exiting PlatoonReconfig\n";
}

//******************************************************************
// PlatoonBroadcasting
//******************************************************************

PlatoonBroadcasting::PlatoonBroadcasting() {
  std::cout << "Entering PlatoonBroadcasting\n";
}

PlatoonBroadcasting::~PlatoonBroadcasting() {
  std::cout << "Exiting PlatoonBroadcasting\n";
}

//******************************************************************
// ObstacleAvoidance
//******************************************************************

ObstacleAvoidance::ObstacleAvoidance() {
  std::cout << "Entering ObstacleAvoidance\n";
}

ObstacleAvoidance::~ObstacleAvoidance() {
  std::cout << "Exiting ObstacleAvoidance\n";
}

//******************************************************************
// AdjustmentLVSpeed
//******************************************************************

AdjustmentLVSpeed::AdjustmentLVSpeed() {
  std::cout << "Entering AdjustmentLVSpeed\n";
}

AdjustmentLVSpeed::~AdjustmentLVSpeed() {
  std::cout << "Exiting AdjustmentLVSpeed\n";
}

//******************************************************************
// EgoLVMoving
//******************************************************************

EgoLVIdle::EgoLVIdle() { std::cout << "Entering EgoLVIdle\n"; }

EgoLVIdle::~EgoLVIdle() { std::cout << "Exiting EgoLVIdle\n"; }

//******************************************************************
// EgoLVSpeedAdj
//******************************************************************

EgoLVSpeedAdj::EgoLVSpeedAdj() { std::cout << "Entering EgoLVSpeedAdj\n"; }

EgoLVSpeedAdj::~EgoLVSpeedAdj() { std::cout << "Exiting EgoLVSpeedAdj\n"; }

//******************************************************************
// EgoLVMoving
//******************************************************************

EgoLVMoving::EgoLVMoving() { std::cout << "Entering EgoLVMoving\n"; }

EgoLVMoving::~EgoLVMoving() { std::cout << "Exiting EgoLVMoving\n"; }

//******************************************************************
// EgoLVSteering
//******************************************************************

EgoLVSteering::EgoLVSteering() { std::cout << "Entering EgoLVSteering\n"; }

EgoLVSteering::~EgoLVSteering() { std::cout << "Exiting EgoLVSteering\n"; }
