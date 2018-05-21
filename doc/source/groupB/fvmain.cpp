//
// Created by Paul Waljaew on 18.01.18.
//

#include <iostream>

#include <master/states.hpp>

using namespace platoon::master;

//******************************************************************
// PlatoonLeaderMain
//******************************************************************

PlatoonFollowerMain::PlatoonFollowerMain() {
  std::cout << "Entering PlatoonFollowerMain\n\n";
}

PlatoonFollowerMain::~PlatoonFollowerMain() {
  std::cout << "\nExiting PlatoonFollowerMain\n";
  std::cout << "---------------------------------------------------\n";
}

//******************************************************************
// PlatoonFollower
//******************************************************************

PlatoonFollower::PlatoonFollower() {
  std::cout << "Entering PlatoonFollower\n";
}

PlatoonFollower::~PlatoonFollower() {
  std::cout << "Exiting PlatoonFollower\n";
}

//******************************************************************
// PlatoonLeaving
//******************************************************************

PlatoonLeaving::PlatoonLeaving() { std::cout << "Entering PlatoonLeaving\n"; }

PlatoonLeaving::~PlatoonLeaving() { std::cout << "Exiting PlatoonLeaving\n"; }

//******************************************************************
// UpdateData
//******************************************************************

UpdateData::UpdateData() { std::cout << "Entering UpdateData\n"; }

UpdateData::~UpdateData() { std::cout << "Exiting UpdateData\n"; }

//******************************************************************
// AdjustementFVSpeed
//******************************************************************

AdjustementFVSpeed::AdjustementFVSpeed() {
  std::cout << "Entering AdjustementFVSpeed\n";
}

AdjustementFVSpeed::~AdjustementFVSpeed() {
  std::cout << "Exiting AdjustementFVSpeed\n";
}

//******************************************************************
// EgoFVIdle
//******************************************************************

EgoFVIdle::EgoFVIdle() { std::cout << "Entering EgoFVIdle\n"; }

EgoFVIdle::~EgoFVIdle() { std::cout << "Exiting EgoFVIdle\n"; }

//******************************************************************
// EgoFVSpeedAdj
//******************************************************************

EgoFVSpeedAdj::EgoFVSpeedAdj() { std::cout << "Entering EgoFVSpeedAdj\n"; }

EgoFVSpeedAdj::~EgoFVSpeedAdj() { std::cout << "Exiting EgoFVSpeedAdj\n"; }

//******************************************************************
// EgoFVMoving
//******************************************************************

EgoFVMoving::EgoFVMoving() { std::cout << "Entering EgoFVMoving\n"; }

EgoFVMoving::~EgoFVMoving() { std::cout << "Exiting EgoFVMoving\n"; }

//******************************************************************
// EgoFVSteering
//******************************************************************

EgoFVSteering::EgoFVSteering() { std::cout << "Entering EgoFVSteering\n"; }

EgoFVSteering::~EgoFVSteering() { std::cout << "Exiting EgoFVSteering\n"; }
