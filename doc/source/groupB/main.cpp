//
// Created by Paul Waljaew on 17.01.18.
//

#include <iostream>

#include <master/states.hpp>

using namespace platoon::master;

//******************************************************************
// PlatoonMain
// Statemachine with main PlatoonStates and transitions to submachine
// PlatoonFollower and submachine PlatoonLeader
//******************************************************************

PlatoonMain::PlatoonMain() { std::cout << "Entering PlatoonMain\n\n"; }

PlatoonMain::~PlatoonMain() {
  std::cout << "\n~Exiting PlatoonMain\n";
  std::cout << "---------------------------------------------------\n";
}

//******************************************************************
// Init
// The initial vehicle state for pre-configuration
//******************************************************************

Init::Init() { std::cout << "Entering Init\n"; };

Init::~Init() { std::cout << "~Exiting Init\n"; };

//******************************************************************
// EgoIdle
//******************************************************************

Idle::Idle() { std::cout << "Entering Idle\n"; }

Idle::~Idle() { std::cout << "Exiting Idle\n"; }

//******************************************************************
// PlatoonRequest
//******************************************************************

PlatoonRequest::PlatoonRequest() { std::cout << "Entering PlatoonRequest\n"; }

PlatoonRequest::~PlatoonRequest() { std::cout << "Exiting PlatoonRequest\n"; }

//******************************************************************
// PlatoonJoining
//******************************************************************

PlatoonJoining::PlatoonJoining() { std::cout << "Entering PlatoonJoining\n"; }

PlatoonJoining::~PlatoonJoining() { std::cout << "Exiting PlatoonJoining\n"; }

//******************************************************************
// PlatoonCreating
//******************************************************************

PlatoonCreating::PlatoonCreating() {
  std::cout << "Entering PlatoonCreating\n";
}

PlatoonCreating::~PlatoonCreating() {
  std::cout << "Exiting PlatoonCreating\n";
}

//******************************************************************
// Final
// The final vehicle state for shutdown
//******************************************************************

Final::Final() { std::cout << "Entering Final\n"; }

Final::~Final() { std::cout << "Exiting Final\n"; }
