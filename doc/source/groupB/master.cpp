//
// Created by Paul Waljaew on 17.01.18.
//

#include <pluginlib/class_list_macros.h>

#include <master/master.hpp>

using namespace platoon::master;





void VehicleMaster::onInit() {
    NODELET_DEBUG("VehicleMaster::onInit");

    // Test Transitions
    VehicleStatechart vm;
    vm.initiate();

    //PlatoonMain
    vm.process_event(PreConfigDone());
    vm.process_event(RequestPlatoonCmd());
    vm.process_event(IsFollower());
    vm.process_event(JoiningDone());

    //PlatoonFollowerMain
    vm.process_event(BroadcastReceived());
    vm.process_event(UpdateDone());

    vm.process_event(AdjustSpeedCmd());
    vm.process_event(AdjustSpeedCmd());

    //EgoVehicle
    vm.process_event(AdjustSpeedCmd());
    vm.process_event(VehicleIsMoving());
    vm.process_event(SteerCmd());
    vm.process_event(AngleAdjDone());
    vm.process_event(AdjustSpeedCmd());
    vm.process_event(VelocityIsZero());


};

//------ ~VehicleMaster ------
VehicleMaster::~VehicleMaster() {
    NODELET_DEBUG("VehicleMaster::~VehicleMaster");


}


PLUGINLIB_DECLARE_CLASS(platoon, master, platoon::master::VehicleMaster,
        nodelet::Nodelet);
