cmake_minimum_required(VERSION 2.8.3)
project(platooning)

enable_language(CXX)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wall -g)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        ## GENERAL
        roscpp
        rospy
        nodelet
        ## MESSAGE GEN
        message_generation
        )

find_package(rostest REQUIRED)

find_package(Boost REQUIRED COMPONENTS
        ## DUNNO LOL
        thread
        system
        )

######################################################

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        ########################
        ## lanekeeping messages
        ########################
        ## environment mapping messages
        ########################
        ## protocol messages
        platooning/fv_heartbeat.msg
        platooning/fv_leave.msg
        platooning/fv_request.msg
        platooning/lv_accept.msg
        platooning/lv_reject.msg
        platooning/lv_broadcast.msg
        platooning/platooningState.msg
        platooning/vehicleId.msg
        ########################
        ## userinterface messages
        userInterface.msg
        ########################
        ## lateralControl messages
        vehiclecontrol/steeringAngle.msg
        ########################
        ## longitudalControl messages
        ########################
        ## radiointerface messages
        platoonProtocol.msg
        ########################
        ## remote control messages
        remotecontrol/platooningToggle.msg
        remotecontrol/remotecontrolInput.msg
        remotecontrol/remotecontrolToggle.msg
        ########################
        ## stm messages
        stm/distance.msg
        stm/speed.msg
        stm/camera.msg
        ########################
        ## templateMessages
        sim/stmupdate.msg
        sim/gazupdate.msg
        ########################
        ## prioritization msgs
        vehiclecontrol/targetSpeed.msg
        vehiclecontrol/targetDistance.msg
        vehiclecontrol/targetAngle.msg
        ########################
        ## vehicle control
        vehiclecontrol/acceleration.msg
        vehiclecontrol/vehicleControl.msg
        ########################
        ## templateMessages
        templateMsg.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   getVehicleId.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES prioritization messagedistribution userinterface
                vehiclecontrol radiointerface platooning udpserver messagetypes topics longitudinalprocessing
                services servicetypes stmsim
        CATKIN_DEPENDS roscpp std_msgs message_runtime nodelet rospy
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include testing/include
        ${catkin_INCLUDE_DIRS}
)
#######################
#### platooning modules
#######################

## Declare a C++ library
add_library(lanedetect SHARED include/platooning/LaneDetect.hpp src/LaneDetect.cpp)
add_dependencies(lanedetect ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(lanedetect ${catkin_LIBRARIES})

add_library(prioritization SHARED include/platooning/Prioritization.hpp src/Prioritization.cpp)
add_dependencies(prioritization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(prioritization ${catkin_LIBRARIES})

add_library(messagedistribution SHARED include/platooning/MessageTypes.hpp include/platooning/MessageDistribution.hpp src/MessageDistribution.cpp)
add_dependencies(messagedistribution messagetypes ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(messagedistribution messagetypes ${catkin_LIBRARIES})

add_library(userinterface SHARED include/platooning/UserInterface.hpp src/UserInterface.cpp)
add_dependencies(userinterface services servicetypes messagetypes topics ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(userinterface services servicetypes messagetypes topics ${catkin_LIBRARIES})

add_library(vehiclecontrol SHARED include/platooning/VehicleControl.hpp src/VehicleControl.cpp)
add_dependencies(vehiclecontrol ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(vehiclecontrol ${catkin_LIBRARIES})

add_library(radiointerface SHARED include/platooning/MessageTypes.hpp include/platooning/UdpServer.hpp include/platooning/RadioInterface.hpp src/RadioInterface.cpp)
add_dependencies(radiointerface udpserver ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(radiointerface udpserver ${catkin_LIBRARIES})

add_library(udpserver SHARED include/platooning/MessageTypes.hpp include/platooning/UdpServer.hpp src/UdpServer.cpp src/UdpPackageSet.cpp include/platooning/UdpPackageSet.hpp)
add_dependencies(udpserver ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(udpserver ${catkin_LIBRARIES})

add_library(topics SHARED include/platooning/Topics.hpp)
set_target_properties(topics PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(topics ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(topics ${catkin_LIBRARIES})

add_library(messagetypes SHARED include/platooning/MessageTypes.hpp src/MessageTypes.cpp include/platooning/json.hpp)
add_dependencies(messagetypes ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(messagetypes ${catkin_LIBRARIES})

add_library(services SHARED include/platooning/Services.hpp)
set_target_properties(services PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(services ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(services ${catkin_LIBRARIES})

add_library(servicetypes SHARED include/platooning/ServiceTypes.hpp)
set_target_properties(servicetypes PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(servicetypes ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(servicetypes ${catkin_LIBRARIES})

add_library(template SHARED include/platooning/Template.hpp src/Template.cpp)
add_dependencies(template ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(template ${catkin_LIBRARIES})

add_library(platooning SHARED include/platooning/Platooning.hpp src/Platooning.cpp)
add_dependencies(platooning ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(platooning ${catkin_LIBRARIES})

add_library(stmsim SHARED include/platooning/StmSim.hpp src/StmSim.cpp)
add_dependencies(stmsim udpserver topics messagetypes ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(stmsim udpserver messagetypes topics ${catkin_LIBRARIES})

add_library(longitudinalprocessing SHARED include/platooning/LongitudinalProcessing.hpp src/LongitudinalProcessing.cpp)
add_dependencies(longitudinalprocessing topics messagetypes ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(longitudinalprocessing topics messagetypes ${catkin_LIBRARIES})

##############################################
#### platooning module tests
##############################################
add_library(moduletest SHARED testing/include/platooning/Moduletest.hpp testing/src/Moduletest.cpp)
add_dependencies(moduletest ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(moduletest ${catkin_LIBRARIES})

add_library(moduletest_messagedistribution SHARED testing/include/platooning/Moduletest_messagedistribution.hpp testing/src/Moduletest_messagedistribution.cpp)
add_dependencies(moduletest_messagedistribution topics messagetypes moduletest messagedistribution ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(moduletest_messagedistribution topics messagetypes moduletest ${catkin_LIBRARIES})

add_library(moduletest_radiointerface SHARED testing/include/platooning/Moduletest_radiointerface.hpp testing/src/Moduletest_radiointerface.cpp)
add_dependencies(moduletest_radiointerface topics messagetypes moduletest udpserver radiointerface ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(moduletest_radiointerface topics messagetypes moduletest udpserver ${catkin_LIBRARIES})

add_library(moduletest_platooning SHARED testing/include/platooning/Moduletest_platooning.hpp testing/src/Moduletest_platooning.cpp)
add_dependencies(moduletest_platooning messagetypes topics moduletest ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(moduletest_platooning messagetypes topics moduletest ${catkin_LIBRARIES})

add_library(moduletest_prioritization SHARED testing/include/platooning/Moduletest_prioritization.hpp testing/src/Moduletest_prioritization.cpp)
add_dependencies(moduletest_prioritization messagetypes topics services servicetypes moduletest ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp )
target_link_libraries(moduletest_prioritization messagetypes topics services servicetypes moduletest ${catkin_LIBRARIES})

add_library(moduletest_longitudinalprocessing SHARED testing/include/platooning/Moduletest_longitudinalprocessing.hpp testing/src/Moduletest_longitudinalprocessing.cpp)
add_dependencies(moduletest_longitudinalprocessing messagetypes topics moduletest ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp )
target_link_libraries(moduletest_longitudinalprocessing messagetypes topics moduletest ${catkin_LIBRARIES})

add_library(moduletest_messagetypes SHARED testing/include/platooning/Moduletest_messagetypes.hpp testing/src/Moduletest_messagetypes.cpp)
add_dependencies(moduletest_messagetypes messagetypes topics moduletest ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp )
target_link_libraries(moduletest_messagetypes messagetypes topics moduletest ${catkin_LIBRARIES})

add_library(moduletest_udpserver SHARED testing/include/platooning/Moduletest_udpserver.hpp testing/src/Moduletest_udpserver.cpp)
add_dependencies(moduletest_udpserver udpserver messagetypes moduletest ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp )
target_link_libraries(moduletest_udpserver udpserver messagetypes moduletest ${catkin_LIBRARIES})

add_library(moduletest_template SHARED testing/include/platooning/Moduletest_template.hpp testing/src/Moduletest_template.cpp)
add_dependencies(moduletest_template moduletest ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(moduletest_template moduletest ${catkin_LIBRARIES})

#add_library(pcap test/Pcap.hpp test/Pcap.cpp)
#target_link_libraries(pcap libpcap.so)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(TARGETS
            prioritization messagedistribution
            userinterface vehiclecontrol radiointerface platooning udpserver messagetypes
            topics longitudinalprocessing services servicetypes stmsim
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark cpp header files for installation
install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        #   FILES_MATCHING PATTERN "*.h"
        #   PATTERN ".svn" EXCLUDE
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY plugins/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins
        PATTERN ".svn" EXCLUDE)

#install(DIRECTORY testing/launch/
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/testing/launch
#        PATTERN ".svn" EXCLUDE)

#install(DIRECTORY testing/plugins/
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/testing/plugins
#        PATTERN ".svn" EXCLUDE)


## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES launch/environment_mapping.launch launch/lanedetect.launch launch/prioritization.launch  messagedistribution.launch  userinterface.launch  vehiclecontrol.launch  radiointerface.launch  template.launch
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#        )
#install(FILES environment_mapping.xml lanedetect.xml  prioritization.xml  messagedistribution.xml  userinterface.xml  vehiclecontrol.xml  radiointerface.xml  template.xml
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

