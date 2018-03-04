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
        roslint
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
        steeringAngle.msg
        ########################
        ## environment mapping messages
        distanceToObj.msg
        ########################
        ## prioritization messages
        oughtData.msg
        ########################
        ## protocol messages
        platooning/fv_heartbeat.msg
        platooning/fv_leave.msg
        platooning/fv_request.msg
        platooning/lv_accept.msg
        platooning/lv_reject.msg
        platooning/lv_broadcast.msg
        platooning/platooningState.msg
        ########################
        ## userinterface messages
        userInterface.msg
        ########################
        ## vehiclecontrol messages
        acceleration.msg
        steeringAngle.msg
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
        ## templateMessages
        templateMsg.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

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
        LIBRARIES environment_mapping lanedetect lanekeeping prioritization messagedistribution userinterface vehiclecontrol radiointerface template platooning udpserver messagetypes topics
        CATKIN_DEPENDS roscpp std_msgs message_runtime nodelet rospy
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include testing/include sim/include
        ${catkin_INCLUDE_DIRS}
)
#######################
#### platooning modules
#######################

## Declare a C++ library
add_library(environment_mapping include/EnvironmentMapping.hpp src/EnvironmentMapping.cpp src/UdpServer.cpp include/UdpServer.hpp)
add_dependencies(environment_mapping ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(environment_mapping ${catkin_LIBRARIES})

add_library(lanedetect include/LaneDetect.hpp src/LaneDetect.cpp)
add_dependencies(lanedetect ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(lanedetect ${catkin_LIBRARIES})

add_library(lanekeeping include/LaneKeeping.hpp src/LaneKeeping.cpp)
add_dependencies(lanekeeping ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(lanekeeping ${catkin_LIBRARIES})

add_library(prioritization include/Prioritization.hpp src/Prioritization.cpp)
add_dependencies(prioritization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(prioritization ${catkin_LIBRARIES})

add_library(messagedistribution include/MessageTypes.hpp include/MessageDistribution.hpp src/MessageDistribution.cpp)
add_dependencies(messagedistribution ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(messagedistribution ${catkin_LIBRARIES})

add_library(userinterface include/UserInterface.hpp src/UserInterface.cpp)
add_dependencies(userinterface ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(userinterface ${catkin_LIBRARIES})

add_library(vehiclecontrol include/VehicleControl.hpp src/VehicleControl.cpp)
add_dependencies(vehiclecontrol ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(vehiclecontrol ${catkin_LIBRARIES})

add_library(radiointerface include/MessageTypes.hpp include/UdpServer.hpp include/RadioInterface.hpp src/RadioInterface.cpp)
add_dependencies(radiointerface udpserver ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(radiointerface udpserver ${catkin_LIBRARIES})

add_library(udpserver include/MessageTypes.hpp include/UdpServer.hpp src/UdpServer.cpp)
add_dependencies(udpserver ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(udpserver ${catkin_LIBRARIES})

add_library(topics include/Topics.hpp)
set_target_properties(topics PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(topics ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(topics ${catkin_LIBRARIES})

add_library(messagetypes include/MessageTypes.hpp src/MessageTypes.cpp)
add_dependencies(messagetypes ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(messagetypes ${catkin_LIBRARIES})

add_library(template include/Template.hpp src/Template.cpp)
add_dependencies(template ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(template ${catkin_LIBRARIES})

add_library(platooning include/Platooning.hpp src/Platooning.cpp)
add_dependencies(platooning ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(platooning ${catkin_LIBRARIES})

##############################################
#### platooning tests
##############################################
add_library(moduletest testing/include/Moduletest.hpp testing/src/Moduletest.cpp)
add_dependencies(moduletest ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(moduletest ${catkin_LIBRARIES})

add_library(moduletest_messagedistribution testing/include/Moduletest_messagedistribution.hpp testing/src/Moduletest_messagedistribution.cpp)
add_dependencies(moduletest_messagedistribution topics messagetypes moduletest messagedistribution ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(moduletest_messagedistribution topics messagetypes moduletest ${catkin_LIBRARIES})

add_library(moduletest_radiointerface testing/include/Moduletest_radiointerface.hpp testing/src/Moduletest_radiointerface.cpp)
add_dependencies(moduletest_radiointerface topics moduletest udpserver radiointerface ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(moduletest_radiointerface topics moduletest udpserver ${catkin_LIBRARIES})

add_library(moduletest_platooning testing/include/Moduletest_platooning.hpp testing/src/Moduletest_platooning.cpp)
add_dependencies(moduletest_platooning messagetypes moduletest ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(moduletest_platooning messagetypes moduletest ${catkin_LIBRARIES})

add_library(moduletest_template testing/include/Moduletest_template.hpp testing/src/Moduletest_template.cpp)
add_dependencies(moduletest_template messagetypes moduletest ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(moduletest_template messagetypes moduletest ${catkin_LIBRARIES})

#add_library(pcap test/Pcap.hpp test/Pcap.cpp)
#target_link_libraries(pcap libpcap.so)

##################
## UNIT Testing ##
##################
# if your test needs a roscore, use
# add_rostest_gtest(YOURNODELET-unit-test launch/YOURNODELET.launch test/unittest_YOURNODELET.cpp)
# target_link_libraries(YOURNODELET-unit-test YOURNODELET_REQUIRED_LIBRARIES ${catkin_LIBRARIES})
##################
## Add gtest based cpp test target and link libraries

catkin_add_gtest(platooning-unit-test testing/src/unittest_platooning.cpp)
target_link_libraries(platooning-unit-test ${catkin_LIBRARIES})

catkin_add_gtest(lanedetect-unit-test testing/src/unittest_lanedetect.cpp)
target_link_libraries(lanedetect-unit-test ${catkin_LIBRARIES})

catkin_add_gtest(priorization-unit-test testing/src/unittest_priorization.cpp)
target_link_libraries(priorization-unit-test ${catkin_LIBRARIES})

catkin_add_gtest(userinterface-unit-test testing/src/unittest_userinterface.cpp)
target_link_libraries(userinterface-unit-test ${catkin_LIBRARIES})

catkin_add_gtest(vehiclecontrol-unit-test testing/src/unittest_vehiclecontrol.cpp)
target_link_libraries(vehiclecontrol-unit-test vehiclecontrol ${catkin_LIBRARIES})

catkin_add_gtest(radiointerface-unit-test launch/radiointerface.launch testing/src/unittest_radiointerface.cpp)
target_link_libraries(radiointerface-unit-test ${catkin_LIBRARIES})

catkin_add_gtest(udpserver-unit-test testing/src/unittest_udpserver.cpp)
target_link_libraries(udpserver-unit-test udpserver ${catkin_LIBRARIES})

catkin_add_gtest(template-unit-test testing/src/unittest_template.cpp)
target_link_libraries(template-unit-test ${catkin_LIBRARIES})

add_rostest_gtest(messagedistribution-unit-test launch/messagedistribution.launch testing/src/unittest_messagedistribution.cpp)
target_link_libraries(messagedistribution-unit-test messagedistribution ${catkin_LIBRARIES})

#add_rostest_gtest(platooningnode-unit-test testing/src/unittest_platooningnode.cpp)
#target_link_libraries(platooningnode-unit-test platooningnode ${catkin_LIBRARIES})

#if (TARGET ${PROJECT_NAME}-test)
#    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
#    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
#endif ()


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
install(TARGETS environment_mapping lanedetect lanekeeping prioritization messagedistribution userinterface vehiclecontrol radiointerface template platooning udpserver messagetypes topics
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

install(DIRECTORY testing/launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY testing/plugins/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins
        PATTERN ".svn" EXCLUDE)


## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES launch/environment_mapping.launch launch/lanedetect.launch launch/priorization.launch  messagedistribution.launch  userinterface.launch  vehiclecontrol.launch  radiointerface.launch  template.launch
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#        )
#install(FILES environment_mapping.xml lanedetect.xml  priorization.xml  messagedistribution.xml  userinterface.xml  vehiclecontrol.xml  radiointerface.xml  template.xml
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

