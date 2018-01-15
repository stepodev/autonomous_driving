cmake_minimum_required(VERSION 2.8.3)
project(platooning)

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

find_package(Boost REQUIRED COMPONENTS
        ## DUNNO LOL
        thread
        system
        )


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
        steeringAngle.msg
        distanceToObj.msg
        userInterface.msg
        prioritisationDrivingVector.msg
        ########################
        ## remoteControlMessages
        remoteDrivingVector.msg
        enableRemoteControl.msg
        forcedDrivingVector.msg
        ########################
        ## templateMessages
        templateMsg.msg
        ########################
        ## lanekeeping messages
        ########################
        ## priorization messages
        ########################
        ## protocol messages
        platooningAction.msg
        platoonProtocolIn.msg
        platoonProtocolOut.msg
        ########################
        ## userinterface messages
        ########################
        ## vehiclecontrol messages
        ########################
        ## wifi messages

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
        LIBRARIES environment_mapping remotecontrol lanekeeping priorization protocol userinterface vehiclecontrol wifi template
        CATKIN_DEPENDS roscpp std_msgs message_runtime nodelet rospy
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include/platooning
        include/tests
        ${catkin_INCLUDE_DIRS}
)
#######################
#### platooning modules
#######################

## Declare a C++ library
add_library(environment_mapping include/platooning/EnvironmentMapping.hpp src/EnvironmentMapping.cpp)
add_dependencies(environment_mapping ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(environment_mapping ${catkin_LIBRARIES})

add_library(remotecontrol include/platooning/RemoteControl.hpp src/RemoteControl.cpp)
add_dependencies(remotecontrol ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(remotecontrol ${catkin_LIBRARIES})

add_library(lanekeeping include/platooning/LaneKeeping.hpp src/LaneKeeping.cpp)
add_dependencies(lanekeeping ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(lanekeeping ${catkin_LIBRARIES})

add_library(priorization include/platooning/Priorization.hpp src/Priorization.cpp)
add_dependencies(priorization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(priorization ${catkin_LIBRARIES})

add_library(protocol include/platooning/Protocol.hpp src/Protocol.cpp)
add_dependencies(protocol ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(protocol ${catkin_LIBRARIES})

add_library(userinterface include/platooning/UserInterface.hpp src/UserInterface.cpp)
add_dependencies(userinterface ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(userinterface ${catkin_LIBRARIES})

add_library(vehiclecontrol include/platooning/VehicleControl.hpp src/VehicleControl.cpp)
add_dependencies(vehiclecontrol ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(vehiclecontrol ${catkin_LIBRARIES})

add_library(wifi include/platooning/Wifi.hpp src/Wifi.cpp)
add_dependencies(wifi ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(wifi ${catkin_LIBRARIES})

add_library(template include/platooning/Template.hpp src/Template.cpp)
add_dependencies(template ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(template ${catkin_LIBRARIES})

add_executable(platooning_node src/platooning_node.cpp)
target_link_libraries(platooning_node ${catkin_LIBRARIES})
add_dependencies(platooning_node platooning_generate_messages_cpp)



##############################################
#### platooning module tests
##############################################
add_library(moduletest_protocol include/tests/Moduletest_protocol.hpp include/platooning/Protocol.hpp test/Moduletest_protocol.cpp)
add_dependencies(moduletest_protocol ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(moduletest_protocol ${catkin_LIBRARIES})


##################
## UNIT Testing ##
##################

## Add gtest based cpp test target and link libraries
catkin_add_gtest(remotecontrol-unit-test test/unittest_remotecontrol.cpp)
catkin_add_gtest(platooning-unit-test test/unittest_platooning.cpp)
catkin_add_gtest(lanekeeping-unit-test test/unittest_lanekeeping.cpp)
catkin_add_gtest(priorization-unit-test launch/priorization.launch test/unittest_priorization.cpp)
catkin_add_gtest(protocol-unit-test test/unittest_protocol.cpp)
catkin_add_gtest(userinterface-unit-test test/unittest_userinterface.cpp)
catkin_add_gtest(vehiclecontrol-unit-test test/unittest_vehiclecontrol.cpp)
catkin_add_gtest(wifi-unit-test launch/wifi.launch test/unittest_wifi.cpp)

catkin_add_gtest(template-unit-test test/unittest_template.cpp)

target_link_libraries(remotecontrol-unit-test ${catkin_LIBRARIES})
target_link_libraries(platooning-unit-test ${catkin_LIBRARIES})
target_link_libraries(lanekeeping-unit-test ${catkin_LIBRARIES})
target_link_libraries(priorization-unit-test ${catkin_LIBRARIES})
target_link_libraries(protocol-unit-test ${catkin_LIBRARIES})
target_link_libraries(userinterface-unit-test ${catkin_LIBRARIES})
target_link_libraries(vehiclecontrol-unit-test ${catkin_LIBRARIES})
target_link_libraries(wifi-unit-test ${catkin_LIBRARIES})

target_link_libraries(template-unit-test ${catkin_LIBRARIES})


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
install(TARGETS platooning_node environment_mapping remotecontrol lanekeeping priorization protocol userinterface vehiclecontrol wifi template
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark cpp header files for installation
install(DIRECTORY include/platooning/
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

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES launch/remotecontrol.launch launch/environment_mapping.launch launch/lanekeeping.launch launch/priorization.launch  protocol.launch  userinterface.launch  vehiclecontrol.launch  wifi.launch  template.launch
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#        )
#install(FILES plugins/remotecontrol.xml environment_mapping.xml lanekeeping.xml  priorization.xml  protocol.xml  userinterface.xml  vehiclecontrol.xml  wifi.xml  template.xml
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



## TESTSNIPPET
add_executable(testsnippet testsnippet.hpp testsnippet.cpp)
target_link_libraries(testsnippet ${catkin_LIBRARIES})
