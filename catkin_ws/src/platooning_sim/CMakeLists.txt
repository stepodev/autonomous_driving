cmake_minimum_required(VERSION 2.8.3)
project(platooning_sim)

enable_language(CXX)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wall -g)

set(PRIUSCUP_MAJOR_VERSION 0)
set(PRIUSCUP_MINOR_VERSION 1)
set(PRIUSCUP_PATCH_VERSION 0)
set(PRIUSCUP_VERSION_FULL ${PRIUSCUP_MAJOR_VERSION}.${PRIUSCUP_MINOR_VERSION}.${PRIUSCUP_PATCH_VERSION})

set(prius_msgs_DIR ../../install/share/prius_msgs/cmake)
set(platooning_DIR ../../install/share/platooning/cmake)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nodelet
        gazebo_ros
        prius_msgs
        sensor_msgs
        )

find_package(platooning REQUIRED COMPONENTS
        platooning
        )

find_package(Boost REQUIRED COMPONENTS
        ## DUNNO LOL
        thread
        system
        )

find_package(gazebo 8 REQUIRED)
find_package(ignition-msgs0 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES gazeboadapter
        CATKIN_DEPENDS gazebo_ros
        #  DEPENDS system_lib
)

include_directories(include ../platooning/include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

add_library(PriusHybridPlugin SHARED plugins/PriusHybridPlugin.cc)
target_link_libraries(PriusHybridPlugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})

add_library(priustopics include/platooning_sim/topics.hpp ${catkin_LIBRARIES})
set_target_properties(priustopics PROPERTIES LINKER_LANGUAGE CXX)

add_library(gazeboadapter include/platooning_sim/gazeboadapter.hpp src/gazeboadapter.cpp)
add_dependencies(gazeboadapter priustopics)
target_link_libraries(gazeboadapter udpserver messagetypes topics priustopics ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})

install(TARGETS PriusHybridPlugin gazeboadapter
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY plugins/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins
        PATTERN ".svn" EXCLUDE)