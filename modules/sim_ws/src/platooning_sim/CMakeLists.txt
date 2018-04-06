cmake_minimum_required(VERSION 2.8.3)
project(platooning_sim)

enable_language(CXX)

add_compile_options(-std=c++11 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nodelet
        platooning
        prius_msgs
        gazebo_ros
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
        LIBRARIES gazeboadapter camerafollowplugin
        CATKIN_DEPENDS gazebo_ros
        # DEPENDS system_lib
)

include_directories(include ${platooning_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${IGNITION-MSGS_INCLUDE_DIRS})

add_library(priustopics include/platooning_sim/topics.hpp )
set_target_properties(priustopics PROPERTIES LINKER_LANGUAGE CXX)

add_library(camerafollowplugin SHARED include/platooning_sim/gazebo_camera_follow.hpp src/gazebo_camera_follow.cpp)
target_link_libraries(camerafollowplugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})

add_library(gazeboadapter SHARED include/platooning_sim/gazeboadapter.hpp src/gazeboadapter.cpp)
add_dependencies(gazeboadapter priustopics )
target_link_libraries(gazeboadapter priustopics ${platooning_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS gazeboadapter camerafollowplugin
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY plugins/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins
        PATTERN ".svn" EXCLUDE)

