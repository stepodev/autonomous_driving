cmake_minimum_required(VERSION 2.8.3)
project(testnodelet)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation genmsg nodelet)



## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES listenerNodelet talkerNodelet
  CATKIN_DEPENDS nodelet roscpp
)


## Build
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(listenerNodelet src/listenerNodelet.cpp)
target_link_libraries(listenerNodelet ${catkin_LIBRARIES})

add_library(talkerNodelet src/talkerNodelet.cpp)
target_link_libraries(talkerNodelet ${catkin_LIBRARIES})

add_executable(listenerTalkerNode src/listenerTalkerNode.cpp)
target_link_libraries(listenerTalkerNode ${catkin_LIBRARIES})
add_dependencies(listenerTalkerNode testnodelet_generate_messages_cpp)
