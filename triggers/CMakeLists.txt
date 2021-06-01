cmake_minimum_required(VERSION 3.0.2)
project(triggers)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation)

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Trigger.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

