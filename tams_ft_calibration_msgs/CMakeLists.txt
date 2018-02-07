cmake_minimum_required(VERSION 2.8.3)
project(tams_ft_calibration_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Payload.msg
 )

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   Calibration.srv
   SetToolWeight.srv
 )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

