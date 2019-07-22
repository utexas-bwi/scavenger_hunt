cmake_minimum_required(VERSION 2.8.12)

project(bwi_scavenger_msgs)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  darknet_ros_msgs
  message_generation
)

add_message_files(FILES
  TaskProof.msg
  TaskStart.msg
  TaskEnd.msg
  RobotMove.msg
  RobotStop.msg
  PerceptionMoment.msg
  DatabaseProof.msg
  DarknetAddTrainingFile.msg
  DarknetStartTraining.msg
  DatabaseFile.msg
)

add_service_files(FILES
  DatabaseInfoSrv.srv
  PoseRequest.srv
)

generate_messages(DEPENDENCIES
  sensor_msgs
  std_msgs
  darknet_ros_msgs
)

catkin_package(CATKIN_DEPENDS
  sensor_msgs
  message_runtime
  std_msgs
  darknet_ros_msgs
)
