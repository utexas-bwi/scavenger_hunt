cmake_minimum_required(VERSION 2.8.12)

project(bwi_scavenger_msgs)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  geometry_msgs
  darknet_ros_msgs
  scavenger_hunt_msgs
  darksocket_ros
  message_generation
)

add_message_files(FILES
  TaskEnd.msg
  RobotMove.msg
  RobotStop.msg
  PerceptionMoment.msg
  DarknetAddTrainingFile.msg
  DarknetStartTraining.msg
  DatabaseFile.msg
  ObjectProbabilities.msg
  ObjmemAdd.msg
  OccurrenceModel.msg
  MultitaskStart.msg
)

add_service_files(FILES
  GetOccurrenceModel.srv
  PoseRequest.srv
  SaveOccurrenceModel.srv
  SendProof.srv
  ConfirmObject.srv
  GetPriorityPoints.srv
  ObjmemDump.srv
)

generate_messages(DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
  darknet_ros_msgs
  scavenger_hunt_msgs
  darksocket_ros
)

catkin_package(CATKIN_DEPENDS
  sensor_msgs
  geometry_msgs
  message_runtime
  std_msgs
  darknet_ros_msgs
  scavenger_hunt_msgs
  darksocket_ros
)
