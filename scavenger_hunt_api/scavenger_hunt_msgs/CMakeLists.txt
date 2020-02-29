cmake_minimum_required(VERSION 2.8.12)

project(scavenger_hunt_msgs)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(FILES
  Login.msg
  Task.msg
  Hunt.msg
  Parameter.msg
  Proof.msg
)

add_service_files(FILES
  GetHunt.srv
  GetProofStatus.srv
  SendProof.srv
)

generate_messages(DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  sensor_msgs
  message_runtime
  std_msgs
)
