#ifndef FILE_EDITOR_H
#define FILE_EDITOR_H

#include <fstream>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <scavenger_hunt/scavenger_hunt.h>

const int NUM_ITEMS = 6;

enum proof_item{
  PROOF_ID,
  VERIFICATION,
  ROBOT_POSE,
  SECONDARY_POSE,
  TASK,
  PARAMETER
};

enum verification{
  CORRECT,
  INCORRECT,
  UNVERIFIED
};

class FileEditor{
protected:
  std::string filename;
  std::ofstream *oFile;
  std::ifstream *iFile;
  std::string proof_id;
  std::string verification;
  geometry_msgs::Pose robot_pose;
  geometry_msgs::Pose secondary_pose;
  std::string task_name;
  std::string parameter_name;
  bool output;

public:
  FileEditor(std::string filename, bool output);

  ~FileEditor();

  proof_id_t get_proof_id();

  std::string get_verification();

  geometry_msgs::Pose get_robot_pose();

  geometry_msgs::Pose get_secondary_pose();

  std::string get_task_name();

  std::string get_parameter();
  
  bool read_line();

  void write_to_file(proof_id_t proof_id, int verification, std::string task_name, 
    std::string parameter_name, geometry_msgs::Pose robot_pose, geometry_msgs::Pose secondary_pose);

  void delete_file();

  void rename_file(std::string name);

  void close();
};

#endif