#ifndef FILE_EDITOR_H
#define FILE_EDITOR_H

#include <fstream>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <scavenger_hunt/scavenger_hunt.h>

const int NUM_ITEMS = 6;

typedef struct{
  proof_id_t proof_id;
  int verification;
  geometry_msgs::Pose robot_pose;
  geometry_msgs::Pose secondary_pose;
  std::string task_name;
  std::string parameter_name;
} proof_item;

enum proof_item_num{
  PROOF_ID,
  VERIFICATION,
  TASK,
  PARAMETER,
  ROBOT_POSE,
  SECONDARY_POSE
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
  proof_item proof;
  bool output;

public:
  FileEditor(std::string filename, bool output);

  ~FileEditor();

  proof_id_t get_proof_id();

  int get_verification();

  geometry_msgs::Pose get_robot_pose();

  geometry_msgs::Pose get_secondary_pose();

  std::string get_task_name();

  std::string get_parameter();
  
  bool read_line();

  void write_to_file(proof_item proof);

  void delete_file();

  void rename_file(std::string name);

  void close();
};

#endif