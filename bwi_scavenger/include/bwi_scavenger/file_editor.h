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

enum writing{
  READ,
  WRITE
};

/*
  FileEditor class that can read and write to files. Specialized for a "proofs file" that stores 
  the proof id, task name, parameter name, robot point, and a secondary point
*/
class FileEditor{
protected:
  std::string filename;
  std::ofstream *oFile;
  std::ifstream *iFile;
  proof_item proof;
  bool output;

public:
  /*
    Creates a FileEditor for either reading or writing a file

    @param filename path to the file you want to read/write to
    @param output boolean value for either reading or writing. Uses the enumerated type writing
  */
  FileEditor(std::string filename, bool output);

  ~FileEditor();

  /*
    Returns the proof id of the line last read
  */
  proof_id_t get_proof_id();

  /*
    Returns the verification state (enum) of the line last read
  */
  int get_verification();

  /*
    Returns the robot pose of the line last read
  */
  geometry_msgs::Pose get_robot_pose();

  /*
    Returns the secondary pose of the line last read
  */
  geometry_msgs::Pose get_secondary_pose();

  /*
    Returns the task name of the line last read
  */
  std::string get_task_name();

  /*
    Returns the parameter associated with the task of the line last read
  */
  std::string get_parameter();
  
  /*
    Reads the next line if available
    Returns false if there are no more lines in the file to read
    
    @post: get_proof_id() returns proof_id
          get_correct() returns 1 if correct and 0 if not
          get_robot_pose() returns the pose the robot when proof was sent
          get_secondary_pose() returns other pose data
          get_task() returns a Task object that contain the task name and parameter
  */
  bool read_line();

  /*
    Writes a proof to the file
    
    @param proof proof_item that stores the proof_id, task name, parameter
      robot pose and secondary pose
  */
  void write_to_file(proof_item proof);

  /*
    Deletes a file. Use only when you know you want to delete this file!
  */
  void delete_file();

  /*
    Renames the file.

    @param name name you want to change this filename to
  */
  void rename_file(std::string name);

  /*
    Closes the FileEditor, must be called after you are done editing
  */
  void close();
};

#endif