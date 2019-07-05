#include <bwi_scavenger/file_editor.h>

/**
  Creates a file editor that can either add on to a file or read a file
*/
FileEditor::FileEditor(std::string filename, bool output){
  this->filename = filename;
  this->output = output;
  if(output){
    oFile = new std::ofstream();
    (*oFile).open(filename, std::ios::app);
  } else {
    std::ifstream temp(filename);
    iFile = new std::ifstream(filename);
  }
}

/**
  Returns the proof_id of the proof just read
*/
proof_id_t FileEditor::get_proof_id(){
  return proof.proof_id;
}

/**
  Returns whether or not the proof just read was correct, incorrect, or unverified
*/
int FileEditor::get_verification(){
  return proof.verification;
}

/**
  Returns the robot pose from where it submitted the proof from
*/
geometry_msgs::Pose FileEditor::get_robot_pose(){
  return proof.robot_pose;
}

/**
  Returns the pose of a secondary pose submitted with the proof
*/
geometry_msgs::Pose FileEditor::get_secondary_pose(){
  return proof.secondary_pose;
}

/**
  Returns the task name of the proof just read
*/
std::string FileEditor::get_task_name(){
  return proof.task_name;
}

/**
  Returns the parameter associated with the task of the proof just read
*/
std::string FileEditor::get_parameter(){
  return proof.parameter_name;
}


/**
  Reads one line of the file. Stores the information parsed within the FileEditor.

  @post: get_proof_id() returns proof_id
    get_correct() returns 1 if correct and 0 if not
    get_robot_pose() returns the pose the robot when proof was sent
    get_secondary_pose() returns other pose data
    get_task() returns a Task object that contain the task name and parameter
*/
bool FileEditor::read_line(){
  std::string str;

  if(std::getline(*iFile, str)){
    std::stringstream stream(str);
    std::string word;

    // reads each word and stores them with the correct item label
    for(int i = 0; i < NUM_ITEMS; i++){
      std::getline(stream, word, ',');

      if (i == PROOF_ID)
        proof.proof_id = std::atoi(word.c_str());

      else if (i == VERIFICATION)
        proof.verification = std::atoi(word.c_str());

      else if (i == ROBOT_POSE || i == SECONDARY_POSE){
        std::stringstream wordStream(word);
        std::string point;
        geometry_msgs::Pose *pose = (i == ROBOT_POSE) ? &(proof.robot_pose) : &(proof.secondary_pose);

        std::getline(wordStream, point, ' ');
        double temp = ::atof(point.c_str());
        (*pose).position.x = temp;

        std::getline(wordStream, point, ' ');
        temp = ::atof(point.c_str());
        (*pose).position.y = temp;

        std::getline(wordStream, point, ' ');
        temp = ::atof(point.c_str());
        (*pose).orientation.x = temp;

        std::getline(wordStream, point, ' ');
        temp = ::atof(point.c_str());
        (*pose).orientation.y = temp;

        std::getline(wordStream, point, ' ');
        temp = ::atof(point.c_str());
        (*pose).orientation.z = temp;

        std::getline(wordStream, point, ' ');
        temp = ::atof(point.c_str());
        (*pose).orientation.w = temp;

      } else if (i == TASK)
        proof.task_name = word;

      else if (i == PARAMETER)
        proof.parameter_name = word;
    }
    return true;
  }

  return false;
}

/**
  Writes one line to the file.

  @param proof_id recieved by sending proof
  @param verification 0 represents incorrect, 1 represents correct, 2 represents unverified
  @param task_name name of the task
  @param parameter_name name of the parameter associated with the task
  @param robot_pose pose of robot when the proof is sent
  @param secondary_pose a secondary pose that may be used depending on the task
*/
void FileEditor::write_to_file(proof_item proof){

    geometry_msgs::Pose pose = proof.robot_pose;
    std::string robot_pose_string = std::to_string(pose.position.x) + " " + std::to_string(pose.position.y) + " " +
      std::to_string(pose.orientation.x) + " " + std::to_string(pose.orientation.y) + " " + std::to_string(pose.orientation.z) + " " +
      std::to_string(pose.orientation.w);

    pose = proof.secondary_pose;
    std::string secondary_pose_string = std::to_string(pose.position.x) + " " + std::to_string(pose.position.y) + " " +
      std::to_string(pose.orientation.x) + " " + std::to_string(pose.orientation.y) + " " + std::to_string(pose.orientation.z) + " " +
      std::to_string(pose.orientation.w);

    (*oFile) << std::to_string(proof.proof_id) << "," << std::to_string(proof.verification) << "," << proof.task_name
      << "," << proof.parameter_name << "," << robot_pose_string << "," << secondary_pose_string << "\n";
}

/**
  Deletes the file and closes the stream
*/
void FileEditor::delete_file(){
  remove(filename.c_str());
  close();
}

/**
  Renames the file

  @param new_name name to rename the file to
*/
void FileEditor::rename_file(std::string new_name){
  rename(filename.c_str(), new_name.c_str());
}

/**
  Closes the streams
*/
void FileEditor::close(){
  if(output)
    (*oFile).close();
  else
    (*iFile).close();
}

FileEditor::~FileEditor(){
  delete oFile;
  delete iFile;
}
