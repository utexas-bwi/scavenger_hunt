#include <bwi_scavenger/file_editor.h>

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

proof_id_t FileEditor::get_proof_id(){
  return proof.proof_id;
}

int FileEditor::get_verification(){
  return proof.verification;
}

geometry_msgs::Pose FileEditor::get_robot_pose(){
  return proof.robot_pose;
}

geometry_msgs::Pose FileEditor::get_secondary_pose(){
  return proof.secondary_pose;
}

std::string FileEditor::get_task_name(){
  return proof.task_name;
}

std::string FileEditor::get_parameter(){
  return proof.parameter_name;
}

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

void FileEditor::delete_file(){
  remove(filename.c_str());
  close();
}

void FileEditor::rename_file(std::string new_name){
  rename(filename.c_str(), new_name.c_str());
}

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
