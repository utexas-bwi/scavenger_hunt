#include <bwi_scavenger/file_editor.h>
#include <scavenger_hunt/rapidxml.hpp>
#include <scavenger_hunt/rapidxml_print.hpp>


FileEditor::FileEditor(std::string filename, bool output){
  this->filename = filename;
  this->output = output;
  
  if(output){ // write  
    std::ifstream file(filename);

    if(file.peek() == std::ifstream::traits_type::eof()){
      std::cout << "empty file " << std::endl;
    } else {
      file.seekg(0, file.end);
      size_t length = file.tellg();
      file.seekg(0, file.beg);
      std::cout << "length: " << length << std::endl;
      buffer = new char[length + 1];
      file.read(buffer, length);
	    doc.parse<0>(buffer);
    }
    buffer = new char[1];
    std::cout << "write" << std::endl;
    file.close();
    
  } else { // read

    std::ifstream file(filename);

    file.seekg(0, file.end);
    size_t length = file.tellg();
    file.seekg(0, file.beg);
    // char* temp = new char[length + 1];
    buffer  = new char[length + 1];
    std::cout << "length: " << length << std::endl;
    file.read(buffer, length);
	  doc.parse<0>(buffer);
    std::cout << "read" << std::endl;
    
    // setting nodes for read
    task_node = doc.first_node("task");
    if(task_node){
      proof.task_name = task_node->first_attribute("name")->value();
	    parameter_node = task_node->first_node("parameter");
      proof.parameter_name = parameter_node->first_attribute("name")->value();
      proof_node = NULL;
    }
    file.close();
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

void set_pose(node *pose_node, geometry_msgs::Pose *pose){
  if(!pose_node){
    pose->position.x = 0;
    pose->position.y = 0;
    pose->position.z = 0;

    pose->orientation.x = 0;
    pose->orientation.y = 0;
    pose->orientation.z = 0;
    pose->orientation.w = 0;
  } else {
    pose->position.x = std::atoi(pose_node->first_attribute("pos_x")->value());
    pose->position.y = std::atoi(pose_node->first_attribute("pos_y")->value());
    pose->position.z = std::atoi(pose_node->first_attribute("pos_z")->value());

    pose->orientation.x = std::atoi(pose_node->first_attribute("orien_x")->value());
    pose->orientation.y = std::atoi(pose_node->first_attribute("orien_y")->value());
    pose->orientation.z = std::atoi(pose_node->first_attribute("orien_z")->value());
    pose->orientation.w = std::atoi(pose_node->first_attribute("orien_w")->value());
  }
}

bool FileEditor::read_line(){

  if(!task_node){ // no more tasks listed in the file
    std::cout << "no task node" << std::endl;
    return false;
  }
  
  if(proof_node){
    proof_node = proof_node->next_sibling();
  }
  else
    proof_node = parameter_node->first_node("proof");

  while(!proof_node){ // reached last proof, get next available one
    parameter_node = parameter_node->next_sibling();
    while(!parameter_node){ // reached last parameter, get next available one
      task_node = task_node->next_sibling();
      if(!task_node){ // reached end of xml
        std::cout << "end of file" << std::endl;
        return false;
      }
      proof.task_name = task_node->first_attribute("name")->value();
      parameter_node = task_node->first_node("parameter");
    }
    proof.parameter_name = parameter_node->first_attribute("name")->value();
    proof_node = parameter_node->first_node("proof");
  }
  std::cout << "[read]checking proof node" << std::endl;
  if(!proof_node){
    std::cout << "proof node doesn't exist" << std::endl;
    exit(0);
  } else if (!(proof_node->first_attribute("id"))){
    std::cout << "attr doesn't exist" << std::endl;
    exit(0);
  }
  std::cout << proof_node->first_attribute("id")->value() << std::endl;
  proof.proof_id = std::atoi(proof_node->first_attribute("id")->value());

  std::string ver = proof_node->first_attribute("verification")->value();
  proof.verification = std::atoi(ver.c_str());

  std::cout << "[read] checking robot pose" << std::endl;
  node* pose_node = proof_node->first_node("robot_pose");
  set_pose(pose_node, &proof.robot_pose);

  std::cout << "[read] checking second pose" << std::endl;
  pose_node = proof_node->first_node("secondary_pose");
  set_pose(pose_node, &proof.secondary_pose);
  return true;
}

/*
 * Converts an int into a string and stores the string in a vector for keeping when the file is written
 */
char* int_to_string(int num, std::vector<char*> *strings_keeper){
  std::string s(std::to_string(num));

  const char* org_string = s.c_str();
  char* new_string = new char[s.length() + 1];
  int length = s.length() + 1;

  for(int i = 0 ; i < length; i++)
    new_string[i] = org_string[i];  

  strings_keeper->push_back(new_string); // copies string into list so it exists when it is written
  return new_string;
}

node* create_bb_points_node(xml_doc* doc, float* points, node* points_node, std::vector<char*> *strings_keeper){
  char* x_min = int_to_string(points[0], strings_keeper);
  char* x_max = int_to_string(points[1], strings_keeper);
  char* y_min = int_to_string(points[2], strings_keeper);
  char* y_max = int_to_string(points[3], strings_keeper);

  points_node->append_attribute(doc->allocate_attribute("x_min", x_min));
  points_node->append_attribute(doc->allocate_attribute("x_max", x_max));
  points_node->append_attribute(doc->allocate_attribute("y_min", y_min));
  points_node->append_attribute(doc->allocate_attribute("y_max", y_max));

  return points_node;
}

/*
 * Creates a new pose node to be appended to a proof node
 */
node* create_pose_node(xml_doc* doc, geometry_msgs::Pose pose, node* pose_node, std::vector<char*> *strings_keeper){
  char* pos_x = int_to_string(pose.position.x, strings_keeper);
  char* pos_y = int_to_string(pose.position.y, strings_keeper);
  char* pos_z = int_to_string(pose.position.z, strings_keeper);

  char* orien_x = int_to_string(pose.orientation.x, strings_keeper);
  char* orien_y = int_to_string(pose.orientation.y, strings_keeper);
  char* orien_z = int_to_string(pose.orientation.z, strings_keeper);
  char* orien_w = int_to_string(pose.orientation.w, strings_keeper);

  pose_node->append_attribute(doc->allocate_attribute("pos_x", pos_x));
  pose_node->append_attribute(doc->allocate_attribute("pos_y", pos_y));
  pose_node->append_attribute(doc->allocate_attribute("pos_z", pos_z));

  pose_node->append_attribute(doc->allocate_attribute("orien_x", orien_x));
  pose_node->append_attribute(doc->allocate_attribute("orien_y", orien_y));
  pose_node->append_attribute(doc->allocate_attribute("orien_z", orien_z));
  pose_node->append_attribute(doc->allocate_attribute("orien_w", orien_w));

  return pose_node;
}

/*
 * Creates a new proof node with the attributes of the proof item
 */
node* create_proof_node(xml_doc* doc, proof_item proof, std::vector<char*> *strings_keeper){

  node* proof_node = doc->allocate_node(rapidxml::node_element, "proof");

  // proof_id
  char* proof_id = int_to_string(proof.proof_id, strings_keeper);
  proof_node->append_attribute(doc->allocate_attribute("id", proof_id));

  // verification
  char* verification = int_to_string(proof.verification, strings_keeper);
  proof_node->append_attribute(doc->allocate_attribute("verification", verification));

  // robot_pose
  node* robot_pose_node = doc->allocate_node(rapidxml::node_element, "robot_pose");
  robot_pose_node = create_pose_node(doc, proof.robot_pose, robot_pose_node, strings_keeper);
  proof_node->append_node(robot_pose_node);

  // secondary_pose
  node* secondary_pose_node = doc->allocate_node(rapidxml::node_element, "secondary_pose");
  secondary_pose_node = create_pose_node(doc, proof.secondary_pose, secondary_pose_node, strings_keeper);
  proof_node->append_node(secondary_pose_node);

  // bounding_box_points
  node* bounding_box_points_node = doc->allocate_node(rapidxml::node_element, "bounding_box");
  bounding_box_points_node = create_bb_points_node(doc, proof.bounding_box_points, bounding_box_points_node, strings_keeper);
  proof_node->append_node(bounding_box_points_node);

  return proof_node;
}

/*
 * Edits the preexisting proof_node with the values stored in the proof item
 *
 * Only verfication at this state, can be modified to edit other nodes
 */
void edit_proof_node(node* proof_node, proof_item proof, std::vector<char*> *strings_keeper){
  char* verification = int_to_string(proof.verification, strings_keeper);
  proof_node->first_attribute("verification")->value(verification);
}

void FileEditor::write_to_file(proof_item proof){

  task_node = doc.first_node("task");
  while(task_node && task_node->first_attribute("name")->value() != proof.task_name)
    task_node = task_node->next_sibling();
  
  
  if(!task_node){ // create new task 
    node* task_temp = doc.allocate_node(rapidxml::node_element, "task");
    task_temp->append_attribute(doc.allocate_attribute("name", proof.task_name.c_str()));
    doc.append_node(task_temp);
    task_node = task_temp;
  }

  parameter_node = task_node->first_node("parameter");
  while(parameter_node && parameter_node->first_attribute("name")->value() != proof.parameter_name)
    parameter_node = parameter_node->next_sibling();

  if(!parameter_node){ // create new parameter
    node* parameter_temp = doc.allocate_node(rapidxml::node_element, "parameter");
    parameter_temp->append_attribute(doc.allocate_attribute("name", proof.parameter_name.c_str()));
    task_node->append_node(parameter_temp);
    parameter_node = parameter_temp;
  }

  proof_node = parameter_node->first_node("proof");
  while(proof_node && proof_node->first_attribute("id")->value() != std::to_string(proof.proof_id))
    proof_node = proof_node->next_sibling();

  if(!proof_node){ //create new proof because it doesn't already exist
    proof_node = create_proof_node(&doc, proof, &strings_keeper);
    parameter_node->append_node(proof_node);

    return;
  }

  edit_proof_node(proof_node, proof, &strings_keeper); // edit preexisting proof
}

void FileEditor::delete_file(){
  remove(filename.c_str());
  close();
}

void FileEditor::rename_file(std::string new_name){
  rename(filename.c_str(), new_name.c_str());
}

void FileEditor::close(){
  if(output){
    std::ofstream write_file(filename);
    write_file << doc;
    write_file.close();
  }
}

FileEditor::~FileEditor(){
  delete task_node;
  delete parameter_node;
  delete proof_node;
  // free(iFile);
  int length = strings_keeper.size();
  for(int i = 0; i < length; i++)
    free(strings_keeper[i]);
  free(proof.bounding_box_points);
}

void FileEditor::print_buffer(){
  std::cout << "printing buffer: " << std::endl;
  std::cout << buffer << std::endl;
  // for(int i = 0 ; i < 1014; i++){
  //   std::cout << buffer[i] << std::endl;
  //   getchar();
  // }
}
