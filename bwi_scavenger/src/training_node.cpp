#include <ros/ros.h>

#include <bwi_scavenger/globals.h>
#include <bwi_scavenger/file_editor.h>

#include <bwi_scavenger_msgs/DarknetAddTrainingFile.h>
#include <bwi_scavenger_msgs/DarknetStartTraining.h>

#include <scavenger_hunt/scavenger_hunt.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "training_node");
  ros::NodeHandle nh;

  ros::Publisher pub_training_file = nh.advertise<bwi_scavenger_msgs::DarknetAddTrainingFile>(
      TPC_DARKNET_NODE_ADD_TRAINING_FILE, 1);

  ros::Publisher pub_start_training = nh.advertise<bwi_scavenger_msgs::DarknetStartTraining>(
      TPC_DARKNET_NODE_START_TRAINING, 1);

  ros::Duration(0.5).sleep();  
  FileEditor *read = new FileEditor(PROOF_DATABASE_PATH, READ);

  while((*read).read_line()){
    proof_id_t proof_id = (*read).get_proof_id();
    int verification = (*read).get_verification();
    std::string task_name = (*read).get_task_name();
    std::string parameter_name = (*read).get_parameter();
    
    if(verification == PROOF_CORRECT){
      bwi_scavenger_msgs::DarknetAddTrainingFile training_file_msg;
      training_file_msg.network_name = task_name;
      training_file_msg.file_path = PROOF_COPY_MATERIAL_PATH + std::to_string(proof_id);
      training_file_msg.label = parameter_name;
      pub_training_file.publish(training_file_msg);
      std::cout << "publishing training file" << std::endl;
    }
  }
  (*read).close();

  //bwi_scavenger_msgs::DarknetStartTraining start_training_msg;  
  //start_training_msg.network_name = "Find object";
  //pub_start_training.publish(start_training_msg);

  ros::spin();
}