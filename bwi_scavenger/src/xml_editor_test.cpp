#include <bwi_scavenger/file_editor.h>

int main(int argc, char** argv){
  std::cout << "Begin FileEditor test" << std::endl;
  
  proof_item proof;
  proof.verification = UNVERIFIED;
  proof.task_name = "task_name";
  proof.parameter_name = "parameter_name";

  proof.robot_pose.position.x = 1;
  proof.robot_pose.position.y = 2;
  proof.robot_pose.position.z = 3;

  proof.robot_pose.orientation.x = 4;
  proof.robot_pose.orientation.y = 5;
  proof.robot_pose.orientation.z = 6;
  proof.robot_pose.orientation.w = 7;

  proof.secondary_pose.position.x = 1;
  proof.secondary_pose.position.y = 2;
  proof.secondary_pose.position.z = 3;

  proof.secondary_pose.orientation.x = 4;
  proof.secondary_pose.orientation.y = 5;
  proof.secondary_pose.orientation.z = 6;
  proof.secondary_pose.orientation.w = 7;

  proof.bounding_box_points[0] = 1;
  proof.bounding_box_points[1] = 2;
  proof.bounding_box_points[2] = 3;
  proof.bounding_box_points[3] = 4;

  for(int i = 4; i < 7; i++){
    getchar();
    proof.proof_id = i;
    FileEditor *fe = new FileEditor("/home/bwilab/scavenger_hunt_ws/BWI_SCAVENGER_PROOF_DB_2.xml.tmp", WRITE);

    fe->write_to_file(proof);

    fe->close();
  }

  // FileEditor *fe = new FileEditor("/home/bwilab/scavenger_hunt_ws/BWI_SCAVENGER_PROOF_DB.dat", READ);

  // while(fe->read_line()){
  //   geometry_msgs::Pose robot_pose = fe->get_robot_pose();
  //   geometry_msgs::Pose secondary_pose = fe->get_secondary_pose();
  //   std::cout << "task: " << fe->get_task_name() << " parameter: " << fe->get_parameter() << std::endl;
  //   std::cout << "  proof: " << std::to_string(fe->get_proof_id()) << " verified: " << std::to_string(fe->get_verification()) 

  //     << " robot pose: (" << std::to_string(robot_pose.position.x) << ", " << std::to_string(robot_pose.position.y) << ", " <<
  //     std::to_string(robot_pose.position.z) << ") (" << std::to_string(robot_pose.orientation.x) << ", " <<
  //     std::to_string(robot_pose.orientation.y) << ", " << std::to_string(robot_pose.orientation.z) << ", " <<
  //     std::to_string(robot_pose.orientation.w) << ") "
      
  //     << " secondary pose: (" << std::to_string(secondary_pose.position.x) << ", " << std::to_string(secondary_pose.position.y) << ", " <<
  //     std::to_string(secondary_pose.position.z) << ") (" << std::to_string(secondary_pose.orientation.x) << ", " <<
  //     std::to_string(secondary_pose.orientation.y) << ", " << std::to_string(secondary_pose.orientation.z) << ", " <<
  //     std::to_string(secondary_pose.orientation.w) << ") "
       
  //     << std::endl;
  // }
  // fe->close();

}