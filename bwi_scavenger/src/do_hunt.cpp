#include <bwi_scavenger/globals.h>
#include <bwi_scavenger_msgs/TaskEnd.h>
#include <bwi_scavenger_msgs/TaskStart.h>
#include <bwi_scavenger_msgs/DatabaseProof.h>

#include <map>
#include <ros/ros.h>
#include <scavenger_hunt/scavenger_hunt.h>
#include <bwi_scavenger/file_editor.h>
#include <bwi_scavenger/dbscan.h>
#include <std_msgs/String.h>
#include <vector>
#include <std_msgs/Bool.h>

#define CURRENT_TASK tasks[task_index]

static ScavengerHuntClient client("stefandebruyn@utexas.edu", "sick robots");
static std::vector<Task> tasks;
static int task_index = 0;
static double t_task_start;

static ros::Publisher pub_task_start, pub_done_parse, pub_proof;

static bool hunt_started = false;
static bool conclude = false;

static proof_item proof;

enum writing{
  READ,
  WRITE
};

/**
  Updates verification of the proof (incorrect or correct)
*/
void parse_proofs(){
  FileEditor *read = new FileEditor(PROOF_DATABASE_PATH, READ);
  FileEditor *write = new FileEditor(PROOF_DATABASE_PATH + ".tmp", WRITE);

  std::string str;
  while((*read).read_line()){
    proof.proof_id = (*read).get_proof_id();
    proof.verification = (*read).get_verification();
    if(proof.verification == UNVERIFIED)
      proof.verification = client.get_proof_status(proof.proof_id);

    if(proof.proof_id){
      proof.task_name = (*read).get_task_name();
      proof.parameter_name = (*read).get_parameter();
      proof.robot_pose = (*read).get_robot_pose();
      proof.secondary_pose = (*read).get_secondary_pose();

      if(proof.verification != UNVERIFIED){
        bwi_scavenger_msgs::DatabaseProof proofMsg;
        proofMsg.task = proof.task_name;
        proofMsg.param = proof.parameter_name;
        proofMsg.robot_pose = proof.robot_pose;
        proofMsg.secondary_pose = proof.secondary_pose;
        proofMsg.verification = proof.verification;
        pub_proof.publish(proofMsg);
        ros::Duration(0.25).sleep();
      }

      (*write).write_to_file(proof);
    }
  }
  (*read).close();
  (*write).close();

  (*read).delete_file();
  // will delete invalid proofs (based on proof id)
  (*write).rename_file(PROOF_DATABASE_PATH);

  std_msgs::Bool msg;
  pub_done_parse.publish(msg);

  ros::Duration(1.0).sleep();

}

/**
  Advances the hunt to the next task.

  @param upload whether or not to upload a proof
*/
void next_task(const bwi_scavenger_msgs::TaskEnd::ConstPtr &msg) {
  if (hunt_started) {
    // Upload proof if requested by the last task node
    if (msg->success) {
      double task_duration = ros::Time::now().toSec() - t_task_start;

      proof.task_name = CURRENT_TASK.get_name();
      proof.parameter_name = CURRENT_TASK.get_parameter_value("object");
      proof.proof_id = client.send_proof(PROOF_MATERIAL_PATH,
                                         CURRENT_TASK,
                                         task_duration);
      proof.verification = UNVERIFIED;
      proof.robot_pose = msg->robot_pose;
      proof.secondary_pose = msg->secondary_pose;

      FileEditor *fe = new FileEditor(PROOF_DATABASE_PATH, WRITE);
      fe->write_to_file(proof);
      fe->close();
    }

    task_index++;
  } else {
    ROS_INFO("[main_node] Starting hunt.");
    hunt_started = true;
    parse_proofs();
  }

  // Terminate
  if (conclude) {
    ROS_INFO("[main_node] Shutting down.");
    exit(0);
  }

  // End of hunt
  if (task_index == tasks.size()) {
    ROS_INFO("[main_node] Hunt complete.");
    ros::Duration(6.0).sleep();

    // Begin Conclude
    bwi_scavenger_msgs::TaskStart task;
    task.name = TASK_CONCLUDE;
    pub_task_start.publish(task);
    conclude = true;
    return;
  }

/*------------------------------------------------------------------------------
FIND OBJECT
------------------------------------------------------------------------------*/
  if (CURRENT_TASK.get_name() == TASK_FIND_OBJECT) {
    ROS_INFO("[main_node] Waking up find_object_node...");

    t_task_start = ros::Time::now().toSec();

    // Fetch parameter
    std::string target_object = CURRENT_TASK.get_parameter_value("object");

    // Initiate task
    bwi_scavenger_msgs::TaskStart task;
    task.name = TASK_FIND_OBJECT;
    task.parameters.push_back(target_object);
    pub_task_start.publish(task);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hunt_node");
  ros::NodeHandle nh;

  pub_task_start = nh.advertise<bwi_scavenger_msgs::TaskStart>(
      TPC_TASK_START, 1);
  pub_done_parse = nh.advertise<std_msgs::Bool>(
      TPC_DATABASE_NODE_DONE_PARSE, 1);
  pub_proof = nh.advertise<bwi_scavenger_msgs::DatabaseProof>(
      TPC_DATABASE_NODE_UPDATE_PROOF, 1);

  ros::Subscriber sub_task_complete = nh.subscribe(TPC_TASK_END, 1, next_task);

  if (argc < 2) {
    ROS_ERROR("Usage: do_hunt <hunt name>");
    exit(0);
  }

  client.get_hunt(argv[1], tasks);
  next_task(nullptr);
  ros::spin();
}
