
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <bwi_scavenger/globals.h>
#include <bwi_scavenger/paths.h>

#include <bwi_scavenger_msgs/PoseRequest.h>
#include <bwi_scavenger_msgs/TaskEnd.h>
#include <bwi_scavenger_msgs/TaskStart.h>
#include <bwi_scavenger_msgs/DarknetAddTrainingFile.h>
#include <bwi_scavenger_msgs/DarknetStartTraining.h>

#include <bwi_scavenger/file_editor.h>
#include <bwi_scavenger/database_node.h>

#include <ros/ros.h>

#include <scavenger_hunt/scavenger_hunt.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <map>
#include <vector>

#define FAILED_PROOF_UPLOAD -1
#define CURRENT_TASK tasks[task_index]

// Image width and height for kinect.
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

static ScavengerHuntClient *client;
static std::vector<Task> tasks;
static int task_index = 0;
static double t_task_start;

static ros::Publisher pub_task_start, pub_done_parse, pub_proof, pub_training_file;
static ros::ServiceClient client_database_info_request;

static bool hunt_started = false;
static bool conclude = false;

static proof_item proof;

void send_training_file(){

  // gets image and encodes it to be sent to the training node

  cv::Mat image = cv::imread(paths::proof_materials_repo() + "/" + std::to_string(proof.proof_id));
  cv::waitKey(30);
  sensor_msgs::ImagePtr imagePtr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  bwi_scavenger_msgs::DarknetAddTrainingFile training_file_msg;
  training_file_msg.image = *imagePtr;
  training_file_msg.network_name = "Test";

  training_file_msg.label = proof.parameter_name;

  // bounding box coordinates are stored in the the "orientation" of the secondary pose
  training_file_msg.xmin = proof.secondary_pose.orientation.x;
  training_file_msg.xmax = proof.secondary_pose.orientation.y;
  training_file_msg.ymin = proof.secondary_pose.orientation.z;
  training_file_msg.ymax = proof.secondary_pose.orientation.w;

  training_file_msg.image_width = IMAGE_WIDTH;
  training_file_msg.image_height = IMAGE_HEIGHT;
  pub_training_file.publish(training_file_msg);
}

/**
  Updates verification of the proof (incorrect or correct)
*/
void parse_proofs(){
  FileEditor *read = new FileEditor(paths::proof_db(), READ);
  FileEditor *write = new FileEditor(paths::proof_db() + ".tmp", WRITE);

  while((*read).read_line()){
    proof.proof_id = (*read).get_proof_id();
    // updates proof if proof id is not 0 (thus it has been sent to the server)
    if(proof.proof_id != FAILED_PROOF_UPLOAD){

      proof.task_name = (*read).get_task_name();
      proof.parameter_name = (*read).get_parameter();
      proof.robot_pose = (*read).get_robot_pose();
      proof.secondary_pose = (*read).get_secondary_pose();
      proof.verification = (*read).get_verification();
      // updates verification if it has been recently verified
      if(proof.verification == UNVERIFIED){
        proof.verification = client->get_proof_status(proof.proof_id);

        // sends the training file to darknet
        if(proof.verification == PROOF_CORRECT){
          send_training_file();

          // ROS_INFO("[main_node] Publishing training file.");
        }
      }

      // sends message of proof to database node
      if(proof.verification != UNVERIFIED){
        bwi_scavenger_msgs::DatabaseInfoSrv add_proof;
        add_proof.request.task_name = proof.task_name;
        add_proof.request.parameter_name = proof.parameter_name;
        add_proof.request.data = ADD_PROOF;
        add_proof.request.verification = proof.verification;
        add_proof.request.pose = proof.robot_pose;
        add_proof.request.secondary_pose = proof.secondary_pose;
        // ROS_INFO("[main_node] Adding proof to clusterer");
        client_database_info_request.call(add_proof);
      }

      (*write).write_to_file(proof);
    }
  }
  (*read).close();
  (*write).close();

  (*read).delete_file();
  (*write).rename_file(paths::proof_db()); // will delete proofs not uploaded to server

  bwi_scavenger_msgs::DatabaseInfoSrv done;
  done.request.data = CREATE_CLUSTERS;
  client_database_info_request.call(done);
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
      proof.proof_id = client->send_proof(paths::proof_material(),
                                         CURRENT_TASK,
                                         task_duration);
      proof.verification = UNVERIFIED;
      proof.robot_pose = msg->robot_pose;
      proof.secondary_pose = msg->secondary_pose;

      // save extra copy of image to separate folder

      // std::stringstream ss;
      // std::string temp;
      // ss << proof.task_name;
      // std::string task_name_no_spaces = "";
      // // create a string with no spaces for the task name
      // while (!ss.eof())
      // {
      //     ss >> temp;
      //     task_name_no_spaces += temp;
      // }

      // std::string proof_copy = std::to_string(proof.proof_id) + "_" + task_name_no_spaces + "_" + proof.parameter_name;

      std::string proof_copy = paths::proof_materials_repo() + "/" + std::to_string(proof.proof_id);
      std::string command = "cp " + paths::proof_material() + " " + proof_copy;
      system(command.c_str());

      FileEditor *fe = new FileEditor(paths::proof_db(), WRITE);
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

  std::string scav_email, scav_pass;
  nh.param("bwi_scavenger/scav_hunt_login/email", scav_email, std::string(""));
  nh.param("bwi_scavenger/scav_hunt_login/password", scav_pass, std::string(""));

  client = new ScavengerHuntClient(scav_email, scav_pass);

  pub_task_start = nh.advertise<bwi_scavenger_msgs::TaskStart>(
      TPC_TASK_START, 1);
  pub_done_parse = nh.advertise<std_msgs::Bool>(
      TPC_DATABASE_NODE_DONE_PARSE, 1);
  pub_proof = nh.advertise<bwi_scavenger_msgs::DatabaseProof>(
      TPC_DATABASE_NODE_UPDATE_PROOF, 1);
  pub_training_file = nh.advertise<bwi_scavenger_msgs::DarknetAddTrainingFile>(
      TPC_DARKNET_NODE_ADD_TRAINING_FILE, 1000);

  ros::Subscriber sub_task_complete = nh.subscribe(TPC_TASK_END, 1, next_task);

  client_database_info_request = nh.serviceClient<bwi_scavenger_msgs::DatabaseInfoSrv>(
      SRV_DATABASE_INFO_REQUEST);

  if (argc < 2) {
    ROS_ERROR("Usage: do_hunt <hunt name>");
    exit(0);
  }

  ros::Duration(2.0).sleep();

  client->get_hunt(argv[1], tasks);
  next_task(nullptr);
  ros::spin();
}
