#include <bwi_scavenger/global_topics.h>
#include <map>
#include <ros/ros.h>
#include <scavenger_hunt/scavenger_hunt.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <geometry_msgs/Pose.h>

#define CURRENT_TASK tasks[task_index]
#define PATH_TO_FILE "/home/scavenger_hunt/proofs.txt"

static ScavengerHuntClient client("jsuriadinata@utexas.edu ", "Tr3asure");
static std::vector<Task> tasks;
static int task_index = 0;
static double t_task_start, t_task_end;

static ros::Publisher pub_task_start, pub_yolo_node_target, pub_location;

static bool hunt_started = false;
static bool conclude = false;

static bool can_write = false;
static std::string taskToWrite;
static std::string paramToWrite;

void write_file(const geometry_msgs::Pose::ConstPtr &pose){
  // writing to a proofs file to update learning model
  if(can_write){
    ROS_INFO("[main_node] Writing to file");
    std::string location;
    location = "(" + std::to_string(pose->position.x) + ", " + std::to_string(pose->position.y) + ")";
    std::ofstream proofsFile;
    proofsFile.open("proofs.txt", std::ios::app);
    std::string proof = "0, " + location + ", " + taskToWrite + ", " + paramToWrite;
    proofsFile << proof << "\n";
    proofsFile.close();
    can_write = false;
  }
}

/**
  Advances the hunt to the next task.

  @param upload whether or not to upload a proof
*/
void next_task(bool upload=false) {
  if (hunt_started) {
    // Upload proof if requested by the last task node
    if (upload) {
      // saving task and parameter to write to file with location
      taskToWrite = CURRENT_TASK.get_name();
      paramToWrite = CURRENT_TASK.get_parameter_value("object");
      can_write = true;
      std_msgs::Bool msg;
      pub_location.publish(msg);

      t_task_end = ros::Time::now().toSec();
      client.send_proof("proof.jpg", CURRENT_TASK, t_task_end - t_task_start);
    }

    task_index++;
  } else {
    ROS_INFO("[main_node] Starting Hunt.");
    hunt_started = true;
    // reading proofs file and proof xml to add to decision tree
    std::ifstream proofsFile("proofs.txt");
    std::string str;
    //thrown away first line
    std::getline(proofsFile,str);
    int count = 0;
    while(std::getline(proofsFile, str)){
      count++;
    }
    ROS_INFO("[main_node] Successful read through %x proofs", count);

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
    std_msgs::String task;
    task.data = "Conclude";
    pub_task_start.publish(task);
    conclude = true;
    return;
  }

/*******************************************************************************
  FIND OBJECT TASK
*******************************************************************************/
  if (CURRENT_TASK.get_name() == "Find Object") {
    ROS_INFO("[main_node] Beginning Find Object protocol");

    t_task_start = ros::Time::now().toSec();

    // Set object target
    std_msgs::String target_object;
    target_object.data = CURRENT_TASK.get_parameter_value("object");
    pub_yolo_node_target.publish(target_object);

    // Begin Find Object
    std_msgs::String task;
    task.data = "Find Object";
    pub_task_start.publish(task);
  }
}

/**
  @brief called when a task node finishes its task
*/
void next_task_cb(const std_msgs::Bool::ConstPtr &msg) {
  next_task(msg->data);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hunt_node");
  ros::NodeHandle nh;

  pub_task_start = nh.advertise<std_msgs::String>(TPC_MAIN_NODE_TASK_START, 1);
  pub_yolo_node_target = nh.advertise<std_msgs::String>(TPC_YOLO_NODE_TARGET, 1);
  pub_location = nh.advertise<std_msgs::Bool>(TPC_MOVE_NODE_REQUEST_POSE, 1);

  ros::Subscriber sub_task_complete = nh.subscribe(TPC_TASK_COMPLETE, 1, next_task_cb);
  ros::Subscriber sub_location = nh.subscribe(TPC_MOVE_NODE_ROBOT_POSE, 1, write_file);

  if (argc < 2) {
    ROS_ERROR("Usage: do_hunt <hunt name>");
    exit(0);
  }

  client.get_hunt(argv[1], tasks);

  ros::Duration(1.0).sleep();

  next_task();

  ros::spin();
}
