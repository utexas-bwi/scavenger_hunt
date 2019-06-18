#include <bwi_scavenger/global_topics.h>
#include <map>
#include <ros/ros.h>
#include <scavenger_hunt/scavenger_hunt.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

static ScavengerHuntClient client("robot@rock.com", "sick robots");
static ScavengerHunt *current_hunt;
static Task *current_task;
static int task_index = 0;

static ros::Publisher pub_task_start, pub_yolo_node_target;
static double taskTime;
static double startTime;

static bool initial_task = true;

void next_task(bool upload=false) {
  if (!initial_task && upload) {
    taskTime = ros::Time::now().toSec() - startTime;
    std::cout << "Time for task to complete: " << taskTime << std::endl;
    client.send_proof("proof.jpg", *current_task, taskTime);
    task_index++;
  }

  if (task_index == current_hunt->size()) {
    ROS_INFO("[main node] Hunt complete. Shutting down...");
    exit(0);
  }

  initial_task = false;
  current_task = (*current_hunt)[task_index];
  startTime = ros::Time::now().toSec();
  if (current_task->get_name() == "Find Object") {
    ROS_INFO("[main_node] Beginning Find Object protocol");

    // Set Find Object target
    std::map<std::string, std::string> parameters = current_task->get_parameters();
    std_msgs::String target_object;
    target_object.data = parameters["object"];
    pub_yolo_node_target.publish(target_object);

    // Begin Find Object
    std_msgs::String task;
    task.data = "Find Object";
    pub_task_start.publish(task);
  } else {
    ROS_INFO("[main_node] Do not have protocol for the task %s. Going on to next task.", current_task->get_name().c_str());
    task_index++;
    next_task();
  }
}

void next_task_cb(const std_msgs::Bool::ConstPtr &msg) {
  next_task(msg->data);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "main_node");
  ros::NodeHandle nh;

  pub_task_start = nh.advertise<std_msgs::String>(TPC_MAIN_NODE_TASK_START, 1);
  pub_yolo_node_target = nh.advertise<std_msgs::String>(TPC_YOLO_NODE_TARGET, 1);
  ros::Subscriber sub_task_complete = nh.subscribe(TPC_TASK_COMPLETE, 1, next_task_cb);

  current_hunt = client.get_hunt("Longhorn Hunt");

  ros::Duration(1.0).sleep();

  next_task();

  ros::spin();
}
