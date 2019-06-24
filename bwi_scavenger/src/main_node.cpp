#include <bwi_scavenger/global_topics.h>
#include <map>
#include <ros/ros.h>
#include <scavenger_hunt/scavenger_hunt.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <vector>

#define CURRENT_TASK tasks[task_index]

static ScavengerHuntClient client("stefandebruyn@utexas.edu", "sick robots");
static std::vector<Task> tasks;
static int task_index = 0;
static double t_task_start, t_task_end;

static ros::Publisher pub_task_start, pub_yolo_node_target;

static bool hunt_started = false;
static bool conclude = false;

/**
  Advances the hunt to the next task.

  @param upload whether or not to upload a proof
*/
void next_task(bool upload=false) {
  if (hunt_started) {
    // Upload proof if requested by the last task node
    if (upload) {
      t_task_end = ros::Time::now().toSec();
      client.send_proof("proof.jpg", CURRENT_TASK, t_task_end - t_task_start);
    }

    task_index++;
  } else
    hunt_started = true;

  // Terminate
  if (conclude) {
    ROS_INFO("[main_node] Shutting down.");
    exit(0);
  }

  // End of hunt
  if (task_index == tasks.size()) {
    ROS_INFO("[main_node] Hunt complete.");

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
  ros::init(argc, argv, "main_node");
  ros::NodeHandle nh;

  pub_task_start = nh.advertise<std_msgs::String>(TPC_MAIN_NODE_TASK_START, 1);
  pub_yolo_node_target = nh.advertise<std_msgs::String>(TPC_YOLO_NODE_TARGET, 1);
  ros::Subscriber sub_task_complete = nh.subscribe(TPC_TASK_COMPLETE, 1, next_task_cb);

  client.get_hunt("BWI Lab Hunt", tasks);

  ros::Duration(1.0).sleep();

  next_task();

  ros::spin();
}
