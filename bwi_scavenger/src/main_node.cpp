#include <bwi_scavenger/global_topics.h>
#include <map>
#include <ros/ros.h>
#include <scavenger_hunt/scavenger_hunt.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <vector>

#define CURRENT_TASK tasks[task_index]

static ScavengerHuntClient client("robot@rock.com", "sick robots");
static std::vector<Task> tasks;
static int task_index = 0;
static double t_task_start, t_task_end;

static ros::Publisher pub_task_start, pub_yolo_node_target;

static bool initial_task = true;

void next_task(bool upload=false) {
  if (!initial_task && upload) {
    t_task_end = ros::Time::now().toSec();
    client.send_proof("proof.jpg", CURRENT_TASK, t_task_end - t_task_start);
    task_index++;
  }

  if (task_index == tasks.size()) {
    ROS_INFO("[main node] Hunt complete. Shutting down...");
    exit(0);
  }

  initial_task = false;

  if (CURRENT_TASK.get_name() == "Find Object") {
    ROS_INFO("[main_node] Beginning Find Object protocol");

    t_task_start = ros::Time::now().toSec();

    // Set Find Object target
    std::map<std::string, std::string> parameters = CURRENT_TASK.get_parameters();
    std_msgs::String target_object;
    target_object.data = parameters["object"];
    pub_yolo_node_target.publish(target_object);

    // Begin Find Object
    std_msgs::String task;
    task.data = "Find Object";
    pub_task_start.publish(task);
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

  client.get_hunt("Longhorn Hunt", tasks);

  ros::Duration(1.0).sleep();

  next_task();

  ros::spin();
}
