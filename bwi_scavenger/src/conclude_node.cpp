#include <bwi_scavenger/global_topics.h>
#include <bwi_scavenger/RobotMove.h>
#include <bwi_scavenger/RobotStop.h>
#include <bwi_scavenger/robot_motion.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

static const char TELEM_TAG[] = "[conclude_node]";
static bool node_active = false;

static ros::Publisher pub_move;
static ros::Publisher pub_task_complete;

// Called when the main node is starting a task
void task_start_cb(const std_msgs::String::ConstPtr &msg) {
  if (msg->data == "Conclude") {
    ROS_INFO("%s Conclusion protocol firing!", TELEM_TAG);
    node_active = true;
    ros::Duration(1.0).sleep();

    // Go to conclusion spot
    bwi_scavenger::RobotMove msg;
    msg.type = 0;
    msg.location = FELLOW_COMPUTERS;
    pub_move.publish(msg);
  }
}

// Called when move finished
void move_finished_cb(const std_msgs::Bool::ConstPtr &msg) {
  if (!node_active)
    return;

  ROS_INFO("%s Conclusion protocol complete.", TELEM_TAG);

  std_msgs::Bool complete_msg;
  complete_msg.data = false;
  pub_task_complete.publish(complete_msg);
  node_active = false;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "conclude_node");
  ros::NodeHandle nh;
  ros::Subscriber sub0 = nh.subscribe(TPC_MAIN_NODE_TASK_START, 1, task_start_cb);
  ros::Subscriber sub1 = nh.subscribe(TPC_MOVE_NODE_FINISHED, 1, move_finished_cb);

  pub_move = nh.advertise<bwi_scavenger::RobotMove>(TPC_MOVE_NODE_GO, 1);
  pub_task_complete = nh.advertise<std_msgs::Bool>(TPC_TASK_COMPLETE, 1);

  // Wait for ROS services to spin up
  ros::Duration(5.0).sleep();

  ROS_INFO("%s Standing by.", TELEM_TAG);

  while (ros::ok()) {
    ros::spinOnce();

    if (!node_active)
      continue;
  }
}
