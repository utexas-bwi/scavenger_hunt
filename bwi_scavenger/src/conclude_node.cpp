#include <bwi_scavenger_msgs/RobotMove.h>
#include <bwi_scavenger_msgs/RobotStop.h>
#include <bwi_scavenger_msgs/TaskEnd.h>

#include <scavenger_hunt_msgs/Task.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "bwi_scavenger/globals.h"
#include "bwi_scavenger/world_mapping.h"

static const char TELEM_TAG[] = "[conclude_node]";
static bool node_active = false;

static ros::Publisher pub_move;
static ros::Publisher pub_task_complete;

// Called when the main node is starting a task
void task_start_cb(const scavenger_hunt_msgs::Task& msg) {
  if (msg.name == TASK_CONCLUDE) {
    ROS_INFO("%s Conclusion protocol firing!", TELEM_TAG);
    ros::Duration(1.0).sleep();

    node_active = true;
    bwi_scavenger_msgs::RobotMove msg;
    msg.type = bwi_scavenger_msgs::RobotMove::WAYPOINT;
    msg.waypoint = FELLOW_COMPUTERS;
    pub_move.publish(msg);
  }
}

// Called when move finished
void move_finished_cb(const std_msgs::Bool::ConstPtr &msg) {
  if (!node_active)
    return;

  ROS_INFO("%s Conclusion protocol complete.", TELEM_TAG);

  bwi_scavenger_msgs::TaskEnd end_msg;
  end_msg.success = false;
  pub_task_complete.publish(end_msg);
  node_active = false;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "conclude_node");
  ros::NodeHandle nh;
  ros::Subscriber sub0 = nh.subscribe(TPC_TASK_START, 1, task_start_cb);
  ros::Subscriber sub1 = nh.subscribe(TPC_MOVE_NODE_FINISHED, 1, move_finished_cb);

  pub_move = nh.advertise<bwi_scavenger_msgs::RobotMove>(TPC_MOVE_NODE_GO, 1);
  pub_task_complete = nh.advertise<bwi_scavenger_msgs::TaskEnd>(TPC_TASK_END, 1);

  // Wait for ROS services to spin up
  ros::Duration(5.0).sleep();

  ROS_INFO("%s Standing by.", TELEM_TAG);

  while (ros::ok()) {
    ros::spinOnce();

    if (!node_active)
      continue;
  }
}
