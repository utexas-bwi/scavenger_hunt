#include <bwi_scavenger/global_topics.h>
#include <bwi_scavenger/RobotMove.h>
#include <bwi_scavenger/RobotStop.h>
#include <bwi_scavenger/robot_motion.h>
#include <ros/ros.h>

static ros::Publisher pub_move;

int main(int argc, char **argv) {
  ros::init(argc, argv, "go");
  ros::NodeHandle nh;

  pub_move = nh.advertise<bwi_scavenger::RobotMove>(TPC_MOVE_NODE_GO, 1);

  ros::Duration(3.0).sleep();

  bwi_scavenger::RobotMove msg;
  msg.type = 0;
  msg.location = FELLOW_COMPUTERS;
  pub_move.publish(msg);

  ros::spin();
  return 0;
}
