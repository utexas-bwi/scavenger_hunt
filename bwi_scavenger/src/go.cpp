#include <bwi_scavenger/globals.h>
#include <bwi_scavenger_msgs/RobotMove.h>
#include <bwi_scavenger_msgs/RobotStop.h>
#include <bwi_scavenger/robot_motion.h>
#include <ros/ros.h>
#include <string.h>

static ros::Publisher pub_move;

int main(int argc, char **argv) {
  if (argc < 2) {
    ROS_ERROR("Usage: go <location>");
    exit(0);
  }

  environment_location l;
  std::string arg(argv[1]);

  if (arg == "glass")
    l = FELLOW_COMPUTERS;
  else if (arg == "bay0")
    l = BWI_LAB_DOOR_NORTH;
  else if (arg == "bay1")
    l = BWI_LAB_DOOR_SOUTH;

  ros::init(argc, argv, "go");
  ros::NodeHandle nh;

  pub_move = nh.advertise<bwi_scavenger_msgs::RobotMove>(TPC_MOVE_NODE_GO, 1);

  ros::Duration(3.0).sleep();

  bwi_scavenger_msgs::RobotMove msg;
  msg.type = 0;
  coordinate c = environment_location_coordinates[l];
  msg.location.push_back(c.first);
  msg.location.push_back(c.second);
  pub_move.publish(msg);

  ros::spinOnce();
}
