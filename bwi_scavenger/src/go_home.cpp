#include <bwi_scavenger_msgs/RobotMove.h>
#include <bwi_scavenger_msgs/RobotStop.h>
#include <bwi_scavenger_msgs/TaskEnd.h>

#include <scavenger_hunt_msgs/Task.h>
#include <bwi_scavenger_msgs/RobotMove.h>
#include <bwi_scavenger_msgs/RobotStop.h>
#include <bwi_scavenger_msgs/PoseRequest.h>

#include <std_msgs/Bool.h>

#include <ros/ros.h>
#include <tf/tf.h>

#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>

#include "bwi_scavenger/globals.h"
#include "bwi_scavenger/robot_motion.h"
#include "bwi_scavenger/world_mapping.h"

static const char TELEM_TAG[] = "[conclude_node]";
static bool node_active = false;
static RobotMotion *rm;
static ros::Publisher pub_move;
static tf::TransformListener *listener;
static std::string grid_frame_id;
static std::map<EnvironmentLocation, coordinates_t>* world_waypoints;

void get_grid_frame_id(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  listener = new tf::TransformListener();
  grid_frame_id = msg->header.frame_id;
  rm = new RobotMotion(grid_frame_id, *listener);

  ros::Duration(3.0).sleep();

  coordinates_t goal_coords = {-35.995, -11.678};
  rm->move_to_location(goal_coords);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "go_home");
  ros::NodeHandle nh;

  ros::Subscriber sub_map =
    nh.subscribe("/level_mux/map", 1, get_grid_frame_id);

  ros::spin();

}
