#ifndef MOVE_NODE_H
#define MOVE_NODE_H

#include <bwi_scavenger/robot_motion.h>
#include <bwi_scavenger_msgs/RobotMove.h>
#include <bwi_scavenger_msgs/RobotStop.h>
#include <std_msgs/Bool.h>

#include <bwi_scavenger/globals.h>

#include <bwi_scavenger_msgs/PoseRequest.h>

#include <geometry_msgs/Pose.h>
#include <math.h>
#include <limits.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

RobotMotion *rm;
ros::Publisher movePub;
std_msgs::Bool result;
int MOVE = 0;
int SPIN = 1;

tf::TransformListener *listener;
std::string gridFrameId;


/**
  Callback for stop topic. Will stop the robot if it is in motion.

  @param scavener_stop msg, placeholder to signal robot to stop
*/
void stop(const bwi_scavenger_msgs::RobotStop::ConstPtr &data);

/**
  Callback for robot movement. Robot will move towards or a location
  or will spin in place.

  @param scavenger_move msg, signifies movement towards a location
  or rotation by some degrees
 */
void move(const bwi_scavenger_msgs::RobotMove::ConstPtr &data);


/**
  Callback for map topic published by the robot.

  @param occupancy grid of the map stored in the robot
*/
void getMapId(const nav_msgs::OccupancyGrid::ConstPtr &grid);

#endif
