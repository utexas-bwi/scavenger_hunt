#ifndef MOVE_NODE_H
#define MOVE_NODE_H

#include <bwi_scavenger/robot_motion.h>
#include <bwi_scavenger/RobotMove.h>
#include <bwi_scavenger/RobotStop.h>
#include <std_msgs/Bool.h>

RobotMotion *rm;
ros::Publisher movePub;
std_msgs::Bool result;
int MOVE = 0;
int SPIN = 1;

/**
  Callback for stop topic. Will stop the robot if it is in motion.

  @param scavener_stop msg, placeholder to signal robot to stop
*/
void stop(const bwi_scavenger::RobotStop::ConstPtr &data);

/**
  Callback for robot movement. Robot will move towards or a location
  or will spin in place.

  @param scavenger_move msg, signifies movement towards a location
  or rotation by some degrees
 */
void move(const bwi_scavenger::RobotMove::ConstPtr &data);


/**
  Callback for map topic published by the robot.

  @param occupancy grid of the map stored in the robot
*/
void getMapId(const nav_msgs::OccupancyGrid::ConstPtr &grid);

#endif
