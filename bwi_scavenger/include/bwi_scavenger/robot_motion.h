#ifndef BWI_SCAVENGER_ROBOT_MOTION_H
#define BWI_SCAVENGER_ROBOT_MOTION_H

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "bwi_scavenger/world_mapping.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RobotMotion final {
private:
  std::string grid_frame_id;
  MoveBaseClient *ac;
  tf::TransformListener *tfl;

public:

  /*
    Creates a RobotMotion object to be used to move the robot.

    @param grid_frame_id frame id of the map the robot is using
    @param tfl copy of TransformListener that was initialized on startup of the calling node
  */
  RobotMotion(std::string grid_frame_id, tf::TransformListener &tfl);

  /*
    Stops the robot
  */
  void end_movement();

  /*
    Moves the robot to the given location

    @param location point to travel to
  */
  void move_to_location(coordinates_t location);

  /*
    Turns the robot by the number of degrees specified
  */
  void turn(float degrees);
};

#endif
