#ifndef BWI_SCAVENGER_ROBOT_MOTION_H
#define BWI_SCAVENGER_ROBOT_MOTION_H

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

enum environment_location {
  BWI_LAB_RIGHT,
  CLEARING_RIGHT,
  // WHITEBOARD_ROOM,
  CLEARING_LEFT,
  ALCOVE,
  KITCHEN,
  BWI_LAB_LEFT,
  LARG_RIGHT,
  LARG_LEFT
};

const int NUM_ENVIRONMENT_LOCATIONS = 8;

static std::map<environment_location, std::pair<float, float>>
    environment_location_coordinates {
  // {BWI_LAB_RIGHT, std::pair<float, float>(14.956, 109.94)},
  {BWI_LAB_RIGHT, std::pair<float, float>(-39.3304, -11.2288)},
  {BWI_LAB_LEFT, std::pair<float, float>(-38.2976, -4.32444)},
  // {WHITEBOARD_ROOM, std::pair<float, float>(-8.98598, -12.0676)},
  {CLEARING_LEFT, std::pair<float, float>(-13.7639, -5.56475)},
  {CLEARING_RIGHT, std::pair<float, float>(-13.999, -12.024)},
  {ALCOVE, std::pair<float, float>(-18.2928, -4.93057)},
  {KITCHEN, std::pair<float, float>(-30.1798, -4.59807)},
  {LARG_RIGHT, std::pair<float, float>(-47.581, -4.14724)},
  {LARG_LEFT, std::pair<float, float>(-47.7036, -11.216)}
};

static std::string grid_frame_id;
static bool received_grid_frame_id = false;
static tf::TransformListener *tfl = NULL;

class RobotMotion {
protected:
  std::string grid_frame_id;
  MoveBaseClient *ac;
  tf::TransformListener *tfl;

public:
  RobotMotion(std::string grid_frame_id, tf::TransformListener &tfl);

  void end_movement();

  void move_to_location(environment_location location);

  void turn(float degrees);
};

#endif
