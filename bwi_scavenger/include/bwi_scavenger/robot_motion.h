#ifndef BWI_SCAVENGER_ROBOT_MOTION_H
#define BWI_SCAVENGER_ROBOT_MOTION_H

#define SIMULATION // Use for simulation points or real world points

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>


typedef std::pair<float, float> coordinate;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

enum environment_location {
  BWI_LAB_DOOR_NORTH,
  BWI_LAB_DOOR_SOUTH,
  CLEARING_NORTH,
  CLEARING_SOUTH,
  ALCOVE,
  KITCHEN,
  SOCCER_LAB_DOOR_NORTH,
  SOCCER_LAB_DOOR_SOUTH,
  FELLOW_COMPUTERS
};

const int NUM_ENVIRONMENT_LOCATIONS = 9;

static std::map<environment_location, coordinate> environment_location_coordinates {
#ifndef SIMULATION
  {BWI_LAB_DOOR_NORTH,    coordinate( -39.3304, -11.2290 )},
  {BWI_LAB_DOOR_SOUTH,    coordinate( -38.2976, -4.3244  )},
  {CLEARING_NORTH,        coordinate( -13.9990, -12.024  )},
  {CLEARING_SOUTH,        coordinate( -13.7639, -5.5648  )},
  {ALCOVE,                coordinate( -18.2928, -4.9306  )},
  {KITCHEN,               coordinate( -30.1798, -4.5981  )},
  {SOCCER_LAB_DOOR_NORTH, coordinate( -47.7036, -11.2160 )},
  {SOCCER_LAB_DOOR_SOUTH, coordinate( -47.5810, -4.1472  )},
  {FELLOW_COMPUTERS,      coordinate( -35.0182, -11.4877 )}
#else
  {BWI_LAB_DOOR_NORTH,    coordinate( 39.6831, 112.569   )},
  {BWI_LAB_DOOR_SOUTH,    coordinate( 39.553,  105.192   )},
  {CLEARING_NORTH,        coordinate( 14.4326, 112.49    )},
  {CLEARING_SOUTH,        coordinate( 14.3866, 105.104   )},
  {ALCOVE,                coordinate( 19.6725, 105.155   )},
  {KITCHEN,               coordinate( 30.9585, 105.623   )},
  {SOCCER_LAB_DOOR_NORTH, coordinate( 48.2878, 112.392   )},
  {SOCCER_LAB_DOOR_SOUTH, coordinate( 48.3504, 105.226   )},
  {FELLOW_COMPUTERS,      coordinate( 35.8519, 112.632   )}
#endif
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

    @param location coordinate point for the robot to travel to
  */
  void move_to_location(coordinate location);

  /* 
    Turns the robot by the number of degrees specified
  */
  void turn(float degrees);
};

#endif
