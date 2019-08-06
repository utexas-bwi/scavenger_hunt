#include <bwi_scavenger_msgs/RobotMove.h>
#include <bwi_scavenger_msgs/RobotStop.h>
#include <bwi_scavenger_msgs/PoseRequest.h>

#include <std_msgs/Bool.h>

#include <ros/ros.h>
#include <tf/tf.h>

#include <iostream>

#include "bwi_scavenger/globals.h"
#include "bwi_scavenger/robot_motion.h"
#include "bwi_scavenger/world_mapping.h"

static RobotMotion *rm;
static ros::Publisher pub_move;
static tf::TransformListener *listener;
static std::string grid_frame_id;
static std::map<EnvironmentLocation, coordinates_t>* world_waypoints;

void stop(const bwi_scavenger_msgs::RobotStop::ConstPtr &msg) {
  rm->end_movement();
}

void move(const bwi_scavenger_msgs::RobotMove::ConstPtr &msg) {
  std_msgs::Bool result;
  result.data = true;

  if (msg->type == bwi_scavenger_msgs::RobotMove::MOVE) {
    coordinates_t goal_coords = {msg->location[0], msg->location[1]};
    rm->move_to_location(goal_coords);
    pub_move.publish(result);
  } else if (msg->type == bwi_scavenger_msgs::RobotMove::TURN) {
    rm->turn(msg->degrees);
    pub_move.publish(result);
  } else if (msg->type == bwi_scavenger_msgs::RobotMove::WAYPOINT) {
    EnvironmentLocation loc = static_cast<EnvironmentLocation>(msg->waypoint);
    coordinates_t goal_coords = (*world_waypoints)[loc];
    rm->move_to_location(goal_coords);
    pub_move.publish(result);
  }
}

void get_grid_frame_id(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  listener = new tf::TransformListener();
  grid_frame_id = msg->header.frame_id;
  rm = new RobotMotion(grid_frame_id, *listener);
}

bool serve_robot_pose(bwi_scavenger_msgs::PoseRequest::Request &req,
                      bwi_scavenger_msgs::PoseRequest::Response &res) {
  tf::StampedTransform robot_transform;
  listener->waitForTransform(
    grid_frame_id,
    "base_link",
    ros::Time::now(),
    ros::Duration(4)
  );
  listener->lookupTransform(
    grid_frame_id,
    "base_link",
    ros::Time(0),
    robot_transform
  );

  geometry_msgs::Point pose_position;
  pose_position.x = robot_transform.getOrigin().x();
  pose_position.y = robot_transform.getOrigin().y();
  pose_position.z = 0;

  geometry_msgs::Quaternion pose_orientation;
  pose_orientation.w = robot_transform.getRotation().w();
  pose_orientation.x = robot_transform.getRotation().x();
  pose_orientation.y = robot_transform.getRotation().y();
  pose_orientation.z = robot_transform.getRotation().z();

  geometry_msgs::Pose pose;
  pose.position = pose_position;
  pose.orientation = pose_orientation;
  res.pose = pose;

  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "move_node");
  ros::NodeHandle nh;

  std::string world;
  nh.param("bwi_scavenger/world", world, std::string("WORLDNOTFOUND"));

  if (world == "irl")
    world_waypoints = &WORLD_WAYPOINTS_IRL;
  else if (world == "sim")
    world_waypoints = &WORLD_WAYPOINTS_SIM;
  else
    ROS_ERROR("Unknown world: %s", world.c_str());

  ROS_INFO("Established world: %s", world.c_str());

  ros::Subscriber sub_map =
    nh.subscribe("/level_mux/map", 1, get_grid_frame_id);
  ros::Subscriber sub_go = nh.subscribe(TPC_MOVE_NODE_GO, 1, move);
  ros::Subscriber sub_stop = nh.subscribe(TPC_MOVE_NODE_STOP, 1, stop);

  pub_move = nh.advertise<std_msgs::Bool>(TPC_MOVE_NODE_FINISHED, 1);

  ros::ServiceServer srv_pose_request =
    nh.advertiseService(SRV_POSE_REQUEST, serve_robot_pose);

  ROS_INFO("[move_node] Standing by.");

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
}
