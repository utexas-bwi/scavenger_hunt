#include <bwi_scavenger/robot_motion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>

#define PI 3.14159265359

RobotMotion::RobotMotion(std::string grid_frame_id, tf::TransformListener &tfl) {
  this->grid_frame_id = grid_frame_id;
  ac = new MoveBaseClient("move_base", true);
  this->tfl = &tfl;
  while(!ac->waitForServer(ros::Duration(5.0)))
    ROS_INFO("[RobotMotion] Waiting for MoveBaseClient server to open...");
  ROS_INFO("[RobotMotion] Connected to MoveBaseClient.");
}

void RobotMotion::end_movement(){
  move_base_msgs::MoveBaseGoal goal;
  ac->cancelGoal();
}

void RobotMotion::move_to_location(environment_location location) {
  std::pair<float, float> coordinates =
      environment_location_coordinates[location];
  double start = ros::Time::now().toSec();

  ROS_INFO("[RobotMotion] Moving to location %d, coordinates (%f, %f)",
      (int)location, coordinates.first, coordinates.second);

  move_base_msgs::MoveBaseGoal goal;
  geometry_msgs::PoseStamped goal_pose;

  goal_pose.header.seq = 0;
  goal_pose.header.stamp = ros::Time(0);
  goal_pose.header.frame_id = grid_frame_id;

  goal_pose.pose.position.x = coordinates.first;
  goal_pose.pose.position.y = coordinates.second;
  goal_pose.pose.orientation.w = 1;

  geometry_msgs::PoseStamped tag_rel_pose;
  tfl->waitForTransform("base_link", goal_pose.header.frame_id, ros::Time::now(),
      ros::Duration(4));
  tfl->transformPose("base_link", goal_pose, tag_rel_pose);

  tag_rel_pose.pose.position.z = 0;
  goal.target_pose = tag_rel_pose;

  ac->sendGoal(goal);
  // Keep moving until goal is completed or stopped
  while(ac->getState() != actionlib::SimpleClientGoalState::SUCCEEDED &&
        ac->getState() != actionlib::SimpleClientGoalState::PREEMPTED);

  double end = ros::Time::now().toSec();
  ROS_INFO("[RobotMotion] Arrived at location after %f seconds", end - start);
}

void RobotMotion::turn(float degrees) {
  double start = ros::Time::now().toSec();
  float radians = degrees * (PI / 180);

  ROS_INFO("[RobotMotion] Executing relative turn of %f degrees (%f radians)", degrees, radians);

  tf::Quaternion quat;
  quat.setRPY(0, 0, radians);
  quat.normalize();

  geometry_msgs::Quaternion geom_quat;
  tf::quaternionTFToMsg(quat, geom_quat);

  move_base_msgs::MoveBaseGoal goal;
  geometry_msgs::PoseStamped goal_pose;

  goal_pose.header.seq = 0;
  goal_pose.header.stamp = ros::Time(0);
  goal_pose.header.frame_id = "base_link";

  goal_pose.pose.position.x = goal_pose.pose.position.y = goal_pose.pose.position.z = 0;
  goal_pose.pose.orientation = geom_quat;
  goal.target_pose = goal_pose;

  ac->sendGoal(goal);
  while(ac->getState() != actionlib::SimpleClientGoalState::SUCCEEDED &&
        ac->getState() != actionlib::SimpleClientGoalState::PREEMPTED);

  double end = ros::Time::now().toSec();
  ROS_INFO("[RobotMotion] Finished turn after %f seconds", end - start);
}
