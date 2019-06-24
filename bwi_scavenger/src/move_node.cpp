#include <bwi_scavenger/global_topics.h>
#include <bwi_scavenger/move_node.h>

#include <geometry_msgs/Pose.h>
#include <math.h>
#include <limits.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

tf::TransformListener *listener;
std::string gridFrameId;

ros::Publisher pub_robot_pose;

void stop(const bwi_scavenger::RobotStop::ConstPtr &data){
    ROS_INFO("[move_node] Cancel goal");
    rm -> end_movement();
    // movePub.publish(result);
}

void move(const bwi_scavenger::RobotMove::ConstPtr &data){
  if(data -> type == MOVE){
    environment_location goal = static_cast<environment_location>(data->location % NUM_ENVIRONMENT_LOCATIONS);
    ROS_INFO("[move_node] Moving to location %d.", (int)goal);
    rm -> move_to_location(goal);
    movePub.publish(result);
  } else if (data -> type == SPIN){
    rm -> turn (data->degrees);
    movePub.publish(result);
  }
}

void getMapId(const nav_msgs::OccupancyGrid::ConstPtr &grid){
  ROS_INFO("Creating RobotMotion");
  listener = new tf::TransformListener();
  gridFrameId = grid->header.frame_id;
  rm = new RobotMotion(gridFrameId, *listener);
}

void broadcastRobotPose(const std_msgs::Bool::ConstPtr &msg) {
  tf::StampedTransform robotTransform;
  listener->waitForTransform(gridFrameId, "base_link", ros::Time::now(), ros::Duration(4));
  listener->lookupTransform(gridFrameId, "base_link", ros::Time(0), robotTransform);

  geometry_msgs::Point pose_position;
  pose_position.x = robotTransform.getOrigin().x();
  pose_position.y = robotTransform.getOrigin().y();
  pose_position.z = 0;

  geometry_msgs::Quaternion pose_orientation;
  pose_orientation.w = robotTransform.getRotation().w();
  pose_orientation.x = robotTransform.getRotation().x();
  pose_orientation.y = robotTransform.getRotation().y();
  pose_orientation.z = robotTransform.getRotation().z();

  geometry_msgs::Pose pose;
  pose.position = pose_position;
  pose.orientation = pose_orientation;

  pub_robot_pose.publish(pose);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "move_node");
  ros::NodeHandle moveNode;

  ros::Subscriber mapSub = moveNode.subscribe("/level_mux/map", 1, getMapId);

  ros::Subscriber findObjectSub = moveNode.subscribe(TPC_MOVE_NODE_GO, 1, move);
  ros::Subscriber stopMoveSub = moveNode.subscribe(TPC_MOVE_NODE_STOP, 1, stop);
  ros::Subscriber requestPoseSub = moveNode.subscribe(TPC_MOVE_NODE_REQUEST_POSE, 1, broadcastRobotPose);

  movePub = moveNode.advertise<std_msgs::Bool>(TPC_MOVE_NODE_FINISHED, 1);
  pub_robot_pose = moveNode.advertise<geometry_msgs::Pose>(TPC_MOVE_NODE_ROBOT_POSE, 1);

  ROS_INFO("[move_node] Standing by.");

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
}
