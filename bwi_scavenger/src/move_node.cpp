// #define VERBOSE
#include <bwi_scavenger/move_node.h>

void stop(const bwi_scavenger_msgs::RobotStop::ConstPtr &data){
    rm->end_movement();
}

void move(const bwi_scavenger_msgs::RobotMove::ConstPtr &data){
  if(data -> type == MOVE){
    coordinate goal_coord = {data->location[0], data->location[1]};
  #ifdef VERBOSE
    ROS_INFO("[move_node] Moving to location (%f, %f).", goal_coord.first, goal_coord.second);
  #endif
    rm -> move_to_location(goal_coord);
    movePub.publish(result);
  } else if (data -> type == SPIN){
    rm -> turn (data->degrees);
    movePub.publish(result);
  }
}

void getMapId(const nav_msgs::OccupancyGrid::ConstPtr &grid) {
  listener = new tf::TransformListener();
  gridFrameId = grid->header.frame_id;
  rm = new RobotMotion(gridFrameId, *listener);
}

bool serveRobotPose(bwi_scavenger_msgs::PoseRequest::Request &req,
                    bwi_scavenger_msgs::PoseRequest::Response &res) {
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

  res.pose = pose;

  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "move_node");
  ros::NodeHandle moveNode;

  ros::Subscriber mapSub = moveNode.subscribe("/level_mux/map", 1, getMapId);

  ros::Subscriber findObjectSub = moveNode.subscribe(TPC_MOVE_NODE_GO, 1, move);
  ros::Subscriber stopMoveSub = moveNode.subscribe(TPC_MOVE_NODE_STOP, 1, stop);

  movePub = moveNode.advertise<std_msgs::Bool>(TPC_MOVE_NODE_FINISHED, 1);

  ros::ServiceServer srv_pose_request =
      moveNode.advertiseService(SRV_POSE_REQUEST, serveRobotPose);

  ROS_INFO("[move_node] Standing by.");

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
}
