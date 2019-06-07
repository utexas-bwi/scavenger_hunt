#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

void explore(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &curPose){
  std::cout << "Coordinate Point: (" << curPose->pose.pose.position.x << ", " << curPose->pose.pose.position.y << ")" << std::endl;;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "coor_point");
  ros::NodeHandle node;

  ros::Subscriber locationSub = node.subscribe("/amcl_pose", 1, explore);

  ros::spin();

}
