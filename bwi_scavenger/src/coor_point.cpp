#include <ros/ros.h>
#include <geometry_msgs>

void explore(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &curPose){
  sdt::cout << "Coordinate Point: (" << curPose->pose->pose->position->x << ", " << curPose.pose.pose.position.x << ")" << std::endl;;
}

int main(int arc, char **argv){

  ros::init(argc, argv, "coor_point");

  ros::Subscriber locationSub = node.subscribe("/amcl_pose", 1, explore);

}
