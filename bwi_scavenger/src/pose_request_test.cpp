#include <bwi_scavenger_msgs/PoseRequest.h>

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_request_test");
  ros::NodeHandle nh;

  ros::ServiceClient client =
      nh.serviceClient<bwi_scavenger_msgs::PoseRequest>("pose_request");

  bwi_scavenger_msgs::PoseRequest req;
  client.call(req);
  ROS_INFO("%f %f %f", req.response.pose.position.x,
                       req.response.pose.position.y,
                       req.response.pose.position.z);
}
