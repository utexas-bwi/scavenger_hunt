#include <bwi_scavenger/kinect_fusion.h>
#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

sensor_msgs::Image depth;
bool depth_stale = true;
std::string target = "cup";

ros::Publisher pub_map_visualizer;

void save_depth(const sensor_msgs::Image::ConstPtr &msg) {
  depth = *msg;
  depth_stale = false;
}

void save_bounding_boxes(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg) {
  if (!depth_stale && depth.height > 0 && msg->bounding_boxes.size() > 0) {
    depth_stale = true;
    darknet_ros_msgs::BoundingBox box;
    for (int i = 0; i < msg->bounding_boxes.size(); i++)
      if (msg->bounding_boxes[i].Class == target) {
        box = msg->bounding_boxes[i];
        break;
      }

    if (box.Class == target) {
      // double distance = kinect_fusion::estimate_distance(box, depth);
      std::pair<double, double> offset = kinect_fusion::get_2d_offset(box, depth);
      pub_map_visualizer.publish(depth);
      ROS_INFO("%s is at relative position (%f, %f)",
          target.c_str(), offset.first, offset.second);
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "kinect_test");
  ros::NodeHandle nh;

  pub_map_visualizer = nh.advertise<sensor_msgs::Image>("kinect_test/map_visualizer", 1);

  ros::Subscriber sub0 = nh.subscribe("/darknet_ros/bounding_boxes/", 1,
      save_bounding_boxes);
  ros::Subscriber sub1 = nh.subscribe("/camera/depth/image", 1, save_depth);

  ros::spin();
  return 0;
}
