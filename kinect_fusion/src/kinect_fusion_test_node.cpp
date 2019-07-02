#include <ros/ros.h>

#include "kinect_fusion/kinect_fusion.h"

static sensor_msgs::Image depth;
static bool depth_stale = true;
static std::string target = "person"; // The YOLO label we're looking for

/**
  @brief buffers a depth map to be used on the next YOLO broadcast
*/
void save_depth(const sensor_msgs::Image::ConstPtr &msg) {
  depth = *msg;
  depth_stale = false;
}

/**
  @brief YOLO has produced bounding boxes; see if one contains our target
*/
void save_bounding_boxes(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg) {
  if (!depth_stale && depth.height > 0 && msg->bounding_boxes.size() > 0) {
    // Mark the depth map as stale to prevent multiple estimations on the same
    // map, which may be mutated for visualization purposes
    depth_stale = true;
    darknet_ros_msgs::BoundingBox box;

    // Look for a box with our target label
    for (int i = 0; i < msg->bounding_boxes.size(); i++)
      if (msg->bounding_boxes[i].Class == target) {
        box = msg->bounding_boxes[i];
        break;
      }

    if (box.Class == target) {
      // Target spotted!
      std::pair<double, double> offset = kinect_fusion::get_2d_offset(box, depth);
      ROS_INFO("%s is at relative position (%f, %f)",
               target.c_str(),
               offset.first,
               offset.second);
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "kinect_fusion_test_node");

  ros::NodeHandle nh;
  ros::Subscriber sub_boxes = nh.subscribe("/darknet_ros/bounding_boxes/",
                                           1,
                                           save_bounding_boxes);
  ros::Subscriber sub_depth = nh.subscribe("/camera/depth/image",
                                           1,
                                           save_depth);

  ros::spin();

  return 0;
}
