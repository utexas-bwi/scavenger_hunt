#include <math.h>
#include <ros/ros.h>

#include "kinect_fusion/kinect_fusion.h"

static sensor_msgs::Image depth;
static bool depth_stale = true;
static std::string target = "sports ball"; // The YOLO label we're looking for

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
void process(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg) {
  if (!depth_stale && depth.height > 0 && msg->bounding_boxes.size() > 0) {
    // Mark the depth map as stale to prevent multiple estimations on the same
    // map
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
      geometry_msgs::Point offset = kinect_fusion::get_position(box, depth);

      float camera_x = 0; // 0.29;
      float camera_y = 0; // 0.39;
      float camera_theta = 0; // -0.96586521;

      float obj_x = camera_x + cos(camera_theta) * offset.x - sin(camera_theta) * offset.y;
      float obj_y = camera_y + sin(camera_theta) * offset.x + cos(camera_theta) * offset.y;
      float obj_z = 0;

      if (obj_x != camera_x) {
        ROS_INFO("%s is at relative position (%f, %f, %f)",
                 target.c_str(),
                 obj_x,
                 obj_y,
                 obj_z);
      }
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "kinect_fusion_test_node");

  ros::NodeHandle nh;
  ros::Subscriber sub_boxes = nh.subscribe("/darknet_ros/bounding_boxes/",
                                           1,
                                           process);
  ros::Subscriber sub_depth = nh.subscribe("/camera/depth/image",
                                           1,
                                           save_depth);

  ros::spin();

  return 0;
}
