#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <utility>

namespace kinect_fusion {

/**
  Used by estimate_distance to project a bounding box from the color camera's
  FOV onto the IR camera's FOV.

  @param box bounding box to adjust
*/
void adjust_bounding_box(darknet_ros_msgs::BoundingBox &box);

/**
  A simple algorithm for estimating the proximity of an object within some
  bounding box.

  @param box object bounding box, probably produced by CV
  @param depth_map depth field produced by IR camera
  @return estimated distance in units of depth field
*/
double estimate_distance(darknet_ros_msgs::BoundingBox &box, cv::Mat &depth_map);

/**
  @brief identical to the above method but takes a ROS image
*/
double estimate_distance(darknet_ros_msgs::BoundingBox &box,
    sensor_msgs::Image &img);

/**
  Gets the relative position of the object located in a bounding box.

  @param box bounding box to analyze
  @param img depth map captured by IR camera
  @return relative (x, y) of the object with the camera at the origin facing
          the positive y axis
*/
std::pair<double, double> get_2d_offset(darknet_ros_msgs::BoundingBox &box,
    sensor_msgs::Image &img);

}; // end namespace kinect_fusion
