#ifndef KINECT_FUSION_H
#define KINECT_FUSION_H

#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <sensor_msgs/Image.h>
#include <utility>

namespace kinect_fusion {

/**
  Estimates the absolute distance between the viewer and the object located in
  some bounding box.

  @param  box        object bounding box produced by YOLO
  @param  depth_map  depth field produced by IR camera; WILL be mutated
  @return estimated  distance in units of depth field
*/
double estimate_distance(const darknet_ros_msgs::BoundingBox &box,
                         cv::Mat depth_map);

/**
  @brief identical to the above method but takes a ROS image
*/
double estimate_distance(const darknet_ros_msgs::BoundingBox &box,
                         const sensor_msgs::Image &img);

/**
  Gets the relative position of the object located in some bounding box.

  @param  box bounding box to analyze
  @param  img depth map captured by IR camera
  @return relative (x, y) of the object with the camera at the origin facing
          the positive y axis
*/
std::pair<double, double> get_2d_offset(
    const darknet_ros_msgs::BoundingBox &box,
    const sensor_msgs::Image &img);

}; // end namespace kinect_fusion

#endif
