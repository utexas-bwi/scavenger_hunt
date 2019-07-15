#ifndef KINECT_FUSION_H
#define KINECT_FUSION_H

#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <utility>

namespace kinect_fusion {

/**
  The minimum slope that can be called "extreme" when searching a depth vector
  for extrema.
*/
extern float extrema_search_minimum_magnitude;
/**
  The inner percentage of data that get averaged for the final estimate
  following extrema removal.
*/
extern float interquartile_trim_width;
/**
  Number of partitions in the extrema search. Optimal value will vary with
  bounding box size.
*/
extern unsigned int extrema_search_partitions;
/**
  Iteration depth of the extrema search.
*/
extern unsigned int extrema_search_thoroughness;
/**
  Layers of erode/dilate applied after extrema filtering.
*/
extern unsigned int erode_dilate_strength;
/**
  The minimum bounding box area in pixels for which the primary estimator is
  used. If a bounding box's area falls under this threshold, a simpler estimator
  is used.
*/
extern unsigned int simplicity_threshold;

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
  The camera lies on the origin, facing 0 radians in the xy plane. The z plane
  is parallel with the camera lens.

  @param  box bounding box to analyze
  @param  img depth map captured by IR camera
  @return relative (x, y, z) of the object
*/
geometry_msgs::Point get_position(const darknet_ros_msgs::BoundingBox &box,
                                  const sensor_msgs::Image &img);

}; // end namespace kinect_fusion

#endif
