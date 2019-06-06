#ifndef CV_SWEEP_H
#define CV_SWEEP_H

#include <actionlib/client/simple_action_client.h>
#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

typedef darknet_ros_msgs::CheckForObjectsAction CvSweepAction;
typedef darknet_ros_msgs::CheckForObjectsGoal CvSweepGoal;
typedef actionlib::SimpleActionClient<CvSweepAction> CvSweepActionClient;
typedef std::shared_ptr<CvSweepActionClient> CvSweepActionClientPtr;

/**
  Callback for a completed CvSweepAction.

  @param state client state
  @param result result of object detection
  @param dest destination for results
*/
void cv_sweep_cb(const actionlib::SimpleClientGoalState& state,
    const darknet_ros_msgs::CheckForObjectsResultConstPtr& result);

/**
  Sweeps a sensor_msgs::Image for objects with YOLO.

  @param nh ROS node handle
  @param image image to sweep
  @param dest destination for sweep results
  @return whether or not the sweep succeeded
*/
bool cv_sweep(ros::NodeHandle &nh, const sensor_msgs::Image &image,
    darknet_ros_msgs::BoundingBoxes *dest);

/**
  Sweeps an image file for objects with YOLO.

  @param nh ROS node handle
  @param image_path relative path to image
  @param dest destination for sweep results
  @return whether or not the sweep succeeded
*/
bool cv_sweep_local(ros::NodeHandle &nh, const std::string &image_path,
    darknet_ros_msgs::BoundingBoxes *dest);

#endif
