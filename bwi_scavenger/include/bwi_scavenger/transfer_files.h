#include <bwi_scavenger/globals.h>

#include <bwi_scavenger_msgs/DatabaseFile.h>

#include <iostream>
#include <fstream>
#include <ros/ros.h>

static const std::string TAG_DNROS_WEIGHTS = "dnros_weights";
static const std::string TAG_DNROS_CFG = "dnros_cfg";
static const std::string TAG_DNROS_MODEL_YAML = "dnros_model_yaml";
static const std::string TAG_DNROS_ROS_YAML = "dnros_ros_yaml";
static const std::string TAG_DNROS_LAUNCH = "dnros_launch";

#define NUM_FILES 5
