#include <ros/ros.h>

#include <bwi_scavenger/globals.h>

#include <bwi_scavenger_msgs/DatabaseFile.h>

#include <iostream>
#include <fstream>

static const std::string WS_PATH = "/home/jsuriadinata/scavenger_hunt";
static const std::string DNROS_PATH = WS_PATH + "/src/darknet_ros/darknet_ros";

static const std::string TAG_DNROS_WEIGHTS = "dnros_weights";
static const std::string TAG_DNROS_CFG = "dnros_cfg";
static const std::string TAG_DNROS_MODEL_YAML = "dnros_model_yaml";
static const std::string TAG_DNROS_ROS_YAML = "dnros_ros_yaml";
static const std::string TAG_DNROS_LAUNCH = "dnros_launch";

#define NUM_FILES 5