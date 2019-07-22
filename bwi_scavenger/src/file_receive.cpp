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

void receive_cb(const bwi_scavenger_msgs::DatabaseFile msg){
  std::string destination;

  if (msg.tag == TAG_DNROS_WEIGHTS)
    destination = DNROS_PATH + "/yolo_network_config/weights";
  else if (msg.tag == TAG_DNROS_CFG)
    destination = DNROS_PATH + "/yolo_network_config/cfg";
  else if (msg.tag == TAG_DNROS_MODEL_YAML)
    destination = DNROS_PATH + "/config";
  else if (msg.tag == TAG_DNROS_ROS_YAML)
    destination = DNROS_PATH + "/config";
  else if (msg.tag == TAG_DNROS_LAUNCH)
    destination = DNROS_PATH + "/launch";
  else {
    ROS_ERROR("Unknown file tag: %s", msg.tag);
    return;
  }

  destination += "/" + msg.name;

  size_t len = msg.data.size();
  std::ofstream file(destination, std::ios::binary);
  char array[len];

  for(int i = 0; i < len; i++)
    array[i] = msg.data[i];

  file.write(array, len);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "file_receive");
  ros::NodeHandle nh;

  ros::Subscriber sub_receive = nh.subscribe(TPC_TRANSFER_NODE_SEND_FILE, 1, receive_cb);

  ros::spin();
}
