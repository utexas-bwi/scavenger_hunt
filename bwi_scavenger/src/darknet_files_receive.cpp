#include "bwi_scavenger/paths.h"
#include "bwi_scavenger/transfer_files.h"

void receive_cb(const bwi_scavenger_msgs::DatabaseFile msg){
  std::string destination;

  if (msg.tag == TAG_DNROS_WEIGHTS)
    destination = paths::dnros() + "/yolo_network_config/weights";
  else if (msg.tag == TAG_DNROS_CFG)
    destination = paths::dnros() + "/yolo_network_config/cfg";
  else if (msg.tag == TAG_DNROS_MODEL_YAML)
    destination = paths::dnros() + "/config";
  else if (msg.tag == TAG_DNROS_ROS_YAML)
    destination = paths::dnros() + "/config";
  else if (msg.tag == TAG_DNROS_LAUNCH)
    destination = paths::dnros() + "/launch";
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
