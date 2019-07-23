#include <bwi_scavenger/transfer_files.h>

ros::Publisher pub_send_proofs_file;

void send_file(int tag){
  std::string source;
  std::string name;
  std::string type = "Find_Object";

  bwi_scavenger_msgs::DatabaseFile msg;

  if (tag == 0){
    msg.tag = TAG_DNROS_WEIGHTS;
    source = DNROS_PATH + "/yolo_network_config/weights";
    name = type + ".weights";
  }
  else if (tag == 1){
    msg.tag = TAG_DNROS_CFG;
    source = DNROS_PATH + "/yolo_network_config/cfg";
    name = type + ".cfg";
  }
  else if (tag == 2){
    msg.tag = TAG_DNROS_MODEL_YAML;
    source = DNROS_PATH + "/config";
    name = type + ".yaml";
  }
  else if (tag == 3){
    msg.tag = TAG_DNROS_ROS_YAML;
    source = DNROS_PATH + "/config";
    name = type + "-ros.yaml";
  }
  else if (tag == 4){
    msg.tag = TAG_DNROS_LAUNCH;
    source = DNROS_PATH + "/launch";
    name = "darknet_ros_" + type + ".launch";
  }
  else {
    ROS_ERROR("Can't send file: undetermined tag");
    return;
  }

  msg.name = name;

  // copy file into byte array
  std::string filepath = source + name;
  std::ifstream fl(filepath);  
  fl.seekg(0, std::ios::end);  
  size_t len = fl.tellg();  
  char *ret = new char[len];  
  fl.seekg(0, std::ios::beg);   
  fl.read(ret, len);  
  fl.close();  

  for(int i = 0; i < len; i++)
    msg.data.push_back(ret[i]);

  pub_send_proofs_file.publish(msg);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "file_send");
  ros::NodeHandle nh;

  pub_send_proofs_file = nh.advertise<bwi_scavenger_msgs::DatabaseFile>(TPC_TRANSFER_NODE_SEND_FILE, 1);
  ros::Duration(2.0).sleep();

  for(int i = 0; i < NUM_FILES; i++)
    send_file(i);
  
  ros::spin();
}