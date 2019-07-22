#include <ros/ros.h>

#include <bwi_scavenger/globals.h>

#include <bwi_scavenger_msgs/DatabaseFile.h>

#include <iostream>
#include <fstream>

void receive_cb(const bwi_scavenger_msgs::DatabaseFile msg){
  std::string name = msg.name;
  size_t len = msg.data.size();
  std::ofstream file(name, std::ios::binary);
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