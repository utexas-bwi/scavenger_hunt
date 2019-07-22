#include <ros/ros.h>

#include <bwi_scavenger/globals.h>
#include <bwi_scavenger/file_editor.h>

#include <bwi_scavenger_msgs/DatabaseFile.h>

#include <iostream>  
#include <fstream>  

int main(int argc, char** argv){
  ros::init(argc, argv, "file_send");
  ros::NodeHandle nh;

  ros::Publisher pub_send = nh.advertise<bwi_scavenger_msgs::DatabaseFile>(TPC_TRANSFER_NODE_SEND_FILE, 1);
  ros::Duration(2.0).sleep();

  std::string name = PROOF_DATABASE_PATH;
  std::ifstream fl(name);  
  fl.seekg(0, std::ios::end);  
  size_t len = fl.tellg();  
  char *ret = new char[len];  
  fl.seekg(0, std::ios::beg);   
  fl.read(ret, len);  
  fl.close();  

  bwi_scavenger_msgs::DatabaseFile msg;
  msg.name = PROOF_DATABASE_PATH;

  for(int i = 0; i < len; i++)
    msg.data.push_back(ret[i]);
  pub_send.publish(msg);

  ros::spin();
}