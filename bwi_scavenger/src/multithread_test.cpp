#include <ros/ros.h>
#include <bwi_scavenger/scavenger_move.h>
#include <bwi_scavenger/scavenger_stop.h>

int MOVE = 0;
int SPIN = 1;
int STOP = 2;

int main(int argc, char** argv){
  ros::init(argc, argv, "multithread_test");
  ros::NodeHandle node;

  ros::Publisher pub = node.advertise<bwi_scavenger::scavenger_move>("/scavenger/move_instruction", 100);
  ros::Publisher stopPub = node.advertise<bwi_scavenger::scavenger_stop>("/scavenger/stop", 100);
 
  ros::Duration(5).sleep();

  bwi_scavenger::scavenger_move msg;

  ROS_INFO("Sending message to move");
  msg.type = MOVE;
  msg.location = 0;
  pub.publish(msg);

  ros::Duration(10).sleep();

  ROS_INFO("Stopping robot");
  bwi_scavenger::scavenger_stop stopMsg;
  stopPub.publish(stopMsg);

}