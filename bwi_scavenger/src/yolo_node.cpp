#include <ros/ros.h>
#include <bwi_scavenger/robot_motion.h>
#include <bwi_scavenger/TaskProof.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <bwi_scavenger/global_topics.h>

std::string objectToFind = "chair";
ros::Publisher proofPub;
ros::Publisher findPub;
sensor_msgs::Image image;

// goes through the objects in view and checks if the object has been found
void objectsCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &objects){
  for(int i = 0; i < objects -> bounding_boxes.size(); i++){
    if (objects -> bounding_boxes[i].Class == objectToFind){
      findPub.publish(objects -> bounding_boxes[i]);
    }
  }
}

void getTargetCb(const std_msgs::String::ConstPtr &msg){
  objectToFind = msg->data;
  ROS_INFO("[yolo_node] Updated target object to %s", objectToFind.c_str());
}

int main(int argc, char **argv){
  ros::init(argc, argv, "yolo_node");
  ros::NodeHandle yoloNode;

  ros::Subscriber boundingBoxSub = yoloNode.subscribe("/darknet_ros/bounding_boxes/", 1, objectsCb);
  ros::Subscriber targetSub = yoloNode.subscribe(TPC_YOLO_NODE_TARGET, 1, getTargetCb);

  findPub = yoloNode.advertise<darknet_ros_msgs::BoundingBox>(TPC_YOLO_NODE_TARGET_SEEN, 1);

  ROS_INFO("[yolo_node] Standing by.");

  ros::spin();
}
