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
bool objectFound = false;
bool saved = false;
sensor_msgs::Image image;
std_msgs::Bool found;

// goes through the objects in view and checks if the object has been found
void objectsCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &objects){
  if(!objectFound){
    for(int i = 0; i < objects -> bounding_boxes.size(); i++){
      if (objects -> bounding_boxes[i].Class == objectToFind){
        objectFound = true;
        findPub.publish(found);
      }
    }
  }
}

// saves the image that YOLO produces if the object has been found in that image
void imageCb(const sensor_msgs::Image::ConstPtr &img){
  if(objectFound && !saved){
    ROS_INFO("[yolo_node] Found object, publish to main node.");
    // Give image to the main node 
    proofPub.publish(*img);
    saved = true;
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "yolo_node");
  ros::NodeHandle yoloNode;

  ros::Subscriber boundingBoxSub = yoloNode.subscribe("/darknet_ros/bounding_boxes/", 1, objectsCb);
  ros::Subscriber imageSub = yoloNode.subscribe("/darknet_ros/detection_image/", 1, imageCb);

  findPub = yoloNode.advertise<std_msgs::Bool>(TPC_YOLO_NODE_TARGET_SEEN, 1);
  proofPub = yoloNode.advertise<sensor_msgs::Image>(TPC_YOLO_NODE_TARGET_IMAGE, 1);

  ros::spin();
}
