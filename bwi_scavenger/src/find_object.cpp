#include <ros/ros.h>
#include <bwi_scavenger/robot_motion.h>
#include <bwi_scavenger/proof.h>
#include "opencv2/opencv.hpp"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Image.h>
#include <string>


const std::string thisTask = "Find Object";
std::string objectToFind = "chair";
ros::Publisher proofPub;
ros::Publisher findPub;
bool objectFound = false;
bool saved = false;
sensor_msgs::Image image;
std_msgs::String found;

// goes through the objects in view and checks if the object has been found
void objectsCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &objects){
  if(!objectFound){
    for(int i = 0; i < objects -> bounding_boxes.size(); i++){
      if (objects -> bounding_boxes[i].Class == objectToFind){
        objectFound = true;
        found.data = "found";
        findPub.publish(found);
      }
    }
  }
}

// saves the image that YOLO produces if the object has been found in that image
void imageCb(const sensor_msgs::Image::ConstPtr &img){
  if(objectFound && !saved){
    std::cout << "found object, now \"saving\" it!" << std::endl;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    // cv::imshow("HI", cv_ptr->image);
    //names the image by its header sequence
    std::string name = std::to_string(img->header.seq);
    cv::imwrite(name + ".jpg", cv_ptr -> image);
    //give back image name to the 
    // proofPub.publish(*img);
    saved = true;
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "find_object");
  ros::NodeHandle yoloNode;

  ros::Subscriber boundingBoxSub = yoloNode.subscribe("/darknet_ros/bounding_boxes/", 100, objectsCb);
  ros::Subscriber imageSub = yoloNode.subscribe("/darknet_ros/detection_image/", 100, imageCb);

  findPub = yoloNode.advertise<std_msgs::String>("/scavenger/target_seen", 100);

  proofPub = yoloNode.advertise<bwi_scavenger::proof>("proof", 100);

  ros::spin();
}