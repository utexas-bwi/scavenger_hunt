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


const std::string thisTask = "Find Object";
std::string objectToFind = "";
ros::Publisher proofPub;
bool objectFound = false;
sensor_msgs::Image image;
RobotMotion *rm;

// goes through the objects in view and checks if the object has been found
void objects(const darknet_ros_msgs::BoundingBoxes::ConstPtr &objects){
  for(int i = 0; i < objects -> bounding_boxes.size(); i++){
    if (objects -> bounding_boxes[i].Class == objectToFind){
      objectFound = true;
    }
  }
}

// saves the image that YOLO produces if the object has been found in that image
void imageCb(const sensor_msgs::Image::ConstPtr &img){
  if(objectFound = true){
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    // names the image by its header sequence
    cv::imwrite(img->header.seq + "", cv_ptr -> image);
    // give back image name to the 
    proofPub.publish(*img);
    objectFound = false;
  }
}

// callback for if the task is to find the object
// void findObject(const std_msgs::String::ConstPtr &task){
void findObject(const nav_msgs::OccupancyGrid::ConstPtr &grid){
  // if(task->data == thisTask){
    //TODO get name of the object to find
    objectToFind = "chair";
    int numLocations = 7;
    int rotation = 45;
    // loop through points that the robot will explore and send to move_robot
    for(int location = 0; location < numLocations; location++){
      environment_location goal = static_cast<environment_location>(location);
      rm->move_to_location(goal);
      // rotate enough to make a 360
      for(int i = 0; i < 360 / rotation; i++){
        if(objectFound)
          break; // then return home
        rm->turn(rotation);
        //buffer for robot to look for object because YOLO is slow on the computers
        ros::Duration(2).sleep();
      }
    }
  // }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "find_object");
  ros::NodeHandle node;

  //TODO get the grid frame id 
  rm = new RobotMotion("asdf");

  ros::Subscriber boundingBoxSub = node.subscribe("/darknet_ros/bounding_boxes/", 100, objects);
  ros::Subscriber imageSub = node.subscribe("/darknet_ros/detection_image/", 100, imageCb);

  ros::Subscriber taskSub = node.subscribe("/level_mux/map", 100, findObject);

  proofPub = node.advertise<bwi_scavenger::proof>("proof", 100);

  ros::spin();
}