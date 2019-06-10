#include <ros/ros.h>
#include <bwi_scavenger/robot_motion.h>
#include <std_msgs/String.h>

RobotMotion *rm;
bool objectFound = false;

void findObjectCb(const std_msgs::String::ConstPtr &string){
  objectFound = string -> data == "found";
}

// callback for if the task is to find the object
// void moveRobot(const std_msgs::String::ConstPtr &task){
void moveRobot(const nav_msgs::OccupancyGrid::ConstPtr &grid){
  rm = new RobotMotion(grid->header.frame_id);
  // if(task->data == thisTask){
    // obejctToFind = "chair";
    int numLocations = 7;
    int rotation = 45;
    // loop through points that the robot will explore and send to move_robot
    for(int location = 0; location < numLocations; location++){
      environment_location goal = static_cast<environment_location>(location);
      rm->move_to_location(goal);
      ros::spinOnce();
      // rotate enough to make a 360
      for(int i = 0; i < 360 / rotation; i++){
        if(objectFound){
          break; // then return home
        }
        rm->turn(rotation);
        //buffer for robot to look for object because YOLO is slow on the computers
        ros::Duration(2).sleep();
        ros::spinOnce();
      }
      if(objectFound){
        break; // then return home
      }
    }
  // }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "move_robot");
  ros::NodeHandle movementNode;
  ros::NodeHandle yoloNode;

  ros::Subscriber mapSub = movementNode.subscribe("/level_mux/map", 100, moveRobot);
  ros::Subscriber findObjectSub = yoloNode.subscribe("/objectFinder", 100, findObjectCb);

  ros::spin();
}