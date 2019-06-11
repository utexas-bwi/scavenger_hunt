#include "bwi_scavenger/robot_motion.h"
#include "bwi_scavenger/scavenger_move.h"
#include "bwi_scavenger/scavenger_stop.h"
#include <std_msgs/Bool.h>

RobotMotion *rm;
ros::Publisher movePub;
std_msgs::Bool result;
int MOVE = 0;
int SPIN = 1;
int STOP = 2;

void stop(const bwi_scavenger::scavenger_stop::ConstPtr &data){
    ROS_INFO("[Move_node] Cancel goal");
    rm -> end_movement();
    movePub.publish(result);
}

void move(const bwi_scavenger::scavenger_move::ConstPtr &data){
  if(data -> type == MOVE){
    ROS_INFO("[Move_node] Sending goal to move");
    environment_location goal = static_cast<environment_location>(data -> location);
    rm -> move_to_location(goal);
    movePub.publish(result);
  } else if (data -> type == SPIN){
    ROS_INFO("[Move_node] Sending goal to spin");
    rm -> turn (data->degrees);
    movePub.publish(result);
  } 
}

void getMapId(const nav_msgs::OccupancyGrid::ConstPtr &grid){
  rm = new RobotMotion(grid->header.frame_id);
}

int main(int argc, char **argv){
  result.data = true;
  ros::init(argc, argv, "move_node");
  ros::NodeHandle moveNode;

  ros::Subscriber mapSub = moveNode.subscribe("/level_mux/map", 100, getMapId);

  ros::Subscriber findObjectSub = moveNode.subscribe("/scavenger/find_object/move", 100, move);
  ros::Subscriber stopMoveSub = moveNode.subscribe("/scavenger/find_object/stop", 100, stop);

  movePub = moveNode.advertise<std_msgs::Bool>("/scavenger/find_object/move_finished", 100);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
}