#include <bwi_scavenger/global_topics.h>
#include <bwi_scavenger/move_node.h>

#include <math.h>
#include <limits.h>

int closest = 0;

void stop(const bwi_scavenger::RobotStop::ConstPtr &data){
    ROS_INFO("[move_node] Cancel goal");
    rm -> end_movement();
    movePub.publish(result);
}

void move(const bwi_scavenger::RobotMove::ConstPtr &data){
  if(data -> type == MOVE){
    ROS_INFO("[move_node] Sending goal to move");
    environment_location goal = static_cast<environment_location>((data -> location + closest) % 7);
    rm -> move_to_location(goal);
    movePub.publish(result);
  } else if (data -> type == SPIN){
    ROS_INFO("[move_node] Sending goal to spin");
    rm -> turn (data->degrees);
    movePub.publish(result);
  }
}

void getMapId(const nav_msgs::OccupancyGrid::ConstPtr &grid){
  tf::TransformListener *tfl = new tf::TransformListener();
  rm = new RobotMotion(grid->header.frame_id, *tfl);

  tf::StampedTransform robotTransform;
  tfl->waitForTransform("base_link", grid->header.frame_id, ros::Time::now(), ros::Duration(4));
  tfl->lookupTransform("base_link", grid->header.frame_id, ros::Time(0), robotTransform);
  float x = robotTransform.getOrigin().x();
  float y = robotTransform.getOrigin().y();

  float minDistance = std::numeric_limits<float>::max();
  for(int i = 0; i < 7; i++){
    std::pair<float, float> coordinates = environment_location_coordinates[static_cast<environment_location>(i)];
    float distance = sqrt(pow(coordinates.first - x, 2) + pow(coordinates.second - y, 2));
    if (distance < minDistance){
      minDistance = distance;
      closest = i;
    }
  }
  
}

int main(int argc, char **argv){
  ros::init(argc, argv, "move_node");
  ros::NodeHandle moveNode;

  ros::Subscriber mapSub = moveNode.subscribe("/level_mux/map", 1, getMapId);

  ros::Subscriber findObjectSub = moveNode.subscribe(TPC_MOVE_NODE_GO, 1, move);
  ros::Subscriber stopMoveSub = moveNode.subscribe(TPC_MOVE_NODE_STOP, 1, stop);

  movePub = moveNode.advertise<std_msgs::Bool>(TPC_MOVE_NODE_FINISHED, 1);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
}
