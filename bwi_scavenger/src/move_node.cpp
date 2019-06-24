#include <bwi_scavenger/global_topics.h>
#include <bwi_scavenger/move_node.h>

#include <math.h>
#include <limits.h>
#include <std_msgs/String.h>

int closest = -1;
tf::TransformListener *listener;
std::string gridFrameId;

void stop(const bwi_scavenger::RobotStop::ConstPtr &data){
    ROS_INFO("[move_node] Cancel goal");
    rm -> end_movement();
    // movePub.publish(result);
}

void move(const bwi_scavenger::RobotMove::ConstPtr &data){
  if(closest != -1){
    if(data -> type == MOVE){
      environment_location goal = static_cast<environment_location>((data -> location + closest) % NUM_ENVIRONMENT_LOCATIONS);
      ROS_INFO("[move_node] Moving to location %d.", (int)goal);
      rm -> move_to_location(goal);
      movePub.publish(result);
    } else if (data -> type == SPIN){
      rm -> turn (data->degrees);
      movePub.publish(result);
    }
  }
}

void start(const std_msgs::String::ConstPtr &msg){
  if (msg -> data == "Find Object") {
    ROS_INFO("[move_node] Finding closest waypoint...");
    tf::StampedTransform robotTransform;
    listener->waitForTransform(gridFrameId, "base_link", ros::Time::now(), ros::Duration(4));
    listener->lookupTransform(gridFrameId, "base_link", ros::Time(0), robotTransform);
    float x = robotTransform.getOrigin().x();
    float y = robotTransform.getOrigin().y();

    float minDistance = std::numeric_limits<float>::max();
    for(int i = 0; i < NUM_ENVIRONMENT_LOCATIONS; i++){
      std::pair<float, float> coordinates = environment_location_coordinates[static_cast<environment_location>(i)];
      float distance = sqrt(pow(coordinates.first - x, 2) + pow(coordinates.second - y, 2));
      if (distance < minDistance){
        minDistance = distance;
        closest = i;
      }
    }

    ROS_INFO("[move_node] Setting destination to %d.", closest);
  }
}

void getMapId(const nav_msgs::OccupancyGrid::ConstPtr &grid){
  ROS_INFO("Creating RobotMotion");
  listener = new tf::TransformListener();
  gridFrameId = grid->header.frame_id;
  rm = new RobotMotion(gridFrameId, *listener);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "move_node");
  ros::NodeHandle moveNode;

  ros::Subscriber mapSub = moveNode.subscribe("/level_mux/map", 1, getMapId);

  ros::Subscriber findObjectSub = moveNode.subscribe(TPC_MOVE_NODE_GO, 1, move);
  ros::Subscriber stopMoveSub = moveNode.subscribe(TPC_MOVE_NODE_STOP, 1, stop);
  ros::Subscriber startFindObject = moveNode.subscribe(TPC_MAIN_NODE_TASK_START, 1, start);

  movePub = moveNode.advertise<std_msgs::Bool>(TPC_MOVE_NODE_FINISHED, 1);

  ROS_INFO("[move_node] Standing by.");

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
}
