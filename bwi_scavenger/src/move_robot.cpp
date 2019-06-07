#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::string gridFrameId = "";
bool moved = false;
tf::TransformListener* tfl = NULL;

void move(){
  if(!moved && gridFrameId != ""){

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    geometry_msgs::PoseStamped goalPose;

    goalPose.header.seq = 0;
    goalPose.header.stamp = ros::Time(0);
    goalPose.header.frame_id = gridFrameId;

    //temp goalPose at a point outside bwi lab next to grad student cubicles
    goalPose.pose.position.x = -39.33;
    goalPose.pose.position.y = -11.23;

    // test points for simulation
    // goalPose.pose.position.x = 14.956;
    // goalPose.pose.position.y = 109.94;

    //TODO adjust this so it doesn't stay the same orientation all the time
    goalPose.pose.orientation.w = 1;

    geometry_msgs::PoseStamped tag_rel_pose;

    tfl->waitForTransform("base_link", goalPose.header.frame_id, ros::Time::now(), ros::Duration(4));
    //ERROR exrtapolation into past
    tfl->transformPose("base_link", goalPose, tag_rel_pose);

    tag_rel_pose.pose.position.z = 0;
    goal.target_pose = tag_rel_pose;

    ac.sendGoal(goal);

    ac.waitForResult();
    moved = true;
  }
} 

void map(const nav_msgs::OccupancyGrid::ConstPtr &grid){
  gridFrameId = grid->header.frame_id;
  move();
}

int main(int argc, char **argv){
  ros::init(argc, argv, "move_robot");
  ros::NodeHandle node;
  tfl = new (tf::TransformListener);
  ros::Subscriber mapSub = node.subscribe("/level_mux/map", 1, map);
  ros::spin();
}
