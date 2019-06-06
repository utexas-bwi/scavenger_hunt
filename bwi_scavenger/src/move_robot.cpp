#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::string gridFrameId;
bool moved = false;

void map(const nav_msgs::OccupancyGrid::ConstPtr &grid){
  gridFrameId = grid->header.frame_id;
}

void move(){
  if(!moved){

    // tf::StampedTransform initialBaseTransform;

    // tfL.waitForTransform(gridFrameId, "base_link", ros::Time::now(), ros::Duration(4));
    // tfL.lookupTransform(gridFrameId, "base_link", ros::Time(0), initialBaseTransform);

    // initialPose.position.x = initialBaseTransform.getOrigin().getX();
    // initialPose.position.y = initialBaseTransform.getOrigin().getY();
    // initialPose.position.z = initialBaseTransform.getOrigin().getZ();

    // initialPose.orientation.x = initialBaseTransform.getRotation().getX();
    // initialPose.orientation.y = initialBaseTransform.getRotation().getY();
    // initialPose.orientation.z = initialBaseTransform.getRotation().getZ();
    // initialPose.orientation.w = initialBaseTransform.getRotation().getW();

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    geometry_msgs::PoseStamped goalPose;

    //set goalPose to target goal
    goalPose.header.seq = 0;
    goalPose.header.stamp = ros::Time::now();
    goalPose.header.frame_id = gridFrameId;

    goalPose.pose.position.x = -39.33;
    goalPose.pose.position.y = -11.23;
    goalPose.pose.orientation.w = 1;

    geometry_msgs::PoseStamped tag_rel_pose;
    
    tf::TransformListener tfL;

    tfL.waitForTransform("base_link", goalPose.header.frame_id, ros::Time::now(), ros::Duration(4));
    tfL.transformPose("base_link", goalPose, tag_rel_pose);

    tag_rel_pose.pose.position.z = 0;
    goal.target_pose = tag_rel_pose;

    ac.sendGoal(goal);

    ac.waitForResult();
    moved = true;
  }

} 

int main(int argc, char **argv){

  ros::init(argc, argv, "coor_point");

  ros::NodeHandle node;

  ros::Subscriber mapSub = node.subscribe("/level_mux/map", 1, map);
  move();

  ros::spin();
  

}
