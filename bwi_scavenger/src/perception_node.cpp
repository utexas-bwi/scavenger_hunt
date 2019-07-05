#include <actionlib/client/simple_action_client.h>
#include <bwi_scavenger/globals.h>
#include <bwi_scavenger_msgs/PerceptionMoment.h>
#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>

typedef darknet_ros_msgs::CheckForObjectsAction YOLOAction;
typedef darknet_ros_msgs::CheckForObjectsGoal YOLOGoal;
typedef actionlib::SimpleActionClient<YOLOAction> YOLOActionClient;
typedef std::shared_ptr<YOLOActionClient> YOLOActionClientPtr;

static darknet_ros_msgs::BoundingBoxes cv_sweep_result_dest;

static sensor_msgs::Image depth_image;

static unsigned long moment_uid = 0;

static std::vector<std::string> moment_callbacks;

static ros::Publisher pub_moment;
static ros::NodeHandle *nh;

/**
  @brief buffers the most recent Kinect depth map to package with perception
         moments
*/
void save_depth(const sensor_msgs::Image::ConstPtr &msg) {
  depth_image = *msg;
}

/**
  @brief called by the YOLO server once CV processing finishes
*/
void cv_sweep_cb(const actionlib::SimpleClientGoalState& state,
    const darknet_ros_msgs::CheckForObjectsResultConstPtr& result) {
  cv_sweep_result_dest = result->bounding_boxes;
}

/**
  @brief called as fast as possible on Kinect color images; when a vision sweep
         of that image completes, the image, bounding boxes, and latest
         depth map are packaged into a PerceptionMoment and broadcast
*/
void vision(const sensor_msgs::Image &msg) {
  YOLOActionClientPtr cv_sweep_ac;

  // Create action client and associate it with the correct action
  std::string cv_sweep_action_name;
  nh->param("/darknet_ros/camera_action", cv_sweep_action_name,
      std::string("/darknet_ros/check_for_objects"));
  cv_sweep_ac.reset(new YOLOActionClient(*nh, cv_sweep_action_name, true));

  // Wait for the action server to launch
  if (!cv_sweep_ac->waitForServer(ros::Duration(10.0))) {
    ROS_ERROR("[perception_node] Failed to connect to YOLO server. Aborting perception moment.");
	  return;
  }

  // Send goal to action server
  YOLOGoal goal;
  goal.image = msg;

  cv_sweep_ac->sendGoal(
      goal,
      boost::bind(&cv_sweep_cb, _1, _2),
      YOLOActionClient::SimpleActiveCallback(),
      YOLOActionClient::SimpleFeedbackCallback());

  // Wait for result
  if (!cv_sweep_ac->waitForResult(ros::Duration(5.0))) {
    ROS_ERROR("[perception_node] YOLO took too long to respond. Aborting perception moment.");
    return;
  }

  // Build perception moment
  bwi_scavenger_msgs::PerceptionMoment perception;
  perception.color_image = msg;
  perception.depth_image = depth_image;
  perception.bounding_boxes = cv_sweep_result_dest;
  perception.uid = moment_uid;

  moment_uid++;

  pub_moment.publish(perception);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "perception_node");
  nh = new ros::NodeHandle();

  pub_moment = nh->advertise<bwi_scavenger_msgs::PerceptionMoment>(
      TPC_PERCEPTION_NODE_MOMENT, 1);
  ros::Subscriber sub_depth =
      nh->subscribe("/camera/depth/image", 1, save_depth);
  ros::Subscriber sub_color =
      nh->subscribe("/camera/rgb/image_color", 1, vision);

  ROS_INFO("[perception_node] Standing by.");

  ros::spin();

  return 0;
}
