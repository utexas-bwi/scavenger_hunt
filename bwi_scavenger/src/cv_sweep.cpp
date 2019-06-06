#include <bwi_scavenger/cv_sweep.h>

darknet_ros_msgs::BoundingBoxes *cv_sweep_result_dest;

void cv_sweep_cb(const actionlib::SimpleClientGoalState& state,
    const darknet_ros_msgs::CheckForObjectsResultConstPtr& result) {
  *cv_sweep_result_dest = result->bounding_boxes;
}

bool cv_sweep(ros::NodeHandle &nh, const sensor_msgs::Image &image,
    darknet_ros_msgs::BoundingBoxes *dest) {
  CvSweepActionClientPtr cv_sweep_ac;

  // Create action client and associate it with the correct action
  std::string cv_sweep_action_name;
  nh.param("/darknet_ros/camera_action", cv_sweep_action_name,
      std::string("/darknet_ros/check_for_objects"));
  cv_sweep_ac.reset(
      new CvSweepActionClient(nh, cv_sweep_action_name, true));

  // Wait for the action server to launch
  if (!cv_sweep_ac->waitForServer(ros::Duration(10.0))) {
    ROS_INFO("[cv_sweep] Failed to connect to CvSweepActionClient. Is Darknet running?");
	  return false;
  }

  // Send goal to action server
  CvSweepGoal goal;
  goal.image = image;
  ros::Time t_sweep_begin = ros::Time::now();
  cv_sweep_result_dest = dest;

  cv_sweep_ac->sendGoal(
      goal,
      boost::bind(&cv_sweep_cb, _1, _2),
      CvSweepActionClient::SimpleActiveCallback(),
      CvSweepActionClient::SimpleFeedbackCallback());

  // Wait for result
  if (!cv_sweep_ac->waitForResult(ros::Duration(120.0))) {
    ROS_INFO("[cv_sweep] CvSweepActionClient took too long to respond!");
    return false;
  }

  // Compute duration and conclude
  ros::Time t_sweep_end = ros::Time::now();

  ROS_INFO("[cv_sweep] Identified %d objects in %f seconds",
      cv_sweep_result_dest->bounding_boxes.size(), t_sweep_end - t_sweep_begin);

  return true;
}

bool cv_sweep_local(ros::NodeHandle &nh, const std::string &image_path,
    darknet_ros_msgs::BoundingBoxes *dest) {
  ROS_INFO("[cv_sweep_local] Loading image...");

  // Pack image into appropriate message type
  cv_bridge::CvImage image_bridge;
  sensor_msgs::Image image_msg;
  std_msgs::Header header;
  cv::Mat cv_image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);

  if (cv_image.empty()) {
    ROS_INFO("[cv_sweep_local] Failed to load file \"%s\"", image_path);
    return false;
  }

  header.seq = 0;
  header.stamp = ros::Time::now();
  image_bridge =
    cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, cv_image);
  image_bridge.toImageMsg(image_msg);

  ROS_INFO("[cv_sweep_local] Starting sweep...");

  return cv_sweep(nh, image_msg, dest);
}
