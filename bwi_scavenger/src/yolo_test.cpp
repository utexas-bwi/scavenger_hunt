#include <bwi_scavenger/cv_sweep.h>
#include <ros/ros.h>

class YoloTest {
private:
  ros::NodeHandle *nh;
  bool image_received = false;
  darknet_ros_msgs::BoundingBoxes boxes;

public:
  YoloTest(ros::NodeHandle *nh) {
    this->nh = nh;
  }

  void vision(const sensor_msgs::Image::ConstPtr &msg) {
    if (image_received)
      return;
    else
      image_received = true;

    cv_sweep(*nh, *msg, &boxes);

    ROS_INFO("Vision processed!");

    for (int i = 0; i < boxes.bounding_boxes.size(); i++) {
      darknet_ros_msgs::BoundingBox &box = boxes.bounding_boxes[i];
      ROS_INFO("%s: %f", box.Class.c_str(), box.probability);
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "yolo_test");
  ros::NodeHandle nh;
  YoloTest yolo(&nh);

  ros::Duration(1.0).sleep();

  ros::Subscriber sub = nh.subscribe("camera/rgb/image_raw", 100,
      &YoloTest::vision, &yolo);
  ros::spin();
  return 0;
}
