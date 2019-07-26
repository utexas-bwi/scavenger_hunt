#include <bwi_scavenger_msgs/PerceptionMoment.h>
#include <bwi_scavenger_msgs/PoseRequest.h>
#include <kinect_fusion/kinect_fusion.h>
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <vector>

#include "bwi_scavenger/globals.h"

static ros::ServiceClient client_pose_request;

typedef struct {
  std::string label;
  double x;
  double y;
  double z;
} object_t;

static std::string target = "chair";
static float identity_distance = 0.25;
static std::vector<object_t> memory;

void think(const object_t& obj) {
  bool obj_new = false;

  ROS_INFO("Current object memory:");

  for (int i = 0; i < memory.size(); i++) {
    const object_t& obj_mem = memory[i];

    ROS_INFO("  %s @ (%f, %f, %f)",
      obj_mem.label, obj_mem.x, obj_mem.y, obj_mem.z
    );

    if (obj_mem.label != obj.label)
      continue;

    if (!obj_new) {
      double dx = obj.x - obj_mem.x;
      double dy = obj.y - obj_mem.y;
      double dz = obj.z - obj_mem.z;
      double dist = sqrt(dx * dx + dy * dy + dz * dz);

      if (dist > identity_distance)
        obj_new = true;
    }
  }

  if (obj_new) {
    ROS_INFO("Saw new %s at (%f, %f, %f)",
      obj.label, obj.x, obj.y, obj.z
    );
    memory.push_back(obj);
  } else
    ROS_INFO("I've seen this %s before", obj.label);
}

void perceive(const bwi_scavenger_msgs::PerceptionMoment::ConstPtr& msg) {
  // Get robot pose
  bwi_scavenger_msgs::PoseRequest req_pose;
  client_pose_request.call(req_pose);

  tf::Quaternion tf_quat(
    req_pose.response.pose.orientation.x,
    req_pose.response.pose.orientation.y,
    req_pose.response.pose.orientation.z,
    req_pose.response.pose.orientation.w
  );
  double roll, pitch, yaw;
  tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

  double x = req_pose.response.pose.position.x;
  double y = req_pose.response.pose.position.y;
  double z = req_pose.response.pose.position.z;

  // Look for target
  for (int i = 0; i < msg->bounding_boxes.bounding_boxes.size(); i++)
    if (msg->bounding_boxes.bounding_boxes[i].Class == target) {
      geometry_msgs::Point rel = kinect_fusion::get_position(
        msg->bounding_boxes.bounding_boxes[i], msg->depth_image
      );

      if (rel.x > 0) {
        double obj_x = x + cos(yaw) * rel.x - sin(yaw) * rel.y;
        double obj_y = y + sin(yaw) * rel.x + cos(yaw) * rel.y;
        double obj_z = z + rel.z;

        object_t obj;
        obj.label = target;
        obj.x = obj_x;
        obj.y = obj_y;
        obj.z = obj_z;

        think(obj);
      }
    }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "memory_node");
  ros::NodeHandle nh;

  ros::Subscriber sub0 = nh.subscribe(TPC_PERCEPTION_NODE_MOMENT, 1, perceive);
  client_pose_request = nh.serviceClient<bwi_scavenger_msgs::PoseRequest>(
      SRV_POSE_REQUEST);

  ros::Duration(3.0).sleep();

  ROS_INFO("[memory_node] Standing by.");

  ros::spin();
}
