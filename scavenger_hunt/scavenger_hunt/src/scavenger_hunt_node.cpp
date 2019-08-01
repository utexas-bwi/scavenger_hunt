#include <scavenger_hunt_msgs/GetHunt.h>
#include <scavenger_hunt_msgs/GetProofStatus.h>
#include <scavenger_hunt_msgs/Hunt.h>
#include <scavenger_hunt_msgs/Login.h>
#include <scavenger_hunt_msgs/Parameter.h>
#include <scavenger_hunt_msgs/SendProof.h>
#include <scavenger_hunt_msgs/Task.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <vector>

#include "scavenger_hunt/scavenger_hunt.h"

static ScavengerHuntClient* scav_client = nullptr;

static const std::string PROOF_TEMP_PATH = "/home/bwilab/proof.jpeg";

void login(const scavenger_hunt_msgs::Login& msg) {
  if (scav_client != nullptr)
    delete scav_client;

  scav_client = new ScavengerHuntClient(msg.username, msg.password);

  ROS_INFO("Created client with email %s", msg.username.c_str());
}

bool serve_hunt(scavenger_hunt_msgs::GetHunt::Request& req,
                scavenger_hunt_msgs::GetHunt::Response& res)
{
  std::vector<Task> tasks;
  scavenger_hunt_msgs::Hunt hunt;

  scav_client->get_hunt(req.hunt_name, tasks);

  for (Task t : tasks) {
    scavenger_hunt_msgs::Task scav_task;
    scav_task.name = t.get_name();
    scav_task.description = t.get_description();
    scav_task.hunt_name = req.hunt_name;

    for (const auto& pair : t.get_parameters()) {
      scavenger_hunt_msgs::Parameter p;
      p.name = pair.first;
      p.value = pair.second;
      scav_task.parameters.push_back(p);
    }

    scav_task.proof_format = t.get_proof_format();
    scav_task.proof_description = t.get_proof_format_description();
    scav_task.point_value = t.get_point_value();
    scav_task.id = t.get_hunt_task_id();

    hunt.tasks.push_back(scav_task);
  }

  res.hunt = hunt;

  return true;
}

bool send_proof(scavenger_hunt_msgs::SendProof::Request& req,
                scavenger_hunt_msgs::SendProof::Response& res)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(
    req.proof.image,
    sensor_msgs::image_encodings::BGR8
  );
  cv::Mat image_raw = cv_ptr->image;
  cv::imwrite(PROOF_TEMP_PATH, image_raw);

  proof_id_t id = scav_client->send_proof(
    PROOF_TEMP_PATH,
    req.task.id,
    req.proof.task_duration
  );

  res.id = id;

  return true;
}

bool get_proof_status(scavenger_hunt_msgs::GetProofStatus::Request& req,
                      scavenger_hunt_msgs::GetProofStatus::Response& res)
{
  proof_status_t status = scav_client->get_proof_status(req.id);
  res.status = status;

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "scavenger_hunt_node");
  ros::NodeHandle nh;

  std::string scav_email, scav_pass;
  nh.param("scavenger_hunt/login/email", scav_email, std::string(""));
  nh.param("scavenger_hunt/login/password", scav_pass, std::string(""));

  scavenger_hunt_msgs::Login login_msg;
  login_msg.username = scav_email;
  login_msg.password = scav_pass;

  login(login_msg);

  ros::Subscriber sub0 = nh.subscribe("/scavenger_hunt/login", 1, login);
  ros::ServiceServer srv0 = nh.advertiseService("/scavenger_hunt/get_hunt", serve_hunt);
  ros::ServiceServer srv1 = nh.advertiseService("/scavenger_hunt/send_proof", send_proof);
  ros::ServiceServer srv2 = nh.advertiseService("/scavenger_hunt/get_proof_status", get_proof_status);

  ROS_INFO("[scavenger_hunt_node] Standing by.");

  ros::spin();
}
