#include <iostream>
#include <fstream>
#include <scavenger_hunt/scavenger_hunt.h>
#include <ros/ros.h>

#include <bwi_scavenger_msgs/SendProof.h>

#include <sensor_msgs/Image.h>

#include <scavenger_hunt_msgs/GetHunt.h>
#include <scavenger_hunt_msgs/GetProofStatus.h>
#include <scavenger_hunt_msgs/SendProof.h>

#include <scavenger_hunt_msgs/Hunt.h>
#include <scavenger_hunt_msgs/Login.h>
#include <scavenger_hunt_msgs/Parameter.h>
#include <scavenger_hunt_msgs/Proof.h>
#include <scavenger_hunt_msgs/Task.h>

sensor_msgs::Image image;
bool sent = false;

ros::ServiceClient client_get_hunt, client_get_proof_status, client_send_proof;

void save_image(const sensor_msgs::Image::ConstPtr& msg) {
  if (sent)
    return;

  image = *msg;
  sent = true;

  scavenger_hunt_msgs::GetHunt get_hunt;
  get_hunt.request.hunt_name = "BWI Lab Hunt";
  client_get_hunt.call(get_hunt);

  scavenger_hunt_msgs::Proof proof;
  proof.image = image;
  scavenger_hunt_msgs::Task task = get_hunt.response.hunt.tasks[0];
  geometry_msgs::Point robot_position;
  robot_position.x = robot_position.y = robot_position.z = 3;

  geometry_msgs::Point secondary_position;
  secondary_position.x = secondary_position.y = secondary_position.z = 5;

  bwi_scavenger_msgs::SendProof send_proof;
  send_proof.request.task = task;
  send_proof.request.proof = proof;
  send_proof.request.robot_position = robot_position;
  send_proof.request.secondary_position = secondary_position;
  send_proof.request.metadata="10,40,100,150";

  client_send_proof.call(send_proof);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "client_example");
  ros::NodeHandle nh;

  ros::Publisher pub_login = nh.advertise<scavenger_hunt_msgs::Login>(
    "/scavenger_hunt/login",
    1
  );

  client_get_hunt = nh.serviceClient<scavenger_hunt_msgs::GetHunt>(
    "/scavenger_hunt/get_hunt"
  );
  client_get_proof_status = nh.serviceClient<scavenger_hunt_msgs::GetProofStatus>(
    "/scavenger_hunt/get_proof_status"
  );
  client_send_proof = nh.serviceClient<bwi_scavenger_msgs::SendProof>(
    "/bwi_scavenger/proofdb_node/send_proof"
  );

  ros::Subscriber sub0 = nh.subscribe(
    "/camera/rgb/image_color",
    1,
    save_image
  );

  ros::Duration(2.0).sleep();

  scavenger_hunt_msgs::Login login;
  login.username = "stefandebruyn@utexas.edu";
  login.password = "sick robots";

  pub_login.publish(login);

  ros::Duration(1.0).sleep();

  ROS_INFO("Ready!");

  ros::spin();

  // // Create a new client object and sign in with your website credentials.
  // ScavengerHuntClient client("stefandebruyn@utexas.edu", "sick robots");
  //
  // // Create an empty vector of tasks for the client to populate with hunt data.
  // std::vector<Task> tasks;
  //
  // // Scavenger hunts are downloaded by name. We've created a never-expiring hunt
  // // called Bottle Hunt to test with. It contains a single task to find a bottle
  // // and take a picture of it.
  // client.get_hunt("Bottle Hunt", tasks);
  //
  // // Task objects contain all of the information necessary to complete them.
  // for (int i = 0; i < tasks.size(); i++) {
  //   Task &task = tasks[i];
  //
  //   // Insert a task into a stream to get a pretty-printed summary.
  //   std::cout << task << std::endl;
  //
  //   // Access to task details. See
  //   // include/scavenger_hunt/scavenger_hunt_structure.h for the full docs.
  //   std::string task_name = task.get_name();
  //   std::string task_description = task.get_description();
  //   std::string target_object = task.get_parameter_value("object"); // Specific to the "Find Object" task
  // }
  //
  // // Upload proof of each completed task with send_proof. This method takes a
  // // path to a proof file (either an image or a video), the task in question,
  // // and the time taken to complete the task in seconds.
  // proof_id_t id = client.send_proof("/home/bwilab/scavenger_hunt/proof.jpeg", tasks[0], 60.0);
  //
  // // A proof ID can be used to download the original uploaded file.
  // client.download_proof_material(id, "/home/my_downloaded_proofs/proof.jpeg");
  //
  // // The upload process generates a unique ID for the proof that can be used to
  // // get feedback on it.
  // proof_status_t status = client.get_proof_status(id);
  //
  // if (status == PROOF_CORRECT)
  //   std::cout << ":)";
  // else if (status == PROOF_INCORRECT)
  //   std::cout << ":(";
  // else if (status == PROOF_NOT_VALIDATED)
  //   std::cout << ":|";
  //
  // // Alternatively, all of the proofs submitted for a particular task that have
  // // been validated can be downloaded in bulk with get_proofs.
  // std::vector<Proof> proofs;
  // client.get_proofs(tasks[0], proofs);
  //
  // for (int i = 0; i < proofs.size(); i++) {
  //   Proof &proof = proofs[i];
  //   bool is_correct = proof.get_correct(); // Was I right?
  //
  //   // The original file uploaded as proof can be retrieved if necessary.
  //   std::string download_directory = "/home/my_downloaded_proofs";
  //   client.download_proof_material(proof, download_directory);
  //   std::string path_to_proof = download_directory + "/" + proof.get_filename();
  // }
}
