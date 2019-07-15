#include <ros/ros.h>
#include <scavenger_hunt/scavenger_hunt.h>

int main(int argc, char** argv){
  std::string nameOfBag = "three_cups.bag";
  std::string nameOfVideo = "video.mp4";
  std::string command = "cd /home/bwilab/scavenger_hunt_ws/; rm " + nameOfVideo + "; ";
  command += "./rosbag2video.py -o " + nameOfVideo + " " + nameOfBag;
  system(command.c_str());  

  std::cin.ignore();
  
  ScavengerHuntClient client("jsuriadinata@utexas.edu", "Tr3asure");

  std::vector<Task> tasks;

  client.get_hunt("Video Hunt", tasks);

  proof_id_t id = client.send_proof("/home/bwilab/scavenger_hunt_ws/video.mp4", tasks[0], 60.0);
}