#include <ros/ros.h>

int main(int argc, char** argv){
  std::string nameOfBag = "three_cups.bag";
  std::string nameOfVideo = "video.mp4";
  std::string command = "cd /home/bwilab/scavenger_hunt_ws/; rm " + nameOfVideo + "; ";
  command += "./rosbag2video.py -o " + nameOfVideo + " " + nameOfBag;
  system(command.c_str());  
}