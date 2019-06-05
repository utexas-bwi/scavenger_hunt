#include <ros/ros.h>
#include <scavenger_hunt/scavenger_hunt.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ROS_INFO("hello");

  ScavengerHuntClient client("stefandebruyn@utexas.edu", "sick robots");
  ScavengerHunt *hunt = client.get_hunt("Longhorn Hunt");

  if (hunt != NULL) {
    for (int i = 0; i < hunt->size(); i++)
      std::cout << hunt->get_task(i); // (*hunt)[i] also works

    client.send_proof("bottle.png", (*hunt)[0]);
  }
}
