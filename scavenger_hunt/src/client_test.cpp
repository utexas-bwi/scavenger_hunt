#include <fstream>
#include <iostream>
#include <scavenger_hunt/rapidxml.hpp>
#include <scavenger_hunt/scavenger_hunt.h>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

using namespace rapidxml;

int main(int argc, char** argv) {
  ScavengerHuntClient client("stefandebruyn@utexas.edu", "sick robots");
  std::vector<Task> tasks;
  client.get_hunt("BWI Lab Hunt", tasks);

  for (int i = 0; i < tasks.size(); i++) {
    Task &t = tasks[i];
    std::cout << t << std::endl;
  }

  // client.send_proof("bottle.png", (*hunt)[0]);
}
