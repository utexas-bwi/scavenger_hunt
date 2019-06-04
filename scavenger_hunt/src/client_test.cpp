#include <fstream>
#include <iostream>
#include <scavenger_hunt/rapidxml.hpp>
#include <scavenger_hunt/scavenger_hunt.h>
#include <string>
#include <vector>

using namespace rapidxml;

int main(int argc, char** argv) {
  ScavengerHuntClient client("stefandebruyn@utexas.edu", "sick robots");
  ScavengerHunt *hunt = client.get_hunt("Longhorn Hunt");

  for (int i = 0; i < hunt->size(); i++)
    std::cout << hunt->get_task(i); // (*hunt)[i] also works

  client.send_proof("bottle.png", (*hunt)[0]);
}
