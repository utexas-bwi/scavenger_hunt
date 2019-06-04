#include <fstream>
#include <iostream>
#include <scavenger_hunt/rapidxml.hpp>
#include <scavenger_hunt/scavenger_hunt.h>
#include <string>
#include <vector>

using namespace rapidxml;

int main(int argc, char** argv) {
  /*Task t("Find Object",
         "Locate an object by name.",
         "Image",
         "Submit an image of the object with a box drawn around it.",
         100,
         101);
  t.add_parameter("target object", "soda can");
  std::cout << t;

  ScavengerHunt hunt("Longhorn Hunt");
  hunt.add_task(t);
  std::cout << hunt.get_name() << " has " << hunt.size() << " tasks" << std::endl;
  std::cout << hunt[0];

  ScavengerHuntClient client("stefandebruyn@utexas.edu", "sick robots");
  client.send_proof("bottle.png", hunt[0]);*/

  /*xml_document<> doc;
  xml_node<> *root_node;

  std::ifstream fstr("hunt.xml");
  std::vector<char> buffer((std::istreambuf_iterator<char>(fstr)), std::istreambuf_iterator<char>());
  buffer.push_back('\0');

  doc.parse<0>(&buffer[0]);
  root_node = doc.first_node("hunt");
  std::string hunt_name = root_node->first_attribute("name")->value();
  ScavengerHunt hunt(hunt_name);

  for (xml_node<> *task_node = root_node->first_node("task"); task_node; task_node = task_node->next_sibling()) {
    // Parse task fields
    std::string name = std::string(task_node->first_attribute("name")->value());
    std::string description = std::string(task_node->first_attribute("description")->value());
    std::string proof_format = std::string(task_node->first_attribute("proof_format")->value());
    std::string proof_description = std::string(task_node->first_attribute("proof_description")->value());
    int points = std::stoi(std::string(task_node->first_attribute("points")->value()));
    int id = std::stoi(std::string(task_node->first_attribute("id")->value()));

    Task task(name, description, proof_format, proof_description, points, id);

    // Parse task parameters
    for (xml_node<> *param_node = task_node->first_node("parameter"); param_node; param_node = param_node->next_sibling()) {
      std::string param_name = std::string(param_node->first_attribute("name")->value());
      std::string param_value = std::string(param_node->first_attribute("value")->value());
      task.add_parameter(param_name, param_value);
    }

    hunt.add_task(task);
  }

  for (int i = 0; i < hunt.size(); i++)
    std::cout << hunt[i];*/

  ScavengerHuntClient client("stefandebruyn@utexas.edu", "sick robots");
  ScavengerHunt *hunt = client.get_hunt("Longhorn Hunt");

  for (int i = 0; i < hunt->size(); i++)
    std::cout << (*hunt)[i];
}
