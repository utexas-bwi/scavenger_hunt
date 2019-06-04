#include "scavenger_hunt/scavenger_hunt_structure.h"

Task::Task(std::string name,
           std::string task_description,
           std::string format,
           std::string proof_format_description,
           int point_value,
           int hunt_task_id) :
           NAME(name),
           TASK_DESCRIPTION(task_description),
           PROOF_FORMAT(format),
           PROOF_FORMAT_DESCRIPTION(proof_format_description),
           POINT_VALUE(point_value),
           HUNT_TASK_ID(hunt_task_id) {}

std::string Task::get_name() {
  return NAME;
}

std::string Task::get_proof_format() {
  return PROOF_FORMAT;
}

int Task::get_point_value() {
  return POINT_VALUE;
}

std::string Task::get_proof_format_description() {
  return PROOF_FORMAT_DESCRIPTION;
}

std::string Task::get_task_description() {
  return TASK_DESCRIPTION;
}

std::map<std::string, std::string> Task::get_parameters() {
  return parameters;
}

void Task::add_parameter(std::string param_name, std::string param_value) {
  parameters[param_name] = param_value;
}

int Task::get_hunt_task_id() {
  return HUNT_TASK_ID;
}

std::ostream& operator<<(std::ostream &stream, const Task &task) {
  stream << "Task \"" << task.NAME << "\": {\n"
         << "\t" << "Description: " << task.TASK_DESCRIPTION << "\n"
         << "\t" << "Proof format: " << task.PROOF_FORMAT << "\n"
         << "\t" << "Proof description: " << task.PROOF_FORMAT_DESCRIPTION << "\n"
         << "\t" << "Points: " << task.POINT_VALUE << "\n"
         << "\t" << "Parameters: {\n";

  // Format task parameters
  for (const auto &pair : task.parameters) {
    stream << "\t\t" << pair.first << ": " << pair.second << "\n";
  }

  stream << "\t}\n}\n";
}

ScavengerHunt::ScavengerHunt(std::string name) : NAME(name) {}

std::string ScavengerHunt::get_name() {
  return NAME;
}

void ScavengerHunt::add_task(Task t) {
  tasks.push_back(t);
}

Task ScavengerHunt::get_task(unsigned int index) {
  return tasks[index];
}

unsigned int ScavengerHunt::size() {
  return tasks.size();
}

Task& ScavengerHunt::operator[](unsigned int i) {
  return tasks[i];
}
