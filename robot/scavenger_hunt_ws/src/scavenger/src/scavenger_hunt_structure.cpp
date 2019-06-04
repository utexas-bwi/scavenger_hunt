#include "scavenger/scavenger_hunt_structure.h"

Task::Task(TaskType type,
           ProofFormat format,
           int point_value,
           std::string proof_format_description,
           std::string task_description) :
           TYPE(type),
           PROOF_FORMAT(format),
           POINT_VALUE(point_value),
           PROOF_FORMAT_DESCRIPTION(proof_format_description),
           TASK_DESCRIPTION(task_description) {}

TaskType Task::get_type() {
  return TYPE;
}

ProofFormat Task::get_proof_format() {
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

std::ostream& operator<<(std::ostream &stream, const Task &task) {
  
}
