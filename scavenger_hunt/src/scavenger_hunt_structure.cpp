#include <scavenger_hunt/scavenger_hunt_structure.h>

Proof::Proof(bool correct, int time_to_complete, std::string filename) {
  this->correct = correct;
  this->time_to_complete = time_to_complete;
  this->filename = filename;
}

bool Proof::get_correct() const {
  return correct;
}

int Proof::get_time_to_complete() const {
  return time_to_complete;
}

std::string Proof::get_filename() const {
  return filename;
}

Task::Task(std::string name,
           std::string hunt_name,
           std::string task_description,
           std::string format,
           std::string proof_format_description,
           int point_value,
           int hunt_task_id) :
           NAME(name),
           HUNT_NAME(hunt_name),
           TASK_DESCRIPTION(task_description),
           PROOF_FORMAT(format),
           PROOF_FORMAT_DESCRIPTION(proof_format_description),
           POINT_VALUE(point_value),
           HUNT_TASK_ID(hunt_task_id) {}

std::string Task::get_name() const {
  return NAME;
}

std::string Task::get_hunt_name() const {
  return HUNT_NAME;
}

std::string Task::get_proof_format() const {
  return PROOF_FORMAT;
}

int Task::get_point_value() const {
  return POINT_VALUE;
}

std::string Task::get_proof_format_description() const {
  return PROOF_FORMAT_DESCRIPTION;
}

std::string Task::get_description() const {
  return TASK_DESCRIPTION;
}

std::map<std::string, std::string> Task::get_parameters() const {
  return parameters;
}

std::string Task::get_parameter_value(std::string param) {
  return parameters[param];
}

void Task::add_parameter(std::string param_name, std::string param_value) {
  parameters[param_name] = param_value;
}

int Task::get_hunt_task_id() const {
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
  for (const auto &pair : task.parameters)
    stream << "\t\t" << pair.first << ": " << pair.second << "\n";

  stream << "\t}\n}\n";
}
