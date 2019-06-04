#ifndef SCAVENGER_HUNT_STRUCTURE_H
#define SCAVENGER_HUNT_STRUCTURE_H

#include <iostream>
#include <map>

enum TaskType {
  FIND_OBJECT,
  VISIT_PLACE,
  GREET_PERSON,
  TAKE_A_STROLL
};

enum ProofFormat {
  IMAGE,
  VIDEO
};

class Task {
protected:
  const TaskType TYPE;
  const ProofFormat PROOF_FORMAT;
  const int POINT_VALUE;
  const std::string TASK_NAME;
  const std::string PROOF_FORMAT_DESCRIPTION;
  const std::string TASK_DESCRIPTION;

  std::map<std::string, std::string> parameters;

public:
  Task(TaskType type,
       ProofFormat format,
       int point_value,
       std::string proof_format_description,
       std::string task_description);

   TaskType get_type();

   ProofFormat get_proof_format();

   int get_point_value();

   std::string get_proof_format_description();

   std::string get_task_description();

   std::map<std::string, std::string> get_parameters();

   void add_parameter(std::string param_name, std::string param_value);

   friend std::ostream& operator<<(std::ostream &stream, const Task &task);
};

#endif
