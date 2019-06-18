#ifndef SCAVENGER_HUNT_STRUCTURE_H
#define SCAVENGER_HUNT_STRUCTURE_H

#include <iostream>
#include <map>
#include <vector>

/**
  A singular task, belonging to some hunt.
*/
class Task {
private:
  const std::string NAME;
  const std::string TASK_DESCRIPTION;
  const std::string PROOF_FORMAT;
  const std::string PROOF_FORMAT_DESCRIPTION;
  const int POINT_VALUE;
  const int HUNT_TASK_ID;

  std::map<std::string, std::string> parameters;

public:
  /**
    @brief creates a new task; the client should never have to call this!
  */
  Task(std::string name,
       std::string task_description,
       std::string format,
       std::string proof_format_description,
       int point_value,
       int hunt_task_id);

  /**
    @brief gets the task's type
  */
  std::string get_name() const;

  /**
    @brief gets the task's preferred proof format
  */
  std::string get_proof_format() const;

  /**
    @brief gets the points awarded for successful completion of task
  */
  int get_point_value() const;

  /**
    @brief gets a written description of the proof format
  */
  std::string get_proof_format_description() const;

  /**
    @brief gets a written description of the task
  */
  std::string get_description() const;

  /**
    @brief gets the task's parameters; a mapping of parameter names to parameter
           values, e.g. "target object" => "soda can"
  */
  std::map<std::string, std::string> get_parameters() const;

  /**
    @brief gets the value of a parameter whose name is known beforehand
  */
  std::string get_parameter_value(std::string param);

  /**
    @brief adds a task parameter; the client should never have to call this!
  */
  void add_parameter(std::string param_name, std::string param_value);

  /**
    @brief gets the ID of the corresponding server-side task; the client should
           never have to call this!
  */
  int get_hunt_task_id() const;

  /**
    @brief pretty-prints information about this task when inserting into a
           stream, e.g. std::cout << task;
  */
  friend std::ostream& operator<<(std::ostream &stream, const Task &task);
};

#endif
