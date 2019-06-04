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
  std::string get_name();

  /**
    @brief gets the task's preferred proof format
  */
  std::string get_proof_format();

  /**
    @brief gets the points awarded for successful completion of task
  */
  int get_point_value();

  /**
    @brief gets a written description of the proof format
  */
  std::string get_proof_format_description();

  /**
    @brief gets a written description of the task
  */
  std::string get_task_description();

  /**
    @brief gets the task's parameters; a mapping of parameter names to parameter
           values, e.g. "target object" => "soda can"
  */
  std::map<std::string, std::string> get_parameters();

  /**
    @brief adds a task parameter; the client should never have to call this!
  */
  void add_parameter(std::string param_name, std::string param_value);

  /**
    @brief gets the ID of the corresponding server-side task; the client should
           never have to call this!
  */
  int get_hunt_task_id();

  /**
    @brief pretty-prints information about this task when inserting into a
           stream, e.g. std::cout << task;
  */
  friend std::ostream& operator<<(std::ostream &stream, const Task &task);
};

/**
  A collection of tasks for your robot to complete.
*/
class ScavengerHunt {
private:
  const std::string NAME;

  std::vector<Task> tasks;

public:
  /**
    @brief creates a new hunt; the client should never have to call this!
  */
  ScavengerHunt(std::string name);

  /**
    @brief gets the name of the hunt
  */
  std::string get_name();

  /**
    @brief adds a task to the hunt; the client should never have to call this!
  */
  void add_task(Task t);

  /**
    @brief gets the number of tasks in the hunt
  */
  int size();

  /**
    @brief lets you index tasks in the hunt via bracket operator, e.g.
           Task first_task = current_hunt[0];
  */
  Task& operator[](int i);
};

#endif
