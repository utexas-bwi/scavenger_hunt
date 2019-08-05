#ifndef SCAVENGER_HUNT_STRUCTURE_H
#define SCAVENGER_HUNT_STRUCTURE_H

#include <iostream>
#include <map>
#include <vector>

/**
  A unique ID that robots can use to correlate local proof information with
  server-side proofs.
*/
typedef unsigned long proof_id_t;

/**
  Indicates a failed proof upload.
*/
const proof_id_t UPLOAD_FAILED = -1;

/**
  The result of querying feedback on a single proof.
*/
enum proof_status_t {
  PROOF_NOT_VALIDATED,
  PROOF_CORRECT,
  PROOF_INCORRECT
};

/**
  Information about a validated proof.
*/
class Proof {
protected:
  bool correct;
  int time_to_complete;
  std::string filename;
  proof_id_t id;

public:
  /**
    @brief creates a new proof record; the client should never have to call this!
  */
  Proof(bool correct, int time_to_complete, std::string filename,
      proof_id_t id);

  /**
    Gets if the proof was validated as correct or not.

    @return if correct
  */
  bool get_correct() const;

  /**
    Gets the time taken to complete the task.

    @return time
  */
  int get_time_to_complete() const;

  /**
    Gets the filename of the proof file.

    @return filename
  */
  std::string get_filename() const;

  /**
    @brief gets the proof's unique ID
  */
  proof_id_t get_id() const;
};

/**
  A singular task, belonging to some hunt.
*/
class Task {
private:
  const std::string NAME;
  const std::string HUNT_NAME;
  const std::string TASK_DESCRIPTION;
  const std::string PROOF_FORMAT;
  const std::string PROOF_FORMAT_DESCRIPTION;
  const int POINT_VALUE;
  const int HUNT_TASK_ID;

  std::map<std::string, std::string> parameters;
  std::vector<Proof> proofs;

public:
  /**
    @brief creates a new task; the client should never have to call this!
  */
  Task(std::string name,
       std::string hunt_name,
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
    @brief gets the name of the hunt this task belongs to
  */
  std::string get_hunt_name() const;

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
