/**
  Finite state machine framework used to define task behavior.
*/
#ifndef BWI_SCAVENGER_STATE_MACHINE_H
#define BWI_SCAVENGER_STATE_MACHINE_H

#include <vector>

namespace scavenger_fsm {

/**
  A vector detailing the complete state of the system, usually the robot.
  This should be extended and defined for the robot in question. Machine states
  update this vector while they run and also use it to evaluate transition
  conditions.
*/
struct SystemStateVector {};

/**
  A unique ID for a machine state.
*/
typedef unsigned int state_id_t;

/**
  A machine state. Defined by the states it can transition to, the conditions
  governing those transitions, and the events that occur when the system is in
  this state and when the machine transitions to and from it.

  This class should be extended for each unique machine state.
*/
class State {
protected:
  std::vector<State*> outputs;
  state_id_t id;

public:
  /**
    @brief creates a new state with some unique ID
  */
  State(state_id_t id);

  /**
    Gets if some state can transition to me.

    @param from state attempting transition to me
    @param ssv current system state
    @return if transition can occur
  */
  virtual bool can_transition(State *from, SystemStateVector *ssv) = 0;

  /**
    An iterative method called when the system is in this state. Should update
    the system state accordingly.

    @param ssv system state to be updated
  */
  virtual void update(SystemStateVector *ssv) = 0;

  /**
    @brief called when the machine transitions from this state
  */
  virtual void on_transition_from(SystemStateVector *ssv) {}

  /**
    @brief called when the machine transitions to this state
  */
  virtual void on_transition_to(SystemStateVector *ssv) {}

  /**
    @brief adds an output; a state that this one can potentially transition to
  */
  void add_output(State *s);

  /**
    @brief gets my unique ID
  */
  state_id_t get_id();

  /**
    Attempts a state transition based on the current system state.

    @param ssv system state
    @return state transition to; either me, or one of my outputs
  */
  State* attempt_transition(SystemStateVector *ssv);
};

/**
  A collection of states with an entry point and one or more exit points.
*/
class StateMachine {
protected:
  std::vector<State*> states;
  std::vector<state_id_t> end_states;
  State *current_state;
  int iterations = 0;

public:
  /**
    @brief tears down the machine by deleting all added states
  */
  ~StateMachine();

  /**
    @brief adds a new state to the machine
  */
  void add_state(State *s);

  /**
    @brief adds a state that constitutes exit from the machine
  */
  void add_end_state(State *s);

  /**
    @brief adds an end state by ID
  */
  void add_end_state(state_id_t id);

  /**
    @brief sets the initial machine state
  */
  void init(State *initial_state);

  /**
    @brief sets the initial state by ID
  */
  void init(state_id_t initial_state_id);

  /**
    Performs a single iteration of the machine. The current state is run and
    transitions are attempted.

    @param ssv system state; will be updated by states in the machine as
               they run
    @return if the machine has ended (if the current state is an end state)
  */
  bool run(SystemStateVector *ssv);

  /**
    @brief fetches a sate by ID
  */
  State* get_state(state_id_t id);

  /**
    @brief fetches the current state
  */
  State* get_current_state();
};

}; // end namespace taskmaster

#endif
