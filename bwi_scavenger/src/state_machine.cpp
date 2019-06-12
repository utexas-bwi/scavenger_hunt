#include <bwi_scavenger/state_machine.h>

using namespace scavenger_fsm;

State::State(state_id_t id) {
  this->id = id;
}

void State::add_output(State *s) {
  outputs.push_back(s);
}

state_id_t State::get_id() {
  return id;
}

State* State::attempt_transition(SystemStateVector *ssv) {
  for (State *output : outputs)
    if (output->can_transition(this, ssv))
      return output;
  return this;
}

StateMachine::~StateMachine() {
  for (State *state : states)
    delete state;
}

void StateMachine::add_state(State *s) {
  states.push_back(s);
}

void StateMachine::add_end_state(State *s) {
  end_states.push_back(s->get_id());
}

void StateMachine::add_end_state(state_id_t id) {
  end_states.push_back(id);
}

void StateMachine::init(State *initial_state) {
  current_state = initial_state;
}

void StateMachine::init(state_id_t initial_state_id) {
  current_state = get_state(initial_state_id);
}

bool StateMachine::run(SystemStateVector *ssv) {
  if (iterations == 0)
    current_state->on_transition_to(ssv);
  iterations++;

  current_state->update(ssv);
  State *current_state_old = current_state;
  current_state = current_state->attempt_transition(ssv);

  if (current_state_old != current_state &&
      current_state_old->get_id() != current_state->get_id()) {
    current_state_old->on_transition_from(ssv);
    current_state->on_transition_to(ssv);
  }

  for (state_id_t id : end_states)
    if (current_state->get_id() == id)
      return true;

  return false;
}

State* StateMachine::get_state(state_id_t id) {
  for (State *state : states)
    if (state->get_id() == id)
      return state;
  return nullptr;
}

State* StateMachine::get_current_state() {
  return current_state;
}
