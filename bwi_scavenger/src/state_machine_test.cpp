#include <iostream>
#include <bwi_scavenger/state_machine.h>
#include <chrono>

using namespace std;
using namespace chrono;
using namespace taskmaster;

state_id_t STATE_TRAVELING = 0;
state_id_t STATE_SCANNING = 1;
state_id_t STATE_END = 2;

const double T_TIMEOUT = 2;

double time() {
	chrono::milliseconds ms =
      duration_cast<chrono::milliseconds>(system_clock::now().time_since_epoch());
	return ms.count() / 1000.0;
}

struct FindObjectStateVector : SystemStateVector {
  double t;
  bool target_seen;
  bool scan_finished;
  bool destination_reached;
};

class FindObjectEndState : public State {
public:
  FindObjectEndState(state_id_t id) : State(id) {}

  bool can_transition(State *from, SystemStateVector *vec) {
    FindObjectStateVector *svec = (FindObjectStateVector*)vec;
    // Traveling; time has run out
    if (from->get_id() == STATE_TRAVELING && svec->t > T_TIMEOUT)
      return true;
    // Scanning; object was found
    else if (from->get_id() == STATE_SCANNING && svec->target_seen)
      return true;
    // Continue in current state otherwise
    return false;
  }

  void update(SystemStateVector *vec) {}

  void on_transition_from(SystemStateVector *vec) {
    cout << "Leaving " << this->get_id() << endl;

    FindObjectStateVector *svec = (FindObjectStateVector*)vec;
    svec->target_seen = false;
    svec->scan_finished = false;
  }

  void on_transition_to(SystemStateVector *vec) {
    cout << "Entering " << this->get_id() << endl;
  }
};

class FindObjectTravelingState : public State {
public:
  FindObjectTravelingState(state_id_t id) : State(id) {}

  bool can_transition(State *from, SystemStateVector *vec) {
    FindObjectStateVector *svec = (FindObjectStateVector*)vec;
    // Scanning; object was not found
    if (from->get_id() == STATE_SCANNING &&
        svec->scan_finished &&
        !svec->target_seen)
      return true;
    // Continue in current state otherwise
    return false;
  }

  void update(SystemStateVector *vec) {
    FindObjectStateVector *svec = (FindObjectStateVector*)vec;
    svec->destination_reached = true;
  }

  void on_transition_from(SystemStateVector *vec) {
    cout << "Leaving " << this->get_id() << endl;

    FindObjectStateVector *svec = (FindObjectStateVector*)vec;
    svec->destination_reached = false;
  }

  void on_transition_to(SystemStateVector *vec) {
    cout << "Entering " << this->get_id() << endl;
  }
};

class FindObjectScanningState : public State {
public:
  FindObjectScanningState(state_id_t id) : State(id) {}

  bool can_transition(State *from, SystemStateVector *vec) {
    FindObjectStateVector *svec = (FindObjectStateVector*)vec;
    // Traveling; destination was reached and the task has not timed out
    if (from->get_id() == STATE_TRAVELING &&
        svec->destination_reached &&
        svec->t < T_TIMEOUT)
      return true;
    // Continue in current state otherwise
    return false;
  }

  void update(SystemStateVector *vec) {
    FindObjectStateVector *svec = (FindObjectStateVector*)vec;
    svec->scan_finished = true;
    svec->target_seen = true;
  }

  void on_transition_from(SystemStateVector *vec) {
    cout << "Leaving " << this->get_id() << endl;
  }

  void on_transition_to(SystemStateVector *vec) {
    cout << "Entering " << this->get_id() << endl;
  }
};

int main() {
  FindObjectStateVector vec;
  vec.t = 0;
  vec.target_seen = false;
  vec.scan_finished = false;
  vec.destination_reached = false;

  State *s_traveling = new FindObjectTravelingState(STATE_TRAVELING);
  State *s_scanning = new FindObjectScanningState(STATE_SCANNING);
  State *s_end = new FindObjectEndState(STATE_END);

  s_traveling->add_output(s_end);
  s_traveling->add_output(s_scanning);
  s_scanning->add_output(s_traveling);
  s_scanning->add_output(s_end);

  StateMachine sm;
  sm.add_state(s_traveling);
  sm.add_state(s_scanning);
  sm.add_state(s_end);
  sm.add_end_state(s_end);
  sm.init(s_traveling);

  cout << "Entering state machine..." << endl;

  double t0 = time();

  while (!sm.run(&vec)) {
    vec.t = time() - t0;
    // cout << sm.get_current_state()->get_id() << endl;
  }

  cout << "State machine concluded." << endl;
}
