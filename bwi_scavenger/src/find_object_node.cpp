#include <ros/ros.h>
#include <bwi_scavenger/state_machine.h>
#include <std_msgs/Bool.h>
#include <bwi_scavenger/scavenger_move.h>

using namespace taskmaster;

static state_id_t STATE_TRAVELING = 0;
static state_id_t STATE_SCANNING = 1;
static state_id_t STATE_END = 2;

static const double T_TIMEOUT = 10;

static ros::Publisher pub_move;

static int location_id = 0;

struct FindObjectStateVector : SystemStateVector {
  double t;
  bool target_seen;
  bool move_finished;
  bool move_in_progress;
  int destination;
} ssv;

class FindObjectEndState : public State {
public:
  FindObjectEndState(state_id_t id) : State(id) {}

  bool can_transition(State *from, SystemStateVector *vec) {
    FindObjectStateVector *svec = (FindObjectStateVector*)vec;
    if (svec->target_seen || // Object was found
        from->get_id() == STATE_TRAVELING && svec->t > T_TIMEOUT) // Timed out
      return true;
    // Continue in current state otherwise
    return false;
  }

  void update(SystemStateVector *vec) {}

  void on_transition_to(SystemStateVector *vec) {
    ROS_INFO("Entering state END");
  }
};

class FindObjectTravelingState : public State {
public:
  FindObjectTravelingState(state_id_t id) : State(id) {}

  bool can_transition(State *from, SystemStateVector *vec) {
    FindObjectStateVector *svec = (FindObjectStateVector*)vec;
    // Scanning; object was not found
    if (from->get_id() == STATE_SCANNING &&
        svec->move_finished &&
        !svec->target_seen)
      return true;
    // Continue in current state otherwise
    return false;
  }

  void update(SystemStateVector *vec) {
    FindObjectStateVector *svec = (FindObjectStateVector*)vec;
    if (!svec->move_in_progress) {
      svec->move_in_progress = true;
      bwi_scavenger::scavenger_move msg;
      msg.type = 0;
      msg.location = location_id++ % 7;
      pub_move.publish(msg);
    }
  }

  void on_transition_from(SystemStateVector *vec) {
    FindObjectStateVector *svec = (FindObjectStateVector*)vec;
    svec->move_finished = false;
    svec->move_in_progress = false;
  }

  void on_transition_to(SystemStateVector *vec) {
    ROS_INFO("Entering state TRAVELING");
  }
};

class FindObjectScanningState : public State {
public:
  FindObjectScanningState(state_id_t id) : State(id) {}

  bool can_transition(State *from, SystemStateVector *vec) {
    FindObjectStateVector *svec = (FindObjectStateVector*)vec;
    // Traveling; destination was reached and the task has not timed out
    if (from->get_id() == STATE_TRAVELING &&
        svec->move_finished &&
        svec->t < T_TIMEOUT)
      return true;
    // Continue in current state otherwise
    return false;
  }

  void update(SystemStateVector *vec) {
    FindObjectStateVector *svec = (FindObjectStateVector*)vec;
    if (!svec->move_in_progress) {
      svec->move_in_progress = true;
      bwi_scavenger::scavenger_move msg;
      msg.type = 1;
      msg.degrees = 360;
      pub_move.publish(msg);
    }
  }

  void on_transition_from(SystemStateVector *vec) {
    FindObjectStateVector *svec = (FindObjectStateVector*)vec;
    svec->move_finished = false;
    svec->move_in_progress = false;
  }

  void on_transition_to(SystemStateVector *vec) {
    ROS_INFO("Entering state SCANNING");
  }
};

void target_seen_cb(const std_msgs::Bool::ConstPtr &msg) {
  ssv.target_seen = true;
}

void move_finished_cb(const std_msgs::Bool::ConstPtr &msg) {
  ssv.move_finished = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "find_object_node");
  ros::NodeHandle nh;

  pub_move = nh.advertise<bwi_scavenger::scavenger_move>("scavenger/move", 1);

  ros::Subscriber sub0 = nh.subscribe("scavenger/target_seen", 1, target_seen_cb);
  ros::Subscriber sub1 = nh.subscribe("scavenger/move_finished", 1, move_finished_cb);

  ssv.t = 0;
  ssv.target_seen = false;
  ssv.move_finished = false;
  ssv.move_in_progress = false;
  ssv.destination = 0;

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

  bool state_machine_active = true;

  while (ros::ok() && state_machine_active) {
    state_machine_active = !sm.run(&ssv);
    ssv.t = ros::Time::now().toSec();
    ros::spinOnce();
  }
}
