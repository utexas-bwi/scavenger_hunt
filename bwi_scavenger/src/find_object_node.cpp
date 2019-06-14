#include <bwi_scavenger/global_topics.h>
#include <bwi_scavenger/RobotMove.h>
#include <bwi_scavenger/RobotStop.h>
#include <bwi_scavenger/robot_motion.h>
#include <bwi_scavenger/state_machine.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

using namespace scavenger_fsm;

static const state_id_t STATE_TRAVELING = 0;
static const state_id_t STATE_SCANNING = 1;
static const state_id_t STATE_INSPECTING = 2;
static const state_id_t STATE_END = 4;

static const double T_TIMEOUT = 10 * 60;
static const char TELEM_TAG[] = "[find_object_node]";

static ros::Publisher pub_move;
static ros::Publisher pub_stop;
static ros::Publisher pub_task_complete;

static int location_id = 0;

static const float INSPECT_DURATION = 8.0;
static const float INSPECT_GOOD_CONFIRMATIONS = 3;

static const double T_TURN_SLEEP = 4.0;

static StateMachine sm;

static bool proof_found = false;
static bool proof_saved = false;
static bool node_active = false;

/*------------------------------------------------------------------------------
STATE MACHINE DEFINITION
------------------------------------------------------------------------------*/

struct FindObjectSystemStateVector : SystemStateVector {
  double t = 0;
  double t_system_epoch = 0;

  bool target_seen = false;
  bool target_confirmed = false;

  bool travel_in_progress = false;
  bool travel_finished = false;

  bool inspect_finished = false;
  int inspect_confirmations = 0;
  double t_inspect_begin = 0;

  bool turn_in_progress = false;
  bool turn_finished = false;
  int turn_angle_traversed = 0;
  bool turn_sleeping = false;
  double t_turn_sleep_begin = 0;

  int destination = location_id;
} ssv;

class FindObjectEndState : public State {
public:
  FindObjectEndState(state_id_t id) : State(id) {}

  bool can_transition(State *from, SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    // Enter END because target was confirmed
    if (svec->target_confirmed) {
      ROS_INFO("%s Target was confirmed. Transitioning to END.", TELEM_TAG);
      return true;
    // Enter END because task timed out
    } else if (svec->t > T_TIMEOUT) {
      ROS_INFO("%s Task timed out. Transitioning to END.", TELEM_TAG);
      return true;
    }
    // Continue in current state otherwise
    return false;
  }

  void update(SystemStateVector *vec) {}

  void on_transition_to(SystemStateVector *vec) {
    ROS_INFO("%s Entering state END", TELEM_TAG);
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    if(svec->target_confirmed)
      proof_found = true;
    node_active = false;

    // Spin the proof subscriber
    ros::spinOnce();

    std_msgs::Bool complete_msg;
    complete_msg.data = proof_found;
    pub_task_complete.publish(complete_msg);
  }
};

// Find Object traveling state
class FindObjectTravelingState : public State {
public:
  FindObjectTravelingState(state_id_t id) : State(id) {}

  bool can_transition(State *from, SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    // Enter TRAVELING because scan failed
    if (from->get_id() == STATE_SCANNING &&
        svec->turn_finished &&
        !svec->target_seen) {
      ROS_INFO("%s Scan saw nothing. Transitioning to TRAVELING.", TELEM_TAG);
      return true;
    }
    // Continue in current state otherwise
    return false;
  }

  void update(SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    // One-time travel command send
    if (!svec->travel_in_progress) {
      svec->travel_in_progress = true;
      bwi_scavenger::RobotMove msg;
      msg.type = 0;
      msg.location = location_id % NUM_ENVIRONMENT_LOCATIONS;
      pub_move.publish(msg);
    }
  }

  void on_transition_from(SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    if (!svec->travel_finished) {
      ROS_INFO("%s Leaving TRAVELING. Suspected early stop.", TELEM_TAG);
      bwi_scavenger::RobotStop stop;
      pub_stop.publish(stop);
    } else
      location_id++;
  }

  void on_transition_to(SystemStateVector *vec) {
    ROS_INFO("%s Entering state TRAVELING", TELEM_TAG);

    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    svec->travel_in_progress = false;
    svec->travel_finished = false;
  }
};

class FindObjectScanningState : public State {
public:
  FindObjectScanningState(state_id_t id) : State(id) {}

  bool can_transition(State *from, SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    // Enter SCANNING because TRAVELING reached destination and task has not yet
    // timed out
    if (from->get_id() == STATE_TRAVELING &&
        svec->travel_finished &&
        svec->t < T_TIMEOUT) {
      ROS_INFO("%s Reached destination. Transitioning to SCANNING.", TELEM_TAG);
      return true;
    // Enter SCANNING because INSPECTING failed
    } else if (from->get_id() == STATE_INSPECTING &&
               !svec->target_confirmed &&
               svec->inspect_finished) {
      ROS_INFO("%s INSPECTING failed to confirm target. Transitioning to SCANNING.", TELEM_TAG);
      return true;
    }
    // Continue in current state otherwise
    return false;
  }

  void update(SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    // One-time turn command send
    if (!svec->turn_in_progress && !svec->turn_sleeping) {
      svec->turn_in_progress = true;
      const int TURN_AMOUNT = 45;
      bwi_scavenger::RobotMove msg;
      msg.type = 1;
      msg.degrees = TURN_AMOUNT;
      pub_move.publish(msg);
      svec->turn_angle_traversed += TURN_AMOUNT;
    }

    // Exiting sleep
    if (svec->turn_sleeping && svec->t - svec->t_turn_sleep_begin >= T_TURN_SLEEP) {
      ROS_INFO("%s SCANNING is exiting sleep.", TELEM_TAG);
      svec->turn_sleeping = false;
    }
  }

  void on_transition_from(SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    if (!svec->turn_finished) {
      ROS_INFO("%s Leaving SCANNING. Suspected early stop.", TELEM_TAG);
      bwi_scavenger::RobotStop stop;
      pub_stop.publish(stop);
    }
  }

  void on_transition_to(SystemStateVector *vec) {
    ROS_INFO("%s Entering state SCANNING", TELEM_TAG);

    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    svec->turn_finished = false;
    svec->turn_in_progress = false;
    svec->turn_angle_traversed = 0;
  }
};

// Find Object inspecting state
class FindObjectInspectingState : public State {
public:
  FindObjectInspectingState(state_id_t id) : State(id) {}

  bool can_transition(State *from, SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    // Enter INSPECTING when target is seen and current state is not END
    if (from->get_id() != STATE_END &&
        svec->target_seen) {
      ROS_INFO("%s Saw the target. Transitioning to INSPECTING.", TELEM_TAG);
      return true;
    }
    // Continue in current state otherwise
    return false;
  }

  void update(SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;

    if (svec->t - svec->t_inspect_begin >= INSPECT_DURATION) {
      svec->inspect_finished = true;
      ROS_INFO("%s Inspection got %d confirmation(s).", TELEM_TAG, svec->inspect_confirmations);
      if (svec->inspect_confirmations >= INSPECT_GOOD_CONFIRMATIONS)
        svec->target_confirmed = true;
    }
  }

  void on_transition_from(SystemStateVector *vec) {}

  void on_transition_to(SystemStateVector *vec) {
    ROS_INFO("%s Entering state INSPECTING", TELEM_TAG);

    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    svec->target_seen = false;
    svec->inspect_finished = false;
    svec->inspect_confirmations = 0;
    svec->t_inspect_begin = svec->t;
  }
};

static State *s_traveling = new FindObjectTravelingState(STATE_TRAVELING);
static State *s_scanning = new FindObjectScanningState(STATE_SCANNING);
static State *s_inspecting = new FindObjectInspectingState(STATE_INSPECTING);
static State *s_end = new FindObjectEndState(STATE_END);

void wipe_ssv() {
  proof_found = false;
  proof_saved = false;

  location_id = 0;

  ssv.t = 0;
  ssv.t_system_epoch = ros::Time::now().toSec();

  ssv.target_seen = false;
  ssv.target_confirmed = false;

  ssv.travel_in_progress = false;
  ssv.travel_finished = false;

  ssv.inspect_finished = false;
  ssv.inspect_confirmations = 0;
  ssv.t_inspect_begin = 0;

  ssv.turn_in_progress = false;
  ssv.turn_finished = false;
  ssv.turn_angle_traversed = 0;
  ssv.turn_sleeping = false;
  ssv.t_turn_sleep_begin = 0;

  ssv.destination = 0;

  sm.init(s_traveling);
}

/*------------------------------------------------------------------------------
SYSTEM STATE VECTOR UPDATE CALLBACKS
------------------------------------------------------------------------------*/

// Called when target is seen - prompts stop
void target_seen_cb(const std_msgs::Bool::ConstPtr &msg) {
  state_id_t state = sm.get_current_state()->get_id();
  if (state == STATE_SCANNING || state == STATE_TRAVELING)
    ssv.target_seen = true;
  else if (!ssv.inspect_finished)
    ssv.inspect_confirmations++;
}

// Called when move finished - prompts state transition
void move_finished_cb(const std_msgs::Bool::ConstPtr &msg) {
  if (ssv.travel_in_progress && !ssv.travel_finished)
    ssv.travel_finished = true;
  if (ssv.turn_in_progress) {
    if (ssv.turn_angle_traversed >= 360)
      ssv.turn_finished = true;
    else {
      ssv.turn_in_progress = false;
      ROS_INFO("%s SCANNING has entered sleep.", TELEM_TAG);
      ssv.turn_sleeping = true;
      ssv.t_turn_sleep_begin = ssv.t;
    }
  }
}

// Called when image of object is recieved
void image_cb(const sensor_msgs::Image::ConstPtr &img) {
  if (proof_found && !proof_saved) {
    ROS_INFO("Saving image");
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cv::imwrite("proof.jpg", cv_ptr -> image);
    proof_saved = true;
  }
}

// Called when the main node is starting a task
void task_start_cb(const std_msgs::String::ConstPtr &msg) {
  if (msg -> data == "Find Object") {
    node_active = true;
    wipe_ssv();
    ros::Duration(5.0).sleep();
  }
}

/*------------------------------------------------------------------------------
NODE ENTRYPOINT
------------------------------------------------------------------------------*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "find_object_node");
  ros::NodeHandle nh;

  // Setup SSV update callbacks
  pub_move = nh.advertise<bwi_scavenger::RobotMove>(TPC_MOVE_NODE_GO, 1);
  pub_stop = nh.advertise<bwi_scavenger::RobotStop>(TPC_MOVE_NODE_STOP, 1);
  pub_task_complete = nh.advertise<std_msgs::Bool>(TPC_TASK_COMPLETE, 1);

  ros::Subscriber sub0 = nh.subscribe(TPC_YOLO_NODE_TARGET_SEEN, 1, target_seen_cb);
  ros::Subscriber sub1 = nh.subscribe(TPC_MOVE_NODE_FINISHED, 1, move_finished_cb);
  ros::Subscriber sub2 = nh.subscribe("/darknet_ros/detection_image", 1, image_cb);
  ros::Subscriber sub3 = nh.subscribe(TPC_MAIN_NODE_TASK_START, 1, task_start_cb);

  // Build state machine
  s_traveling->add_output(s_end);
  s_traveling->add_output(s_scanning);
  s_traveling->add_output(s_inspecting);
  s_scanning->add_output(s_traveling);
  s_scanning->add_output(s_end);
  s_scanning->add_output(s_inspecting);
  s_inspecting->add_output(s_scanning);
  s_inspecting->add_output(s_end);

  sm.add_state(s_traveling);
  sm.add_state(s_scanning);
  sm.add_state(s_inspecting);
  sm.add_state(s_end);
  sm.add_end_state(s_end);

  // Wait for ROS services to spin up
  ros::Duration(5.0).sleep();

  ROS_INFO("%s Entering Find Object state machine...", TELEM_TAG);

  // Run until state machine exit
  while (ros::ok()) {
    ros::spinOnce();

    if (!node_active)
      continue;

    sm.run(&ssv);
    ssv.t = ros::Time::now().toSec() - ssv.t_system_epoch;
  }
}
