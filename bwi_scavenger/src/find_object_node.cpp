#include <bwi_scavenger/global_topics.h>
#include <bwi_scavenger/RobotMove.h>
#include <bwi_scavenger/RobotStop.h>
#include <bwi_scavenger/mapping.h>
#include <bwi_scavenger/robot_motion.h>
#include <bwi_scavenger/state_machine.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <darknet_ros_msgs/BoundingBox.h>

using namespace scavenger_fsm;

static const state_id_t STATE_TRAVELING = 0;
static const state_id_t STATE_SCANNING = 1;
static const state_id_t STATE_INSPECTING = 2;
static const state_id_t STATE_END = 4;

static const double T_TIMEOUT = 10 * 60;
static const double MIN_INSPECT_PROBABILITY = 0;
static const char TELEM_TAG[] = "[find_object_node]";

static ros::Publisher pub_move;
static ros::Publisher pub_stop;
static ros::Publisher pub_task_complete;
static ros::Publisher pub_pose_request;

static environment_location current_location;

static const float INSPECT_DURATION = 8.0;
static const float INSPECT_GOOD_CONFIRMATIONS = 3;

static const double T_TURN_SLEEP = 6.0;

static StateMachine sm;

static bool node_active = false;
static bool pose_request_sent = false;

static sensor_msgs::Image::ConstPtr last_darknet_img;

static OrderedLocationSet map;

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

  environment_location destination = BWI_LAB_LEFT;
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
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;

    if(svec->target_confirmed) {
      ROS_INFO("%s Saving proof locally...", TELEM_TAG);
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(last_darknet_img, sensor_msgs::image_encodings::BGR8);
      cv::imwrite("proof.jpg", cv_ptr -> image);
    }

    std_msgs::Bool complete_msg;
    complete_msg.data = svec->target_confirmed;
    pub_task_complete.publish(complete_msg);
    node_active = false;
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
      msg.location = svec->destination;
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
      svec->destination = map.get_next_location();
  }

  void on_transition_to(SystemStateVector *vec) {
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

  ssv.destination = BWI_LAB_LEFT;

  sm.init(s_traveling);
}

/*------------------------------------------------------------------------------
SYSTEM STATE VECTOR UPDATE CALLBACKS
------------------------------------------------------------------------------*/

// Called when target is seen - prompts stop
void target_seen_cb(const darknet_ros_msgs::BoundingBox::ConstPtr &msg) {
  if(msg->probability > MIN_INSPECT_PROBABILITY){
    state_id_t state = sm.get_current_state()->get_id();
    if (state == STATE_SCANNING || state == STATE_TRAVELING)
      ssv.target_seen = true;
    else if (!ssv.inspect_finished)
      ssv.inspect_confirmations++;
  }
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
      ssv.turn_sleeping = true;
      ssv.t_turn_sleep_begin = ssv.t;
    }
  }
}

// Called when image of object is recieved
void image_cb(const sensor_msgs::Image::ConstPtr &img) {
  last_darknet_img = img;
}

// Called when the main node is starting a task
void task_start_cb(const std_msgs::String::ConstPtr &msg) {
  if (msg->data == "Find Object") {
    wipe_ssv();
    ros::Duration(5.0).sleep();

    // Request current robot pose from move_node to inform mapping
    std_msgs::Bool msg;
    pub_pose_request.publish(msg);
    pose_request_sent = true;
  }
}

// Called when the move node computes the closest waypoint
void pose_request_cb(const geometry_msgs::Pose::ConstPtr &msg) {
  map.start(msg->position.x, msg->position.y);
  ssv.destination = map.get_next_location();
  ROS_INFO("%s Pose request fulfilled. Circuit will begin at location %d.",
      TELEM_TAG, (int)(ssv.destination));
  if (pose_request_sent) {
    pose_request_sent = false;
    node_active = true;
  }
}

/*------------------------------------------------------------------------------
NODE ENTRYPOINT
------------------------------------------------------------------------------*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "find_object_node");
  ros::NodeHandle nh;

  // Build search circuit
  map.add_location(BWI_LAB_RIGHT);
  map.add_location(CLEARING_RIGHT);
  map.add_location(CLEARING_LEFT);
  map.add_location(ALCOVE);
  map.add_location(KITCHEN);
  map.add_location(BWI_LAB_LEFT);
  map.add_location(LARG_RIGHT);
  map.add_location(LARG_LEFT);

  // Setup SSV update callbacks
  pub_move = nh.advertise<bwi_scavenger::RobotMove>(TPC_MOVE_NODE_GO, 1);
  pub_stop = nh.advertise<bwi_scavenger::RobotStop>(TPC_MOVE_NODE_STOP, 1);
  pub_task_complete = nh.advertise<std_msgs::Bool>(TPC_TASK_COMPLETE, 1);
  pub_pose_request = nh.advertise<std_msgs::Bool>(TPC_MOVE_NODE_REQUEST_POSE, 1);

  ros::Subscriber sub0 = nh.subscribe(TPC_YOLO_NODE_TARGET_SEEN, 1, target_seen_cb);
  ros::Subscriber sub1 = nh.subscribe(TPC_MOVE_NODE_FINISHED, 1, move_finished_cb);
  ros::Subscriber sub2 = nh.subscribe("/darknet_ros/detection_image", 1, image_cb);
  ros::Subscriber sub3 = nh.subscribe(TPC_MAIN_NODE_TASK_START, 1, task_start_cb);
  ros::Subscriber sub4 = nh.subscribe(TPC_MOVE_NODE_ROBOT_POSE, 1, pose_request_cb);

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

  ROS_INFO("%s Standing by.", TELEM_TAG);

  // Run until state machine exit
  while (ros::ok()) {
    ros::spinOnce();

    if (!node_active)
      continue;

    sm.run(&ssv);
    ssv.t = ros::Time::now().toSec() - ssv.t_system_epoch;
  }
}
