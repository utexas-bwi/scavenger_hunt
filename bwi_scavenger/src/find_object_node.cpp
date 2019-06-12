#include <bwi_scavenger/global_topics.h>
#include <bwi_scavenger/RobotMove.h>
#include <bwi_scavenger/RobotStop.h>
#include <bwi_scavenger/state_machine.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "opencv2/opencv.hpp"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>

using namespace scavenger_fsm;

static const state_id_t STATE_TRAVELING = 0;
static const state_id_t STATE_SCANNING = 1;
static const state_id_t STATE_END = 2;

static const double T_TIMEOUT = 10 * 60;
static const char TELEM_TAG[] = "[find_object_node]";

static ros::Publisher pub_move;
static ros::Publisher pub_stop;

static int location_id = 0;

/*------------------------------------------------------------------------------
STATE MACHINE DEFINITION
------------------------------------------------------------------------------*/

struct FindObjectSystemStateVector : SystemStateVector {
  double t = 0;
  bool target_seen = false;
  bool move_finished = false;
  bool move_in_progress = false;
  int destination = location_id;
} ssv;

class FindObjectEndState : public State {
public:
  FindObjectEndState(state_id_t id) : State(id) {}

  bool can_transition(State *from, SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    // Enter END because target was seen
    if (svec->target_seen) {
      ROS_INFO("%s Target was seen. Transitioning to END.", TELEM_TAG);
      return true;
    // Enter END because task timed out
    } else if (from->get_id() == STATE_TRAVELING && svec->t > T_TIMEOUT) {
      ROS_INFO("%s Task timed out. Transitioning to END.", TELEM_TAG);
      return true;
    }
    // Continue in current state otherwise
    return false;
  }

  void update(SystemStateVector *vec) {}

  void on_transition_to(SystemStateVector *vec) {
    ROS_INFO("%s Entering state END", TELEM_TAG);
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
        svec->move_finished &&
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
    if (!svec->move_in_progress) {
      svec->move_in_progress = true;
      bwi_scavenger::RobotMove msg;
      msg.type = 0;
      msg.location = location_id % 7;
      location_id++;
      pub_move.publish(msg);
    }
  }

  void on_transition_from(SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    svec->move_finished = false;
    svec->move_in_progress = false;
  }

  void on_transition_to(SystemStateVector *vec) {
    ROS_INFO("%s Entering state TRAVELING", TELEM_TAG);
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
        svec->move_finished &&
        svec->t < T_TIMEOUT) {
      ROS_INFO("%s Reached destination. Transitioning to SCANNING.", TELEM_TAG);
      return true;
    }
    // Continue in current state otherwise
    return false;
  }

  void update(SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    // One-time turn command send
    if (!svec->move_in_progress) {
      svec->move_in_progress = true;
      bwi_scavenger::RobotMove msg;
      msg.type = 1;
      msg.degrees = 360;
      pub_move.publish(msg);
    }
  }

  void on_transition_from(SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    svec->move_finished = false;
    svec->move_in_progress = false;
  }

  void on_transition_to(SystemStateVector *vec) {
    ROS_INFO("%s Entering state SCANNING", TELEM_TAG);
  }
};

/*------------------------------------------------------------------------------
SYSTEM STATE VECTOR UPDATE CALLBACKS
------------------------------------------------------------------------------*/

// Called when target is seen - prompts stop
void target_seen_cb(const std_msgs::Bool::ConstPtr &msg) {
  ssv.target_seen = true;
  bwi_scavenger::RobotStop stop;
  pub_stop.publish(stop);
}

// Called when move finished - prompts state transition
void move_finished_cb(const std_msgs::Bool::ConstPtr &msg) {
  ssv.move_finished = true;
}

// Called when image of object is recieved
void image_cb(const sensor_msgs::Image::ConstPtr &img){
  ROS_INFO("Saving image");
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  // gives unique name to image as defined by its header sequence
  std::string name = std::to_string(img->header.seq);
  cv::imwrite(name + ".jpg", cv_ptr -> image);
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

  ros::Subscriber sub0 = nh.subscribe(TPC_YOLO_NODE_TARGET_SEEN, 1, target_seen_cb);
  ros::Subscriber sub1 = nh.subscribe(TPC_MOVE_NODE_FINISHED, 1, move_finished_cb);
  ros::Subscriber sub2 = nh.subscribe(TPC_YOLO_NODE_TARGET_IMAGE, 1, image_cb);

  // Build state machine
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

  // Wait for ROS services to spin up
  ros::Duration(1.0).sleep();

  bool state_machine_active = true;
  double org = ros::Time::now().toSec();

  ROS_INFO("%s Entering Find Object state machine...", TELEM_TAG);

  // Run until state machine exit
  while (ros::ok() || state_machine_active) {
    state_machine_active = !sm.run(&ssv);
    ssv.t = ros::Time::now().toSec() - org;
    ros::spinOnce();
  }
}
