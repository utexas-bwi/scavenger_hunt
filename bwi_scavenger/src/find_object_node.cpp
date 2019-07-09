#include <cv_bridge/cv_bridge.h>
#include <kinect_fusion/kinect_fusion.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include <bwi_scavenger_msgs/DatabaseProof.h>
#include <bwi_scavenger_msgs/DatabaseInfoSrv.h>
#include <bwi_scavenger_msgs/RobotMove.h>
#include <bwi_scavenger_msgs/RobotStop.h>
#include <bwi_scavenger_msgs/PerceptionMoment.h>
#include <bwi_scavenger_msgs/PoseRequest.h>
#include <bwi_scavenger_msgs/TaskEnd.h>
#include <bwi_scavenger_msgs/TaskStart.h>

#include <darknet_ros_msgs/BoundingBox.h>

#include <geometry_msgs/Pose.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include "bwi_scavenger/globals.h"
#include "bwi_scavenger/mapping.h"
#include "bwi_scavenger/robot_motion.h"
#include "bwi_scavenger/state_machine.h"

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
static ros::Publisher pub_get_info;

static environment_location current_location;

static const float INSPECT_DURATION = 8.0;
static const float INSPECT_GOOD_CONFIRMATIONS = 3;
static const double T_TURN_SLEEP = 6.0;

static StateMachine sm;

static bool node_active = false;

static OrderedLocationSet map;
static PriorityLocationSet prioritized_map;

static std::string target_object;

static bwi_scavenger_msgs::PerceptionMoment last_perception_moment;

static geometry_msgs::Pose target_pose;

static ros::ServiceClient client_pose_request;
static ros::ServiceClient client_database_info_request;

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
  bool prioritized_finished = false;
  // bool travel_do_not_disturb = true;

  bool inspect_finished = false;
  int inspect_confirmations = 0;
  double t_inspect_begin = 0;

  bool turn_in_progress = false;
  bool turn_finished = false;
  int turn_angle_traversed = 0;
  bool turn_sleeping = false;
  double t_turn_sleep_begin = 0;

  coordinate destination;
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
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(last_perception_moment.color_image,
                                   sensor_msgs::image_encodings::BGR8);
      cv::imwrite(PROOF_MATERIAL_PATH, cv_ptr -> image);
    }

    // Fetch most recent robot pose
    bwi_scavenger_msgs::PoseRequest req;
    client_pose_request.call(req);
    geometry_msgs::Pose robot_pose = req.response.pose;

    // Build success msg
    bwi_scavenger_msgs::TaskEnd end_msg;
    end_msg.robot_pose = robot_pose;
    end_msg.secondary_pose = target_pose;
    end_msg.success = svec->target_confirmed;
    pub_task_complete.publish(end_msg);
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
      // svec->travel_do_not_disturb = prioritized_map.get_laps() < 1;
      bwi_scavenger_msgs::RobotMove msg;
      msg.type = 0;
      coordinate dest = svec->destination;
      msg.location.push_back(dest.first);
      msg.location.push_back(dest.second);
      pub_move.publish(msg);
    }
  }

  void on_transition_from(SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;

    if(!svec->prioritized_finished){ //only call get_laps if needed
      if(prioritized_map.get_laps() >= 1){
        svec->prioritized_finished = true;
      }
    }
    if (!svec->travel_finished) {
      bwi_scavenger_msgs::RobotStop stop;
      pub_stop.publish(stop);
    } else
      if(!svec->prioritized_finished){
        svec->destination = prioritized_map.get_next_location();
      }
      else{
        svec->destination = map.get_next_location();
      }
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
               svec->inspect_finished)
      return true;
    // Continue in current state otherwise
    return false;
  }

  void update(SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    // One-time turn command send
    if (!svec->turn_in_progress && !svec->turn_sleeping) {
      svec->turn_in_progress = true;
      const int TURN_AMOUNT = 45;
      bwi_scavenger_msgs::RobotMove msg;
      msg.type = 1;
      msg.degrees = TURN_AMOUNT;
      pub_move.publish(msg);
      svec->turn_angle_traversed += TURN_AMOUNT;
    }

    // Exiting sleep
    if (svec->turn_sleeping &&
        svec->t - svec->t_turn_sleep_begin >= T_TURN_SLEEP) {
      svec->turn_sleeping = false;
    }
  }

  void on_transition_from(SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    if (!svec->turn_finished) {
      bwi_scavenger_msgs::RobotStop stop;
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
    // Enter INSPECTING when target is seen and current state is not END and currently not travelling to a prioritized location
    if (from->get_id() != STATE_END &&
        svec->target_seen) {
          //&& !svec->travel_do_not_disturb
      ROS_INFO("%s Glimpsed the target. Transitioning to INSPECTING.", TELEM_TAG);
      return true;
    }
    // Continue in current state otherwise
    return false;
  }

  void update(SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;

    if (svec->t - svec->t_inspect_begin >= INSPECT_DURATION) {
      svec->inspect_finished = true;
      ROS_INFO("%s Inspection yielded %d confirmation(s).",
               TELEM_TAG,
               svec->inspect_confirmations);
      if (svec->inspect_confirmations >= INSPECT_GOOD_CONFIRMATIONS)
        svec->target_confirmed = true;
    }
  }

  void on_transition_from(SystemStateVector *vec) {}

  void on_transition_to(SystemStateVector *vec) {
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

  sm.init(s_traveling);
}

/*------------------------------------------------------------------------------
SYSTEM STATE VECTOR UPDATE CALLBACKS
------------------------------------------------------------------------------*/

/**
  @brief called when move_node completes a particular command
*/
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

/**
  @brief called when the main node begins a new task
*/
void task_start_cb(const bwi_scavenger_msgs::TaskStart::ConstPtr &msg) {
  if (msg->name == TASK_FIND_OBJECT) {
    wipe_ssv();
    ros::Duration(2.0).sleep();

    if (msg->parameters.size() > 0) {
      // Set the target object
      target_object = msg->parameters[0];

      ROS_INFO("%s Beginning %s protocol with parameter \"%s\".",
               TELEM_TAG, TASK_FIND_OBJECT.c_str(), target_object.c_str());

      // Query the current robot pose for planning purposes
      bwi_scavenger_msgs::PoseRequest req;
      client_pose_request.call(req);
      geometry_msgs::Pose robot_pose = req.response.pose;
      coordinate c;
      c.first = robot_pose.position.x;
      c.second = robot_pose.position.y;
      map.start(c);
      ssv.destination = map.get_next_location();

      node_active = true;
    } else
      ROS_ERROR("%s Start failed because message had no parameters.",
                TELEM_TAG);
  }
}

/**
  @brief called when perception_node publishes a new moment
*/
void perceive(const bwi_scavenger_msgs::PerceptionMoment::ConstPtr &msg) {
  if(!node_active)
    return;
  last_perception_moment = *msg;
  const darknet_ros_msgs::BoundingBoxes &boxes = msg->bounding_boxes;

  for (int i = 0; i < boxes.bounding_boxes.size(); i++) {
    const darknet_ros_msgs::BoundingBox &box = boxes.bounding_boxes[i];
    const sensor_msgs::Image &depth_image = msg->depth_image;

    if (box.Class == target_object) {
      geometry_msgs::Point target_position =
          kinect_fusion::get_position(box, depth_image);
      target_pose.position = target_position;

      bwi_scavenger_msgs::DatabaseInfoSrv req;
      req.request.task_name = TASK_FIND_OBJECT;
      req.request.parameter_name = target_object;
      req.request.data = 0;
      req.request.pose.position = kinect_fusion::get_position(box, depth_image);
      client_database_info_request.call(req);
      // ROS_INFO("[find_object_node] made call");
      if(req.response.correct){
        state_id_t state = sm.get_current_state()->get_id();
        if (state == STATE_SCANNING || state == STATE_TRAVELING)
          ssv.target_seen = true;
        else if (state == STATE_INSPECTING && !ssv.inspect_finished)
          ssv.inspect_confirmations++;
      }

      break;
    }
  }
}

// void incorrect_cb(const std_msgs::Bool::ConstPtr &msg){
//   // if it is NOT in an incorrect cluster, add to count of correct data
//   if(!(msg->data)){
//     state_id_t state = sm.get_current_state()->get_id();
//     if (state == STATE_SCANNING || state == STATE_TRAVELING)
//       ssv.target_seen = true;
//     else if (state == STATE_INSPECTING && !ssv.inspect_finished)
//       ssv.inspect_confirmations++;
//   }
// }

// void set_locations_cb(const bwi_scavenger_msgs::DatabaseLocationList::ConstPtr &msg){
//   unsigned int size = msg->size;
//   for(int i = 0; i < size / 2; i++){}
// }

/*------------------------------------------------------------------------------
NODE ENTRYPOINT
------------------------------------------------------------------------------*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "find_object_node");
  ros::NodeHandle nh;


  // Create clients
  client_pose_request = nh.serviceClient<bwi_scavenger_msgs::PoseRequest>(
      SRV_POSE_REQUEST);

  client_database_info_request = nh.serviceClient<bwi_scavenger_msgs::DatabaseInfoSrv>(
      SRV_DATABASE_INFO_REQUEST);

  // Setup SSV update callbacks
  pub_move = nh.advertise<bwi_scavenger_msgs::RobotMove>(
      TPC_MOVE_NODE_GO, 1);
  pub_stop = nh.advertise<bwi_scavenger_msgs::RobotStop>(
      TPC_MOVE_NODE_STOP, 1);
  pub_task_complete = nh.advertise<bwi_scavenger_msgs::TaskEnd>(
      TPC_TASK_END, 1);
  // pub_get_info = nh.advertise<bwi_scavenger_msgs::DatabaseInfo>(
  //     TPC_DATABASE_NODE_GET_INFO, 1);

  ros::Subscriber sub_move_finished = nh.subscribe(
      TPC_MOVE_NODE_FINISHED, 1, move_finished_cb);
  ros::Subscriber sub_task_start = nh.subscribe(
      TPC_TASK_START, 1, task_start_cb);
  ros::Subscriber sub_perception = nh.subscribe(
      TPC_PERCEPTION_NODE_MOMENT, 1, perceive);
  // ros::Subscriber sub_db_incorrect = nh.subscribe(
  //     TPC_DATABASE_NODE_INCORRECT, 1, incorrect_cb);
  // ros::Subscriber sub_prioritize_locations = nh.subscribe(
  //     TPC_DATABASE_NODE_LOCATIONS, 1, set_locations_cb);

  // Build default search circuit
  map.add_location(BWI_LAB_DOOR_NORTH);
  map.add_location(CLEARING_NORTH);
  map.add_location(CLEARING_SOUTH);
  map.add_location(ALCOVE);
  map.add_location(KITCHEN);
  map.add_location(BWI_LAB_DOOR_SOUTH);
  map.add_location(SOCCER_LAB_DOOR_SOUTH);
  map.add_location(SOCCER_LAB_DOOR_NORTH);

  // Creating prioritized location setup
  bwi_scavenger_msgs::DatabaseInfoSrv req;
  req.request.task_name = TASK_FIND_OBJECT;
  req.request.parameter_name = target_object;
  req.request.data = 1;
  client_database_info_request.call(req);

  for(int i = 0; i < req.response.location_list.size() / 2; i++){
    coordinate c = {req.response.location_list[i * 2], req.response.location_list[i * 2 + 1]};
    prioritized_map.add_location(c);
  }


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
