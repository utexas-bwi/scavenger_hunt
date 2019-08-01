#include <cv_bridge/cv_bridge.h>
#include <kinect_fusion/kinect_fusion.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <scavenger_hunt/scavenger_hunt.h>

#include <bwi_scavenger_msgs/PerceptionMoment.h>
#include <bwi_scavenger_msgs/PoseRequest.h>
#include <bwi_scavenger_msgs/RobotMove.h>
#include <bwi_scavenger_msgs/RobotStop.h>
#include <bwi_scavenger_msgs/SendProof.h>
#include <bwi_scavenger_msgs/TaskEnd.h>
#include <bwi_scavenger_msgs/ConfirmObject.h>
#include <bwi_scavenger_msgs/GetPriorityPoints.h>

#include <scavenger_hunt_msgs/Task.h>
#include <scavenger_hunt_msgs/Proof.h>

#include <darknet_ros_msgs/BoundingBox.h>

#include <geometry_msgs/Pose.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Bool.h>

#include "bwi_scavenger/globals.h"
#include "bwi_scavenger/mapping.h"
#include "bwi_scavenger/paths.h"
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

static environment_location current_location;

static const float INSPECT_DURATION = 8.0;
static const float INSPECT_GOOD_CONFIRMATIONS = 2;
static const double T_TURN_SLEEP = 6.0;

static StateMachine sm;

static double t_task_start;
static bool node_active = false;

static OrderedLocationSet map_circuit;
static OrderedLocationSet map_priority;
static LocationSet* map_active;

static std::string target_object;

static sensor_msgs::Image target_object_image;
static darknet_ros_msgs::BoundingBox target_object_bbox;
static geometry_msgs::Point target_object_position;

static scavenger_hunt_msgs::Task task_of_interest;

static ros::ServiceClient client_pose_request;
static ros::ServiceClient client_send_proof;
static ros::ServiceClient client_confirm_object;
static ros::ServiceClient client_get_priority_points;

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

  coordinates destination;
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

    if (svec->target_confirmed) {
      ROS_INFO("%s Preparing to send proof...", TELEM_TAG);

      // Package up the proof information and send it to the website
      scavenger_hunt_msgs::Proof proof;
      proof.image = target_object_image;
      proof.task_duration = ros::Time::now().toSec() - t_task_start;

      bwi_scavenger_msgs::PoseRequest pose_req;
      client_pose_request.call(pose_req);

      bwi_scavenger_msgs::SendProof send_proof;
      send_proof.request.proof = proof;
      send_proof.request.task = task_of_interest;
      send_proof.request.robot_position = pose_req.response.pose.position;
      send_proof.request.secondary_position = target_object_position;
      send_proof.request.metadata =
          std::to_string(target_object_bbox.xmin) + "," +
          std::to_string(target_object_bbox.xmax) + "," +
          std::to_string(target_object_bbox.ymin) + "," +
          std::to_string(target_object_bbox.ymax);

      client_send_proof.call(send_proof);
    }

    // Signal main node that the task has concluded
    bwi_scavenger_msgs::TaskEnd end_msg;
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
      bwi_scavenger_msgs::RobotMove msg;
      msg.type = 0;
      coordinates dest = svec->destination;
      msg.location.push_back(dest.x);
      msg.location.push_back(dest.y);
      pub_move.publish(msg);
    }
  }

  void on_transition_from(SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;

    // Transition to circuit map if the priority map has finished
    if (map_active == &map_priority && map_active->get_laps() >= 1) {
      ROS_INFO("%s Changing maps...", TELEM_TAG);

      map_active = &map_circuit;
      bwi_scavenger_msgs::PoseRequest req_pose;
      client_pose_request.call(req_pose);
      coordinates c = {
        req_pose.response.pose.position.x,
        req_pose.response.pose.position.y
      };
      map_active->start(c);
    }

    if (!svec->travel_finished) {
      bwi_scavenger_msgs::RobotStop stop;
      pub_stop.publish(stop);
    } else
        svec->destination = map_active->get_next_location();
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
      if (from->get_id() == STATE_TRAVELING && map_active == &map_priority){
        svec->target_seen = false;
        ROS_INFO("%s Priority map has blocked INSPECTING!", TELEM_TAG);
        return false;
      }
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
      if (svec->inspect_confirmations >= INSPECT_GOOD_CONFIRMATIONS) {
        // Confirm object identity thru objmem
        bwi_scavenger_msgs::ConfirmObject confirm_object;
        confirm_object.request.label = target_object;
        confirm_object.request.position = target_object_position;
        client_confirm_object.call(confirm_object);

        if (confirm_object.response.ok)
          svec->target_confirmed = true;
        else
          ROS_INFO("%s objmem claims the target to be incorrect. Ignoring...", TELEM_TAG);
      }
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
void task_start_cb(const scavenger_hunt_msgs::Task& msg) {
  if (msg.name != TASK_FIND_OBJECT)
    return;

  wipe_ssv();

  task_of_interest = msg;
  target_object = msg.parameters[0].value;

  ROS_INFO("%s Beginning task solution with parameter \"%s\".",
           TELEM_TAG, target_object.c_str());

  // Get priority point for this task
  bwi_scavenger_msgs::GetPriorityPoints get_points;
  get_points.request.task_name = TASK_FIND_OBJECT;
  get_points.request.task_parameter = target_object;
  client_get_priority_points.call(get_points);

  if (get_points.response.points.size() > 0) {
    ROS_INFO("%s Received priority locations list with %d points.",
      TELEM_TAG, get_points.response.points.size()
    );

    for (geometry_msgs::Point& point : get_points.response.points)
      map_priority.add_location({point.x, point.y});

    map_active = &map_priority;
  } else
    map_active = &map_circuit;

  // Query the current robot pose so we can find the closest waypoint
  bwi_scavenger_msgs::PoseRequest reqPose;
  client_pose_request.call(reqPose);
  geometry_msgs::Pose robot_pose = reqPose.response.pose;
  coordinates c;
  c.x = robot_pose.position.x;
  c.y = robot_pose.position.y;

  map_active->start(c);
  ssv.destination = map_active->get_next_location();
  node_active = true;
  t_task_start = ros::Time::now().toSec();
}

/**
  @brief called when perception_node publishes a new moment
*/
void perceive(const bwi_scavenger_msgs::PerceptionMoment::ConstPtr &msg) {
  if (!node_active)
    return;

  const darknet_ros_msgs::BoundingBoxes &boxes = msg->bounding_boxes;

  for (int i = 0; i < boxes.bounding_boxes.size(); i++) {
    const darknet_ros_msgs::BoundingBox &box = boxes.bounding_boxes[i];
    const sensor_msgs::Image &depth_image = msg->depth_image;

    if (box.Class == target_object) {
      target_object_bbox = box;
      target_object_image = msg->color_image;
      target_object_position = kinect_fusion::get_position(box, depth_image);

      bwi_scavenger_msgs::PoseRequest req_pose;
      client_pose_request.call(req_pose);

      tf::Quaternion tf_quat(req_pose.response.pose.orientation.x,
                             req_pose.response.pose.orientation.y,
                             req_pose.response.pose.orientation.z,
                             req_pose.response.pose.orientation.w);
      double roll, pitch, yaw;
      tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

      target_object_position.x = req_pose.response.pose.position.x +
                                 cos(yaw) * target_object_position.x -
                                 sin(yaw) * target_object_position.y;
      target_object_position.y = req_pose.response.pose.position.y +
                                 sin(yaw) * target_object_position.x +
                                 cos(yaw) * target_object_position.y;
      target_object_position.z = req_pose.response.pose.position.z +
                                 target_object_position.z;

      state_id_t state = sm.get_current_state()->get_id();

      if (state == STATE_SCANNING || state == STATE_TRAVELING)
        ssv.target_seen = true;
      else if (state == STATE_INSPECTING && !ssv.inspect_finished)
        ssv.inspect_confirmations++;

      break;
    }
  }
}

/*------------------------------------------------------------------------------
NODE ENTRYPOINT
------------------------------------------------------------------------------*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "find_object_node");
  ros::NodeHandle nh;

  // Create clients
  client_pose_request = nh.serviceClient<bwi_scavenger_msgs::PoseRequest>(
    SRV_POSE_REQUEST
  );
  client_send_proof = nh.serviceClient<bwi_scavenger_msgs::SendProof>(
    SRV_SEND_PROOF
  );
  client_confirm_object = nh.serviceClient<bwi_scavenger_msgs::ConfirmObject>(
    SRV_CONFIRM_OBJECT
  );
  client_get_priority_points = nh.serviceClient<bwi_scavenger_msgs::GetPriorityPoints>(
    SRV_GET_PRIORITY_POINTS
  );

  // Setup SSV update callbacks
  pub_move = nh.advertise<bwi_scavenger_msgs::RobotMove>(
    TPC_MOVE_NODE_GO,
    1
  );
  pub_stop = nh.advertise<bwi_scavenger_msgs::RobotStop>(
    TPC_MOVE_NODE_STOP,
    1
  );
  pub_task_complete = nh.advertise<bwi_scavenger_msgs::TaskEnd>(
    TPC_TASK_END,
    1
  );

  ros::Subscriber sub_move_finished = nh.subscribe(
    TPC_MOVE_NODE_FINISHED,
    1,
    move_finished_cb
  );
  ros::Subscriber sub_task_start = nh.subscribe(
    TPC_TASK_START,
    1,
    task_start_cb
  );
  ros::Subscriber sub_perception = nh.subscribe(
    TPC_PERCEPTION_NODE_MOMENT,
    1,
    perceive
  );

  // Build default search circuit
  map_circuit.add_location(BWI_LAB_DOOR_NORTH);
  map_circuit.add_location(CLEARING_NORTH);
  map_circuit.add_location(CLEARING_SOUTH);
  map_circuit.add_location(ALCOVE);
  map_circuit.add_location(KITCHEN);
  map_circuit.add_location(BWI_LAB_DOOR_SOUTH);
  map_circuit.add_location(SOCCER_LAB_DOOR_SOUTH);
  map_circuit.add_location(SOCCER_LAB_DOOR_NORTH);

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
