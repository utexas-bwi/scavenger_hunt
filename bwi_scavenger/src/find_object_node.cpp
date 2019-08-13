#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <kinect_fusion/kinect_fusion.h>
#include <map>
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <random>
#include <ros/ros.h>
#include <scavenger_hunt/scavenger_hunt.h>
#include <tf/tf.h>
#include <vector>

#include <bwi_scavenger_msgs/PerceptionMoment.h>
#include <bwi_scavenger_msgs/PoseRequest.h>
#include <bwi_scavenger_msgs/RobotMove.h>
#include <bwi_scavenger_msgs/RobotStop.h>
#include <bwi_scavenger_msgs/SendProof.h>
#include <bwi_scavenger_msgs/TaskEnd.h>
#include <bwi_scavenger_msgs/ConfirmObject.h>
#include <bwi_scavenger_msgs/GetPriorityPoints.h>
#include <bwi_scavenger_msgs/MultitaskStart.h>
#include <scavenger_hunt_msgs/Task.h>
#include <scavenger_hunt_msgs/Proof.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>

#include "bwi_scavenger/globals.h"
#include "bwi_scavenger/paths.h"
#include "bwi_scavenger/path_planning.h"
#include "bwi_scavenger/state_machine.h"
#include "bwi_scavenger/world_mapping.h"

// The strategy employed by the object search
enum SearchStrategy {
  CIRCUIT,
  GREEDY,
  SMART
};

static const SearchStrategy SEARCH_STRATEGY = GREEDY;
static const bool RANDOMIZE_CIRCUIT = true;

// IDs for state machine states
static const state_id_t STATE_TRAVELING = 0;
static const state_id_t STATE_SCANNING = 1;
static const state_id_t STATE_INSPECTING = 2;
static const state_id_t STATE_END = 4;

// Parameters for search and inspect behavior
static const double T_TIMEOUT = 0; // 0 for no timeout
static const double T_TURN_SLEEP = 15.0;
static const double MIN_INSPECT_PROBABILITY = 0;
static const double INSPECT_DURATION = 10.0;
static const int INSPECT_GOOD_CONFIRMATIONS = 2;

// Roslog node identifying tag
static const char TELEM_TAG[] = "[find_object_node]";

// Publishers
static ros::Publisher pub_move;
static ros::Publisher pub_stop;
static ros::Publisher pub_task_complete;

// State machine
static StateMachine sm;

// Flags for node activity
static double t_task_start;
static bool node_active = false;

// Pathing
static OrderedLocationSet* map_circuit;
static OrderedLocationSet* map_priority;
static LocationSet* map_active;

// Target object metadata
static std::vector<std::string> target_object_labels;
static std::string target_label_current;
static std::map<std::string, int> target_object_confirmations;
static std::map<std::string, bwi_scavenger_msgs::PerceptionMoment> target_object_perceptions;
static std::map<std::string, scavenger_hunt_msgs::Task> target_object_tasks;
static std::map<std::string, darknet_ros_msgs::BoundingBox> target_object_bboxes;

// Service clients
static ros::ServiceClient client_pose_request;
static ros::ServiceClient client_send_proof;
static ros::ServiceClient client_confirm_object;
static ros::ServiceClient client_get_priority_points;

/**
 * Called when a target object is confirmed. Relies on perceive_cb(...) to
 * populate the target metadata maps with things like bounding box, RGBD images,
 * etc. Once the proof is sent, the target label is removed from the list of
 * active labels.
 *
 * @param task task to prove
 */
void send_proof(const scavenger_hunt_msgs::Task& task) {
  // Retrieve metadata from maps
  std::string label = task.parameters[0].value;
  bwi_scavenger_msgs::PerceptionMoment perception =
      target_object_perceptions[label];
  darknet_ros_msgs::BoundingBox bbox = target_object_bboxes[label];

  ROS_INFO("%s Sending proof for %s...", TELEM_TAG, label.c_str());

  // Package up the proof information
  scavenger_hunt_msgs::Proof proof;
  proof.image = perception.color_image;
  proof.task_duration = ros::Time::now().toSec() - t_task_start;

  // Attempt resolving the object's position
  geometry_msgs::Point tobj_rel_position =
      kinect_fusion::get_position(bbox, perception.depth_image);

  bwi_scavenger_msgs::PoseRequest pose_req;
  client_pose_request.call(pose_req);

  tf::Quaternion tf_quat(pose_req.response.pose.orientation.x,
                         pose_req.response.pose.orientation.y,
                         pose_req.response.pose.orientation.z,
                         pose_req.response.pose.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

  geometry_msgs::Point target_object_position;
  target_object_position.x = pose_req.response.pose.position.x +
                             cos(yaw) * tobj_rel_position.x -
                             sin(yaw) * tobj_rel_position.y;
  target_object_position.y = pose_req.response.pose.position.y +
                             sin(yaw) * tobj_rel_position.x +
                             cos(yaw) * tobj_rel_position.y;
  target_object_position.z = pose_req.response.pose.position.z +
                             tobj_rel_position.z;

  // Package up the metadata and submit
  bwi_scavenger_msgs::SendProof send_proof;
  send_proof.request.proof = proof;
  send_proof.request.task = task;
  send_proof.request.robot_position = pose_req.response.pose.position;
  send_proof.request.secondary_position = target_object_position;
  send_proof.request.metadata =
      std::to_string(bbox.xmin) + "," +
      std::to_string(bbox.xmax) + "," +
      std::to_string(bbox.ymin) + "," +
      std::to_string(bbox.ymax);

  client_send_proof.call(send_proof);

  // Remove object from list to find
  std::vector<std::string>::iterator it = std::find(
    target_object_labels.begin(), target_object_labels.end(), label
  );
  target_object_labels.erase(it);

  ROS_INFO("%s Things I'm still looking for:", TELEM_TAG);

  for (const std::string& str : target_object_labels)
    ROS_INFO("%s %s", TELEM_TAG, str);
}

/*******************************************************************************
 * STATE MACHINE DEFINITION
 ******************************************************************************/

struct FindObjectSystemStateVector : SystemStateVector {
  double t = 0;                    // Current time
  double t_system_epoch = 0;       // Time of node initiation

  bool target_seen = false;        // If a

  bool travel_in_progress = false; // If a TRAVELING is in progress
  bool travel_finished = false;    // If TRAVELING reached its destination

  bool inspect_finished = false;   // If INSPECTING finished
  double t_inspect_begin = 0;      // Time of transition to INSPECTING

  bool turn_in_progress = false;   // If SCANNING is currently turning
  bool turn_finished = false;      // If SCANNING finished turning
  int turn_angle_traversed = 0;    // Total arc turned by SCANNING so far
  bool turn_sleeping = false;      // If SCANNING is sleeping the turn cycle
  double t_turn_sleep_begin = 0;   // Time of sleep cycle begin

  coordinates_t destination;       // Last set destination
} ssv;

/**
 * The terminal state. All tasks have been attempted. The node enters
 * hibernation and relinquishes control of the robot.
 */
class FindObjectEndState : public State {
public:
  FindObjectEndState(state_id_t id) : State(id) {}

  bool can_transition(State *from, SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    // Enter END because target was confirmed
    if (target_object_labels.size() == 0) {
      ROS_INFO("%s All tasks completed or aborted. Transitioning to END.", TELEM_TAG);
      return true;
    // Enter END because task timed out
    } else if (T_TIMEOUT > 0 && svec->t > T_TIMEOUT) {
      ROS_INFO("%s Tasks timed out. Transitioning to END.", TELEM_TAG);
      return true;
    }
    // Continue in current state otherwise
    return false;
  }

  void update(SystemStateVector *vec) {}

  void on_transition_to(SystemStateVector *vec) {
    // Signal main node that the task has concluded
    bwi_scavenger_msgs::TaskEnd end_msg;
    pub_task_complete.publish(end_msg);
    node_active = false;
  }
};

/**
 * State in which the robot is navigating between two world coordinates. This
 * may be interrupted if a target is seen.
 */
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
    } else if (from->get_id() == STATE_INSPECTING &&
               svec->inspect_finished)
      return true;
    // Continue in current state otherwise
    return false;
  }

  void update(SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    // One-time travel command send
    if (!svec->travel_in_progress) {
      svec->travel_in_progress = true;
      bwi_scavenger_msgs::RobotMove msg;
      msg.type = bwi_scavenger_msgs::RobotMove::MOVE;
      coordinates_t dest = svec->destination;
      msg.location.push_back(dest.x);
      msg.location.push_back(dest.y);
      pub_move.publish(msg);
    }
  }

  void on_transition_from(SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;

    // Transition to circuit map if the priority map has finished
    if (map_active == map_priority && map_active->get_laps() >= 1) {
      ROS_INFO("%s Changing maps...", TELEM_TAG);

      map_active = map_circuit;
      bwi_scavenger_msgs::PoseRequest req_pose;
      client_pose_request.call(req_pose);
      coordinates_t c = {
        req_pose.response.pose.position.x,
        req_pose.response.pose.position.y
      };
      map_active->start(c);
    }

    if (!svec->travel_finished) {
      bwi_scavenger_msgs::RobotStop stop;
      pub_stop.publish(stop);
    } else {
      // Reshuffle circuit on every travel state when using random search
      if (RANDOMIZE_CIRCUIT)
        map_circuit->shuffle();

      svec->destination = map_active->get_next_location();
    }
  }

  void on_transition_to(SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    svec->travel_in_progress = false;
    svec->travel_finished = false;
  }
};

/**
 * State in which the robot is spinning in place and looking for targets.
 * Interrupted with a transition to inspecting if a target is seen.
 */
class FindObjectScanningState : public State {
public:
  FindObjectScanningState(state_id_t id) : State(id) {}

  bool can_transition(State *from, SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    // Enter SCANNING because TRAVELING reached destination and task has not yet
    // timed out
    if (from->get_id() == STATE_TRAVELING &&
        svec->travel_finished &&
        (T_TIMEOUT == 0 || svec->t < T_TIMEOUT)) {
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
      bwi_scavenger_msgs::RobotMove msg;
      msg.type = bwi_scavenger_msgs::RobotMove::TURN;
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

/**
 * State in which the robot sits motionless and staring at a target, looking for
 * repeated confirmations.
 */
class FindObjectInspectingState : public State {
public:
  FindObjectInspectingState(state_id_t id) : State(id) {}

  bool can_transition(State *from, SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    // Enter INSPECTING when target is seen and current state is not END and currently not travelling to a prioritized location
    if (from->get_id() != STATE_END &&
        svec->target_seen) {
      ROS_INFO("%s Glimpsed the target. Transitioning to INSPECTING.", TELEM_TAG);
      return true;
    }
    // Continue in current state otherwise
    return false;
  }

  void update(SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;

    if (svec->inspect_finished)
      return;

    if (svec->t - svec->t_inspect_begin >= INSPECT_DURATION) {
      for (const auto& pair : target_object_confirmations) {
        ROS_INFO("%s \"%s\" got %d hits",
          TELEM_TAG, pair.first.c_str(), pair.second
        );

        if (pair.second > INSPECT_GOOD_CONFIRMATIONS)
          send_proof(target_object_tasks[pair.first]);

        target_object_confirmations[pair.first] = 0;
      }

      svec->inspect_finished = true;
    }
  }

  void on_transition_from(SystemStateVector *vec) {}

  void on_transition_to(SystemStateVector *vec) {
    FindObjectSystemStateVector *svec = (FindObjectSystemStateVector*)vec;
    svec->target_seen = false;
    svec->inspect_finished = false;
    svec->t_inspect_begin = svec->t;
  }
};

static State *s_traveling = new FindObjectTravelingState(STATE_TRAVELING);
static State *s_scanning = new FindObjectScanningState(STATE_SCANNING);
static State *s_inspecting = new FindObjectInspectingState(STATE_INSPECTING);
static State *s_end = new FindObjectEndState(STATE_END);

/**
 * @brief resets the state machine system state vector
 */
void wipe_ssv() {
  ssv.t = 0;
  ssv.t_system_epoch = ros::Time::now().toSec();

  ssv.target_seen = false;

  ssv.travel_in_progress = false;
  ssv.travel_finished = false;

  ssv.inspect_finished = false;
  ssv.t_inspect_begin = 0;

  ssv.turn_in_progress = false;
  ssv.turn_finished = false;
  ssv.turn_angle_traversed = 0;
  ssv.turn_sleeping = false;
  ssv.t_turn_sleep_begin = 0;

  sm.init(s_traveling);
}

/*******************************************************************************
 * SSV UPDATE CALLBACKS
 ******************************************************************************/

/**
 * @brief Called when move_node completes a particular command.
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
 * @brief Called when the main node initiates a single task. Currently disabled
 *        while we conduct experiments involving multitasking.
 */
void task_start_cb(const scavenger_hunt_msgs::Task& msg) {
  // if (msg.name != TASK_FIND_OBJECT)
  //   return;
  //
  // wipe_ssv();
  //
  // task_of_interest = msg;
  // target_object = msg.parameters[0].value;
  //
  // // Get priority point for this task
  // bwi_scavenger_msgs::GetPriorityPoints get_points;
  // get_points.request.task_name = TASK_FIND_OBJECT;
  // get_points.request.task_parameter = target_object;
  // client_get_priority_points.call(get_points);
  //
  // if (get_points.response.points.size() > 0) {
  //   ROS_INFO("%s Received priority locations list with %d points.",
  //     TELEM_TAG, get_points.response.points.size()
  //   );
  //
  //   for (geometry_msgs::Point& point : get_points.response.points)
  //     map_priority->add_location({point.x, point.y});
  //
  //   map_active = map_priority;
  // } else
  //   map_active = map_circuit;
  //
  // // Query the current robot pose so we can find the closest waypoint
  // bwi_scavenger_msgs::PoseRequest reqPose;
  // client_pose_request.call(reqPose);
  // geometry_msgs::Pose robot_pose = reqPose.response.pose;
  // coordinates_t c;
  // c.x = robot_pose.position.x;
  // c.y = robot_pose.position.y;
  //
  // map_active->start(c);
  // ssv.destination = map_active->get_next_location();
  // node_active = true;
  // t_task_start = ros::Time::now().toSec();
}

/**
 * @brief Called when the main node initiates a multitask. Any find object tasks
 *        will be taken up by this node. WARNING: do not mix multitasks or nodes
 *        will fight for control of the robot.
 */
void multitask_start_cb(const bwi_scavenger_msgs::MultitaskStart& msg) {
  ROS_INFO("%s Received multitask request.", TELEM_TAG);

  // Reset the system state
  wipe_ssv();

  // Temporary util for sorting priority points
  typedef struct PriorityPoint {
    coordinates_t coords;
    float score;
    bool operator<(const PriorityPoint& rhs) const {
      return score > rhs.score;
    }
  } priority_point_t;

  std::vector<priority_point_t> priority_points;

  // Collect target labels
  for (const scavenger_hunt_msgs::Task& task : msg.tasks) {
    if (task.name == TASK_FIND_OBJECT) {
      // Log this label to be searched for
      std::string target_label = task.parameters[0].value;

      ROS_INFO("%s Will search for %s", TELEM_TAG, target_label.c_str());

      target_object_labels.push_back(target_label);
      target_object_tasks[target_label] = task;
      target_object_confirmations[target_label] = 0;

      // Get priority points for this label
      bwi_scavenger_msgs::GetPriorityPoints get_points;
      get_points.request.task_name = TASK_FIND_OBJECT;
      get_points.request.task_parameter = target_label;
      client_get_priority_points.call(get_points);

      for (std::size_t i = 0; i < get_points.response.points.size(); i++) {
        // Collapse similiar points
        const float SIMILARITY_THRESH = 0.25;
        bool new_point = true;

        for (std::size_t j = 0; j < priority_points.size(); j++) {
          float dx = priority_points[j].coords.x -
                     get_points.response.points[i].x;
          float dy = priority_points[j].coords.y -
                     get_points.response.points[i].y;
          float dist = sqrt(dx * dx + dy * dy);

          if (dist < SIMILARITY_THRESH) {
            new_point = false;
            break;
          }
        }

        if (new_point) {
          priority_points.push_back(
            {
              {
                get_points.response.points[i].x,
                get_points.response.points[i].y
              },
              get_points.response.scores[i]
            }
          );
        }
      }
    } else
      ROS_ERROR("%s Cannot multitask: %s", TELEM_TAG, task.name.c_str());
  }

  // Default to counter-clockwise circuit
  map_active = map_circuit;

  if (RANDOMIZE_CIRCUIT)
    map_circuit->shuffle();

  if (SEARCH_STRATEGY == CIRCUIT) {
    // Already configured
  } else if (SEARCH_STRATEGY == GREEDY) {
    // Obtain and sort priority points for all currently assigned tasks
    if (priority_points.size() > 0) {
      // Sort priority point list
      std::sort(priority_points.begin(), priority_points.end());

      ROS_INFO("%s Built priority map with %d points:",
        TELEM_TAG, priority_points.size()
      );

      for (const priority_point_t& ppoint : priority_points) {
        map_priority->add_location(ppoint.coords);
        ROS_INFO("%s (%f, %f) with score=%f",
          TELEM_TAG, ppoint.coords.x, ppoint.coords.y, ppoint.score
        );
      }

      // Begin the search with the priority circuit
      map_active = map_priority;
    }
  } else if (SEARCH_STRATEGY == SMART) {
    // TODO: figure this out
    ROS_ERROR("wtfffff");
  }

  // Localize in the map and begin searching
  bwi_scavenger_msgs::PoseRequest pose_req;
  client_pose_request.call(pose_req);

  geometry_msgs::Pose robot_pose = pose_req.response.pose;

  coordinates_t c;
  c.x = robot_pose.position.x;
  c.y = robot_pose.position.y;

  map_active->start(c);

  ssv.destination = map_active->get_next_location();
  node_active = true;
  t_task_start = ros::Time::now().toSec();
}

/**
 * @brief Called when perception_node publishes a new moment.
 */
void perceive_cb(const bwi_scavenger_msgs::PerceptionMoment::ConstPtr &msg) {
  if (!node_active)
    return;

  const darknet_ros_msgs::BoundingBoxes &boxes = msg->bounding_boxes;
  state_id_t state = sm.get_current_state()->get_id();

  // Search the bounding boxes for one of our target labels
  for (int i = 0; i < boxes.bounding_boxes.size(); i++) {
    const darknet_ros_msgs::BoundingBox &box = boxes.bounding_boxes[i];
    const sensor_msgs::Image &depth_image = msg->depth_image;
    bool target_identified = std::find(
      target_object_labels.begin(),
      target_object_labels.end(),
      box.Class
    ) != target_object_labels.end();

    // A match was found
    if (target_identified) {
      // The current state is interruptable; initiate inspection
      if (state == STATE_SCANNING || state == STATE_TRAVELING) {
        ssv.target_seen = true;
        target_label_current = box.Class;
      // We're already inspecting; update target confirmations
      } else if (state == STATE_INSPECTING) {
        target_object_confirmations[box.Class]++;
        target_object_perceptions[box.Class] = *msg;
        target_object_bboxes[box.Class] = box;
      }
    }
  }
}

/*******************************************************************************
 * NODE ENTRYPOINT
 ******************************************************************************/

int main(int argc, char **argv) {
  ros::init(argc, argv, "find_object_node");
  ros::NodeHandle nh;

  // Determine world
  std::string world_str;
  World world;
  nh.param("bwi_scavenger/world", world_str, std::string("WORLDNOTFOUND"));

  if (world_str == "irl")
    world = IRL;
  else if (world_str == "sim")
    world = SIM;
  else
    ROS_ERROR("Unknown world: %s", world_str.c_str());

  // Node build up
  client_pose_request = nh.serviceClient<bwi_scavenger_msgs::PoseRequest>(SRV_POSE_REQUEST);
  client_send_proof = nh.serviceClient<bwi_scavenger_msgs::SendProof>(SRV_SEND_PROOF);
  client_confirm_object = nh.serviceClient<bwi_scavenger_msgs::ConfirmObject>(SRV_CONFIRM_OBJECT);
  client_get_priority_points = nh.serviceClient<bwi_scavenger_msgs::GetPriorityPoints>(SRV_GET_PRIORITY_POINTS);

  pub_move = nh.advertise<bwi_scavenger_msgs::RobotMove>(TPC_MOVE_NODE_GO, 1);
  pub_stop = nh.advertise<bwi_scavenger_msgs::RobotStop>(TPC_MOVE_NODE_STOP, 1);
  pub_task_complete = nh.advertise<bwi_scavenger_msgs::TaskEnd>(TPC_TASK_END, 1);

  ros::Subscriber sub_move_finished = nh.subscribe(TPC_MOVE_NODE_FINISHED, 1, move_finished_cb);
  ros::Subscriber sub_task_start = nh.subscribe(TPC_TASK_START, 1, task_start_cb);
  ros::Subscriber sub_perception = nh.subscribe(TPC_PERCEPTION_NODE_MOMENT, 1, perceive_cb);
  ros::Subscriber sub_multitask_start = nh.subscribe(TPC_MULTITASK_START, 1, multitask_start_cb);

  // Assemble maps
  map_circuit = new OrderedLocationSet(world);
  map_priority = new OrderedLocationSet();

  // Build default search circuit
  map_circuit->add_location(BWI_LAB_DOOR_NORTH);
  map_circuit->add_location(CLEARING_NORTH);
  map_circuit->add_location(CLEARING_SOUTH);
  // map_circuit->add_location(ALCOVE);
  map_circuit->add_location(KITCHEN);
  // map_circuit->add_location(BWI_LAB_DOOR_SOUTH);
  map_circuit->add_location(HALLWAY0);
  // map_circuit->add_location(SOCCER_LAB_DOOR_SOUTH);
  // map_circuit->add_location(SOCCER_LAB_DOOR_NORTH);

  // Build state machine
  s_traveling->add_output(s_end);
  s_traveling->add_output(s_scanning);
  s_traveling->add_output(s_inspecting);

  s_scanning->add_output(s_traveling);
  s_scanning->add_output(s_end);
  s_scanning->add_output(s_inspecting);

  s_inspecting->add_output(s_scanning);
  s_inspecting->add_output(s_end);
  s_inspecting->add_output(s_traveling);

  sm.add_state(s_traveling);
  sm.add_state(s_scanning);
  sm.add_state(s_inspecting);
  sm.add_state(s_end);
  sm.add_end_state(s_end);

  // Wait a bit for ROS services to spin up
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
