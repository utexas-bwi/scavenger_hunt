#include <bwi_scavenger_msgs/TaskEnd.h>

#include <scavenger_hunt_msgs/Hunt.h>
#include <scavenger_hunt_msgs/GetHunt.h>
#include <scavenger_hunt_msgs/Task.h>

#include <ros/ros.h>

#include "bwi_scavenger/globals.h"

static ros::Publisher pub_task_start;
static ros::ServiceClient client_database_info_request;
static ros::ServiceClient client_get_hunt;
static ros::ServiceClient client_send_proof;

static std::size_t task_index = 0;
static bool conclude = false;

static scavenger_hunt_msgs::Hunt hunt;

/**
 * Advances to the next task. On the first call, the hunt begins.
 *
 * @param msg end message broadcast by task node or nullptr if initial task
 */
void next_task(const bwi_scavenger_msgs::TaskEnd::ConstPtr &msg) {
  // Terminate
  if (conclude) {
    ROS_INFO("[main_node] Shutting down.");
    exit(0);
  }

  if (msg == nullptr)
    ROS_INFO("[main_node] Starting hunt.");
  else
    task_index++;

  if (task_index < hunt.tasks.size()) {
    scavenger_hunt_msgs::Task& current_task = hunt.tasks[task_index];
    pub_task_start.publish(current_task);
  } else {
    ROS_INFO("[main_node] Hunt complete.");
    ros::Duration(6.0).sleep();

    // Begin Conclude
    scavenger_hunt_msgs::Task task;
    task.name = TASK_CONCLUDE;
    pub_task_start.publish(task);
    conclude = true;
    return;
  }
}

int main(int argc, char **argv) {
  if (argc < 2) {
    ROS_ERROR("Usage: do_hunt <hunt name>");
    exit(0);
  }

  ros::init(argc, argv, "hunt_node");
  ros::NodeHandle nh;
  pub_task_start = nh.advertise<scavenger_hunt_msgs::Task>(TPC_TASK_START, 1);
  ros::Subscriber sub_task_complete = nh.subscribe(TPC_TASK_END, 1, next_task);

  client_get_hunt = nh.serviceClient<scavenger_hunt_msgs::GetHunt>(
    "/scavenger_hunt/get_hunt"
  );

  scavenger_hunt_msgs::GetHunt get_hunt;
  get_hunt.request.hunt_name = std::string(argv[1]);
  client_get_hunt.call(get_hunt);
  hunt = get_hunt.response.hunt;

  ros::Duration(3.0).sleep();
  next_task(nullptr);
  ros::spin();
}
