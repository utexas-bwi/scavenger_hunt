#ifndef DATABASE_NODE_H
#define DATABASE_NODE_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <unordered_map>

#include <bwi_scavenger_msgs/DatabaseProof.h>
#include <bwi_scavenger_msgs/PoseRequest.h>
#include <bwi_scavenger_msgs/DatabaseInfoSrv.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/OccupancyGrid.h>

#include "bwi_scavenger/dbscan_object.h"
#include "bwi_scavenger/globals.h"

#define FEAR_CONSTANT 5
#define PRIORITY_THRESHOLD 0.50

/*
  Labels to be used for getting information from the database
*/
enum data_label{
  GET_INCORRECT,
  SET_LOCATION,
  ADD_PROOF,
  CREATE_CLUSTERS
};

typedef std::pair<std::string, std::string> task;

struct task_hash
{
	template <class T1, class T2>
	std::size_t operator() (const std::pair<T1, T2> &pair) const
	{
		return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
	}
};

struct proof{
  geometry_msgs::Pose robot_pose;
  geometry_msgs::Pose secondary_pose;
  bool verification;
};

static std::unordered_map<task, std::vector<proof>, task_hash>* proofsMap;
static std::unordered_map<task, ObjectClusterer*, task_hash>* clustererMap;
static ros::ServiceClient pose_client;

/*
  Callback for updating the database's list of proofs
*/
void update_proofs_cb(const bwi_scavenger_msgs::DatabaseProof::ConstPtr &msg);

/*
  Callback that creates the clusterers for the set of proofs stored in the database
  Will be called once all proofs have been updated to the database.
*/
void create_clusterers_cb(const std_msgs::Bool::ConstPtr &msg);


/**
  ROS service for returning info based on cluster data. 
*/
bool info(bwi_scavenger_msgs::DatabaseInfoSrv::Request &msg,
          bwi_scavenger_msgs::DatabaseInfoSrv::Response &res);

#endif