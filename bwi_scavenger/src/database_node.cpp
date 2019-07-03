#include <ros/ros.h>

#include <bwi_scavenger/global_topics.h>
#include <bwi_scavenger_msgs/DatabaseProof.h>
#include <bwi_scavenger_msgs/DatabaseInfo.h>
#include <bwi_scavenger_msgs/PoseRequest.h>

#include <bwi_scavenger/dbscan.h>
#include <bwi_scavenger/dbscan_object.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <unordered_map>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

enum data_label{
  GET_INCORRECT
};

typedef std::pair<std::string, std::string> task;

static tf::TransformListener *tfl;
static std::string gridFrameId;

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
  proof* next_proof;
};

static std::unordered_map<task, proof, task_hash>* proofsMap;
static std::unordered_map<task, Clusterer, task_hash>* clustererMap;
static ros::Publisher pub_incorrect_point;
static ros::ServiceClient pose_client;


void update_proofs_cb(const bwi_scavenger_msgs::DatabaseProof::ConstPtr &msg){

  // ROS_INFO("[Database_node] Updating proofs map for %s / %s", msg->task.c_str(), msg->param.c_str());
  task curTask = {msg->task, msg->param};

  proof curProof;
  curProof.robot_pose = msg->robot_pose;
  curProof.secondary_pose = msg->secondary_pose;
  curProof.verification = msg->verification;
  curProof.next_proof = 0;

  if(proofsMap->count(curTask)){ // map does not contain this task yet, simply add in
    proofsMap->insert({curTask, curProof});  

  } else { // map contains this task, add onto the proofs linked list
    proof mapProof = (*proofsMap)[curTask];
    curProof.next_proof = &mapProof;
    proofsMap->insert({curTask, curProof});
  }
}

void create_clusterers_cb(const std_msgs::Bool::ConstPtr &msg){
  ROS_INFO("[Database_node] Creating Clusterers");
  for(const auto &taskProof : *proofsMap){
    proof curProof = taskProof.second;

    int num_proofs = 0;
    std::vector<bool> verification_v;
    std::vector<float*> robot_points_v;
    std::vector<float*> secondary_points_v;

    while(curProof.next_proof){
      num_proofs++;
      verification_v.push_back(curProof.verification);
      
      float robot_point[3];
      robot_point[0] = curProof.robot_pose.position.x;
      robot_point[1] = curProof.robot_pose.position.y;
      robot_point[2] = curProof.robot_pose.position.z;
      robot_points_v.push_back(robot_point);

      float secondary_point[3];
      secondary_point[0] = curProof.secondary_pose.position.x;
      secondary_point[1] = curProof.secondary_pose.position.y;
      secondary_point[2] = curProof.secondary_pose.position.z;
      secondary_points_v.push_back(secondary_point);
      
    }
    bool* verification = new bool[num_proofs];
    std::copy(std::begin(verification_v), std::end(verification_v), verification);

    float** robot_points = &robot_points_v[0];
    float** secondary_points = &secondary_points_v[0];

    task curTask = taskProof.first;

    if(curTask.first == "Find Object"){
      ROS_INFO("[Database_node] Find Object / %s Clusterer", curTask.second.c_str());
      ObjectClusterer c(secondary_points, robot_points, verification, num_proofs);
      clustererMap->insert({curTask, c});
    }

    delete[] verification;
  }
}

void getMapId(const nav_msgs::OccupancyGrid::ConstPtr &grid){
  tfl = new tf::TransformListener();
  gridFrameId = grid->header.frame_id;
}

void get_info_cb(const bwi_scavenger_msgs::DatabaseInfo::ConstPtr &msg){

  if(msg->data == GET_INCORRECT){
    if(!clustererMap->count(curTask)){
      
      task curTask = {msg->task_name, msg->parameter_name};

      geometry_msgs::Pose obj_pose = msg->pose;

      ObjectClusterer* curClusterer = (ObjectClusterer*) &clustererMap->at(curTask);

      bwi_scavenger_msgs::PoseRequest req;
      pose_client.call(req);

      // ROS_INFO("[database_node] Robot point: (%f ,%f, %f)", req.response.pose.position.x, req.response.pose.position.y, req.response.pose.position.z);

      tf::Quaternion rpy(req.response.pose.orientation.x,
                         req.response.pose.orientation.y,
                         req.response.pose.orientation.z,
                         req.response.pose.orientation.w);

      double roll, pitch, yaw;
      tf::Matrix3x3(rpy).getRPY(roll, pitch, yaw);

      // ROS_INFO("obj relative: (%f, %f)", obj_pose.position.x, obj_pose.position.y);

      float point[3];
      point[0] = req.response.pose.position.x + cos(yaw) * obj_pose.position.x - sin(yaw) * obj_pose.position.y;
      point[1] = req.response.pose.position.y + sin(yaw) * obj_pose.position.x + cos(yaw) * obj_pose.position.y;
      point[2] = req.response.pose.position.z + obj_pose.position.z;

      // ROS_INFO("[database_node] Object point: (%f ,%f, %f)", point[0], point[1], point[2]);

      std::vector<int> incorrect_clusters = curClusterer->get_incorrect_clusters();

      for(int i = 0; i < incorrect_clusters.size(); i++){
        if(curClusterer->in_cluster(point, i)){
          ROS_INFO("[database_node] Point is in an incorrect cluster! Do not save.");
          std_msgs::Bool msgIncorrect;
          msgIncorrect.data = true;
          pub_incorrect_point.publish(msgIncorrect);
          return;
        }
      }

    }

    // ROS_INFO("[database_node] Point is NOT in an incorrect cluster! Save point.");
    std_msgs::Bool msgIncorrect;
    msgIncorrect.data = false;
    pub_incorrect_point.publish(msgIncorrect);
  
  }

}

int main(int argc, char **argv){
  ros::init(argc, argv, "database_node");
  ros::NodeHandle n;

  ros::Subscriber mapSub = n.subscribe("/level_mux/map", 1, getMapId);

  ros::Subscriber sub0 = n.subscribe(TPC_DATABASE_NODE_UPDATE_PROOF, 1, update_proofs_cb);
  ros::Subscriber sub1 = n.subscribe(TPC_DATABASE_NODE_DONE_PARSE, 1, create_clusterers_cb);
  ros::Subscriber sub2 = n.subscribe(TPC_DATABASE_NODE_GET_INFO, 1, get_info_cb);

  pub_incorrect_point = n.advertise<std_msgs::Bool>(TPC_DATABASE_NODE_INCORRECT, 1);

  pose_client = n.serviceClient<bwi_scavenger_msgs::PoseRequest>("pose_request");

  std::unordered_map<task, proof, task_hash> tempMap;
  proofsMap = &tempMap;

  std::unordered_map<task, Clusterer, task_hash> tempMap2;
  clustererMap = &tempMap2;

  tfl = new tf::TransformListener();

  ros::spin();
}