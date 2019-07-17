#include <bwi_scavenger/database_node.h>

void update_proofs(task curTask, proof curProof){
  // ROS_INFO("[database_node] adding location: (%f, %f)", curProof.robot_pose.position.x, curProof.robot_pose.position.y);
  if(proofsMap->count(curTask)){ // map contains this task, add onto the proofs vector
    std::vector<proof> &mapProof = (*proofsMap)[curTask];
    mapProof.push_back(curProof);
  
  } else { // map does not contain this task yet, simply add in
    std::vector<proof> newList;
    newList.push_back(curProof);
    proofsMap->insert({curTask, newList});
  }

}

void create_clusterers(){
  ROS_INFO("[database_node] Creating Clusterers");
  for(const auto &taskProof : *proofsMap){
    std::vector<proof> curProofList = taskProof.second;

    std::vector<bool> verification_v;
    std::vector<float*> robot_points_v;
    std::vector<float*> secondary_points_v;

    int count = 0;
    while(count < curProofList.size()){
      proof curProof = curProofList[count];
      count++;
      verification_v.push_back(curProof.verification);

      float* robot_point = new float[3];
      robot_point[0] = curProof.robot_pose.position.x;
      robot_point[1] = curProof.robot_pose.position.y;
      robot_point[2] = curProof.robot_pose.position.z;
      robot_points_v.push_back(robot_point);

      float* secondary_point = new float[3];
      secondary_point[0] = curProof.secondary_pose.position.x;
      secondary_point[1] = curProof.secondary_pose.position.y;
      secondary_point[2] = curProof.secondary_pose.position.z;
      secondary_points_v.push_back(secondary_point);

    }

    bool* verification = new bool[curProofList.size()];
    for(int i = 0; i < verification_v.size(); i++)
      verification[i] = verification_v[i];

    float** robot_points = &robot_points_v[0];
    float** secondary_points = &secondary_points_v[0];

    task curTask = taskProof.first;

    // generate correct clusterer for the task type
    if(curTask.first == TASK_FIND_OBJECT){
      ROS_INFO("[database_node] Find Object / %s Clusterer", curTask.second.c_str());
      ObjectClusterer* c_ptr = new ObjectClusterer(secondary_points, robot_points, verification, curProofList.size());
      clustererMap->insert({curTask, c_ptr});
    }
  }
}

/*
  Returns whether or not the given pose is in an incorrect cluster
*/
bool is_correct(task curTask, geometry_msgs::Pose obj_pose){
  if(clustererMap->count(curTask)){
    ObjectClusterer *curClusterer = clustererMap->at(curTask);

    float point[3];
    point[0] = obj_pose.position.x;
    point[1] = obj_pose.position.y;
    point[2] = obj_pose.position.z;

    std::vector<int> incorrect_clusters = curClusterer->get_incorrect_clusters();
    for(int i = 0; i < incorrect_clusters.size(); i++){
      if(curClusterer->in_cluster(point, incorrect_clusters[i])){
        ROS_INFO("[database_node] Point is in an incorrect cluster! Do not save.");
        return false;
      }
    }
  }
  ROS_INFO("[database_node] Point is NOT in an incorrect cluster! Save point.");
  return true;
}

/*
  Generates a list of locations and prioirities
  location list stores floats so every 2 numbers are a coordinate point
  priority list stores floats of priorities based on the number of correct points in the cluster

  prioirty_list[i] is associated with the point (location_list[i * 2], location_list[i * 2 + 1])
*/
void set_priority_locations(task curTask, bwi_scavenger_msgs::DatabaseInfoSrv::Response &res){
  if(clustererMap->count(curTask)){

    ObjectClusterer *curClusterer = clustererMap->at(curTask);
    std::vector<int> correct_clusters = curClusterer->get_correct_clusters();
    for(int i = 0; i < correct_clusters.size(); i++){
      ObjectCluster* obj_cluster = curClusterer->get_cluster(correct_clusters[i]);
      float *robot_location = obj_cluster->get_robot_location();

      float priority = obj_cluster->get_correct() - (obj_cluster->get_incorrect() * FEAR_CONSTANT);
      priority /= obj_cluster->size();

      if(priority > PRIORITY_THRESHOLD){
        res.location_list.push_back(robot_location[0]);
        res.location_list.push_back(robot_location[1]);
      
        res.priority_list.push_back(priority);
      }
    }
  }
}

bool info(bwi_scavenger_msgs::DatabaseInfoSrv::Request &msg,
          bwi_scavenger_msgs::DatabaseInfoSrv::Response &res){
  // ROS_INFO("[database_node] Getting info for %s / %s", msg.task_name.c_str(), msg.parameter_name.c_str());
  bool methodWorking = true;

  task curTask = {msg.task_name, msg.parameter_name};

  if(msg.data == GET_INCORRECT){
    geometry_msgs::Pose obj_pose = msg.pose;
    res.correct = is_correct(curTask, obj_pose);
  
  } else if(msg.data == SET_LOCATION){
    set_priority_locations(curTask, res);
  
  } else if(msg.data == ADD_PROOF){
    // ROS_INFO("[database_node] Adding proof to clusterers");
    proof curProof;
    curProof.robot_pose = msg.pose;
    curProof.secondary_pose = msg.secondary_pose;
    curProof.verification = msg.verification;
    update_proofs(curTask, curProof);
  } else if(msg.data == CREATE_CLUSTERS){
    create_clusterers();
  } else 
    methodWorking = false;

  return methodWorking; // will note if there is a method associated with the data type sent
}

int main(int argc, char **argv){
  ros::init(argc, argv, "database_node");
  ros::NodeHandle n;

  // ros::Subscriber sub0 = n.subscribe(TPC_DATABASE_NODE_UPDATE_PROOF, 1, update_proofs_cb);
  // ros::Subscriber sub1 = n.subscribe(TPC_DATABASE_NODE_DONE_PARSE, 1, create_clusterers_cb);

  pose_client = n.serviceClient<bwi_scavenger_msgs::PoseRequest>(SRV_POSE_REQUEST);
  ros::ServiceServer srv_info_request = n.advertiseService(SRV_DATABASE_INFO_REQUEST, info);

  std::unordered_map<task, std::vector<proof>, task_hash> tempMap;
  proofsMap = &tempMap;

  std::unordered_map<task, ObjectClusterer*, task_hash> tempMap2;
  clustererMap = &tempMap2;

  ROS_INFO("[database_node] Standing by.");

  ros::spin();
}
