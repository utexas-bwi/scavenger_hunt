#include <bwi_scavenger/dbscan_object.h>

ObjectClusterer::ObjectClusterer(float** object_points, float** robot_points, bool* verification, int size_of_database):
  Clusterer(object_points, OBJECT_DIMEN, size_of_database){
  
  num_clusters = generate_clusters(EPS, MIN_POINTS);
    
  for(int i = 0; i < num_clusters; i++){
    Cluster* current_cluster = &cluster_list[i];
    ObjectCluster* objc = (ObjectCluster*) (current_cluster);

    // std::cout << "Cluster number " << std::to_string(i)<< " size " << std::to_string(objc->size()) << std::endl;
    // for(int k = 0; k < objc->size(); k++)
    //   std::cout << "(" << std::to_string(objc->get_point(k).coordinate[0]) << ", " << 
    //   std::to_string(objc->get_point(k).coordinate[1]) << ") " << 
    //   std::to_string(objc->get_point(k).num) << std::endl;
    // std::cout << std::endl;
    
    int correct = 0;
    int incorrect = 0;
    for(int j = 0; j < size_of_database; j++){
      if(in_cluster(object_points[j], *current_cluster)){
        if(verification[j])
          correct++;
        else
          incorrect++;
      }
    }
    objc->set_num_correct(correct);
    objc->set_num_incorrect(incorrect);
    std::cout << "size: " << objc->size() << " correct: " << std::to_string(correct) << " incorrect: " << std::to_string(objc->get_incorrect()) << std::endl;
  }

  this->object_points = object_points;
  this->robot_points = robot_points;
  this->size_of_database = size_of_database;
  Cluster* clus = &cluster_list[0];
  ObjectCluster* obj = (ObjectCluster*) (clus);
  std::cout<<obj->size() << " " << obj->get_incorrect() << " " << obj->get_correct() << std::endl;
}

ObjectClusterer::~ObjectClusterer(){
  // delete c;
  // delete obj_cluster_list;
  // delete object_points;
  // delete robot_points;
}

/**
  Obtains all of the clusters above a certain threshold of "correctness"
  Returns a vector of cluster numbers that are considered "correct"
*/
std::vector<int> ObjectClusterer::get_correct_clusters(){
  std::vector<int> result;
  for(int i = 0; i < num_clusters; i++){
    Cluster* clus = &cluster_list[i];
    ObjectCluster* current_cluster = (ObjectCluster*) clus;
    if(current_cluster->get_correct() / (double) current_cluster->size() > CORRECT_THRESHOLD)
      result.push_back(current_cluster->cluster_num());
  }
  return result;
}

/**
  Obtains all of the clusters above a certain threshold of "incorrectness"
  Returns a vector of cluster numbers that are considered "incorrect"
*/
std::vector<int> ObjectClusterer::get_incorrect_clusters(){
  std::vector<int> result;
  for(int i = 0; i < num_clusters; i++){
    Cluster* clus = &cluster_list[i];
    ObjectCluster* current_cluster = (ObjectCluster*) clus;
    // std::cout << "size: " << current_cluster -> size();
    // std::cout << " There are " << std::to_string(current_cluster->get_incorrect()) << " num incorrect in this cluster" << std::endl;
    if(current_cluster->get_incorrect() / (double) current_cluster->size() > INCORRECT_THRESHOLD)
      result.push_back(current_cluster->cluster_num());
  }
  return result;
}

/**
  Gets the mean location based on the robot positions from the database
*/
float* avg_robot_location(Cluster cluster){
}

/**
  Calculates the closest location to the current position of the robot that 
  contains a "correct" point. Returns a pose of the correct position
  @param robot_pose pose of robot
*/
float* ObjectClusterer::closest_correct(float* robot_position){
  float min_distance = DBL_MAX; 
  int cluster_num = -1;
  float* min_distance_position;
  for(int i = 0; i < num_clusters; i++){
    // float* avg_position = avg_robot_location(*obj_cluster_list[i]);
    // float distance = calculate_distance(avg_position, robot_position, OBJECT_DIMEN);
    // if(distance < min_distance){
    //   min_distance = distance;
    //   cluster_num = i;
    //   min_distance_position = avg_position;
    // }
  }
  return min_distance_position;
}

ObjectCluster::ObjectCluster(int num):Cluster(num){
  std::vector<point> newList;
  list = newList;
}

ObjectCluster::~ObjectCluster(){}

int ObjectCluster::get_correct(){
  return num_correct;
}

int ObjectCluster::get_incorrect(){
  return num_incorrect;
}

void ObjectCluster::set_num_correct(int num){
  num_correct = num;
}

void ObjectCluster::set_num_incorrect(int num){
  num_incorrect = num;
}