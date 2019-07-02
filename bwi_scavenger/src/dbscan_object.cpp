#include <bwi_scavenger/dbscan_object.h>

/**
  Returns whether or not the point is contained in the cluster or not (not by guess)
*/
bool contains_point(float* point, Cluster cluster){
  for(int i = 0; i < cluster.size(); i++){
    float* coor = cluster.get_point(i).coordinate;
    bool contains = true;
    for(int j = 0; j < OBJECT_DIMEN; j++)
      contains = contains && (coor[j] == point[j]);
    if(contains)
      return true;
  }
  return false;
}

// sets the verification and robot positions for the cluster
void set_variables(int size_of_database, float** object_points, ObjectCluster* objc, bool* verification, float** robot_points){
  // for verification
  int correct = 0;
  int incorrect = 0;

  // for robot location
  float robot_location[ROBOT_DIMEN];

  for(int j = 0; j < size_of_database; j++){
    if(contains_point(object_points[j], *objc)){
      // for verification
      if(verification[j])
        correct++;
      else
        incorrect++;

      // for robot location
      float* cur_robot_point = robot_points[j];
      for(int k = 0; k < ROBOT_DIMEN; k++)
        robot_location[k] += cur_robot_point[k];
    }
  }
  
  // for verification
  objc->set_num_correct(correct);
  objc->set_num_incorrect(incorrect);
  objc->set_verification(!(incorrect / (float) objc->size() > INCORRECT_THRESHOLD));

  // for robot location
  for(int k = 0; k < ROBOT_DIMEN; k++)
    robot_location[k] /= objc -> size();
  objc->set_robot_location(robot_location);

  std::cout << "size: " << objc->size() << " correct: " << std::to_string(correct) << " incorrect: " 
    << std::to_string(objc->get_incorrect()) << " robot location: (" << std::to_string(robot_location[0])
    << ", " << std::to_string(objc->get_robot_location()[1]) << ") ";
  if(objc->get_verification())
    std::cout << "CORRECT";
  else 
    std::cout << "INCORRECT";

  std::cout << std::endl;
}

/**
  Creates an ObjectClusterer by clustering points from the "Find Object" task. Sets the number of incorrect and correct points 
  in each set based on the verification array sent and the clusters generated.
*/
ObjectClusterer::ObjectClusterer(float** object_points, float** robot_points, bool* verification, int size_of_database):
  Clusterer(object_points, OBJECT_DIMEN, size_of_database){
  
  int num_clusters = generate_clusters(EPS, MIN_POINTS);
    
  for(int i = 0; i < num_clusters; i++){
    ObjectCluster* objc = (ObjectCluster*) &cluster_list[i];

    set_variables(size_of_database, object_points, objc, verification, robot_points);

    // std::cout << "Cluster number " << std::to_string(i)<< " size " << std::to_string(objc->size()) << std::endl;
    // for(int k = 0; k < objc->size(); k++)
    //   std::cout << "(" << std::to_string(objc->get_point(k).coordinate[0]) << ", " << 
    //   std::to_string(objc->get_point(k).coordinate[1]) << ") " << 
    //   std::to_string(objc->get_point(k).num) << std::endl;
    // std::cout << std::endl;
  }

  this->object_points = object_points;
  this->robot_points = robot_points;
  this->size_of_database = size_of_database;
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
  for(int i = 0; i < cluster_list.size(); i++){
    ObjectCluster* objc = (ObjectCluster*) &cluster_list[i];
    if(objc->get_correct() / (float) objc->size() > CORRECT_THRESHOLD)
      result.push_back(objc->cluster_num());
  }
  return result;
}

/**
  Obtains all of the clusters above a certain threshold of "incorrectness"
  Returns a vector of cluster numbers that are considered "incorrect"
*/
std::vector<int> ObjectClusterer::get_incorrect_clusters(){
  std::vector<int> result;
  for(int i = 0; i < cluster_list.size(); i++){
    ObjectCluster* objc = (ObjectCluster*) &cluster_list[i];
    // std::cout << "size: " << current_cluster -> size();
    // std::cout << " There are " << std::to_string(current_cluster->get_incorrect()) << " num incorrect in this cluster" << std::endl;
    if(objc->get_incorrect() / (float) objc->size() > INCORRECT_THRESHOLD)
      result.push_back(objc->cluster_num());
  }
  return result;
}

/**
  Calculates the closest location to the current position of the robot that 
  contains a "correct" cluster. Returns a point of the correct position
  @param robot_pose pose of robot
*/
float* ObjectClusterer::closest_correct(float* robot_position){

  float min_distance = FLT_MAX; 
  int cluster_num = -1;
  float* min_distance_position;
  for(int i = 0; i < cluster_list.size(); i++){
    ObjectCluster* objc = (ObjectCluster*) &cluster_list[i];
    if(objc -> get_verification()){
      float* robot_location = objc -> get_robot_location();
      float distance = calculate_distance(robot_location, robot_position, OBJECT_DIMEN);
      if(distance < min_distance){
        min_distance = distance;
        cluster_num = i;
        min_distance_position = robot_location;
    }
    }
  }
  return min_distance_position;
}

//ObjectCluster 

ObjectCluster::ObjectCluster(int num):Cluster(num){
  std::vector<point> newList;
  list = newList;
  float avg_robot_location[ROBOT_DIMEN];
}

ObjectCluster::~ObjectCluster(){}

/**
  Returns the number of correct values in this cluster
*/
int ObjectCluster::get_correct(){
  return num_correct;
}

/**
  Returns the number of incorrect values in this cluster
*/
int ObjectCluster::get_incorrect(){
  return num_incorrect;
}

/**
  Returns the average robot location associated with this cluster of object points
*/
float* ObjectCluster::get_robot_location(){
  return robot_location;
}

bool ObjectCluster::get_verification(){
  return correct;
}

/**
  Sets the number of correct values in this cluster to num
*/
void ObjectCluster::set_num_correct(int num){
  num_correct = num;
}
/**
  Sets the number of incorrect values in this cluster to num
*/
void ObjectCluster::set_num_incorrect(int num){
  num_incorrect = num;
}

void ObjectCluster::set_verification(bool ver){
  correct = ver;
}

void ObjectCluster::set_robot_location(float* point){
  for(int i = 0; i < ROBOT_DIMEN; i++){
    *(robot_location + i) = *(point + i);
  }
}