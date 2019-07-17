#include <bwi_scavenger/dbscan_object.h>

typedef std::vector<point*> Neighbors;


/**
  Calculates the distance between the two 3D points
*/
float calculate_distance(float* p1, float* p2){
  int total = 0;
  for(int i = 0; i < OBJECT_DIMEN; i++){
    total += pow((p1[i] - p2[i]), 2);
  }
  return sqrt(total);
}

/**
  Gets all the points that are within radius from the point
*/
Neighbors get_neighbors(point* db, point &p, float eps, int size_of_database){
  Neighbors n;
  for(int i = 0; i < size_of_database; i++){
    point &p2 = db[i];
    float dist = calculate_distance(p.coordinate, p2.coordinate);
    if(dist < EPS && p2.num != p.num)
      n.push_back(&p2);
  }
  return n;
}

/**
  Expands the cluster for each of the original point's neighbors
*/
void expand_cluster(point* db, ObjectCluster &current_cluster, Neighbors n, float eps, int minPoints, int size_of_database){
  for(int j = 0; j < n.size(); j++){
    point &secondary_point = *(n[j]);

    // if already labelled "noise" it cannot be a main cluster point, thus is set to edge point
    if(secondary_point.label == NOISE){
      secondary_point.label = IN_CLUSTER;
      current_cluster.add_to_list(secondary_point);
    }
    
    // continues if already in a cluster or was previously defined as noise and now is in this cluster (edge point)
    if(secondary_point.label != UNDEFINED)
      continue;

    Neighbors n2 = get_neighbors(db, secondary_point, eps, size_of_database);
    secondary_point.label = IN_CLUSTER;
    current_cluster.add_to_list(secondary_point);
    // main cluster gets points that are also with the min points required
    if(n2.size() < minPoints)
      continue;
    
    // labels each neighbors as in a cluster and add to cluster's list
    for(int k = 0; k < n2.size(); k++){   
      if(secondary_point.label != UNDEFINED)
        continue;
      point &point2 = *(n2[k]);
      point2.label = IN_CLUSTER;
      current_cluster.add_to_list(secondary_point);
    }
  }
}

/**
  Clusters the database with the given range of eps and minimum number of points
  Returns the number of clusters for the given parameters

  @param eps radius to check for neighbors at and main cluster radius
  @param minPoints minimum number of points required to create a cluster
*/
int ObjectClusterer::generate_clusters(float eps, int minPoints){
  
  int count = 0; // cluster id

  for(int i = 0; i < size_of_database; i++){
    point &current_point = database[i];
    current_point.num = i;
  }
  // goes through the entire dataset to create clusters
  for(int i = 0; i < size_of_database; i++){
    point &current_point = database[i];

    // only continue if label has not been placed yet
    if(current_point.label != UNDEFINED){
      continue;
    }
      
    Neighbors n = get_neighbors(database, current_point, EPS, size_of_database);
    // point is "noise" if it does not have enough neighbors;
    if(n.size() + 1 < minPoints){
      current_point.label = NOISE;
      continue;
    }

    current_point.label = IN_CLUSTER;
    ObjectCluster* cluster = new ObjectCluster(count);
    cluster->add_to_list(current_point);
    count++;
    // expand cluster with the current point's neighbors
    expand_cluster(database, *cluster, n, EPS, minPoints, size_of_database);
    cluster_list.push_back(*cluster);
  }
  return count;
}


/**
  Returns whether or not the point is contained in the cluster or not (not by guess)
*/
bool contains_point(float* point, ObjectCluster cluster){
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

/*
  Sets the verification and robot positions for the cluster
*/
void set_variables(int size_of_database, float** object_points, ObjectCluster &objc, bool* verification, float** robot_points){
  // for verification
  int correct = 0;
  int incorrect = 0;

  // for robot location
  float* robot_location = new float[ROBOT_DIMEN];
  
  for(int k = 0; k < ROBOT_DIMEN; k++){
    robot_location[k] = 0;
  }

  for(int j = 0; j < size_of_database; j++){
    if(contains_point(object_points[j], objc)){
      // for verification
      if(verification[j])
        incorrect++;
      else
        correct++;

      // for robot location
      float* cur_robot_point = robot_points[j];
      // std::cout << "adding point: ("; 
      for(int k = 0; k < ROBOT_DIMEN; k++){
        // std::cout << cur_robot_point[k] << " ";
        robot_location[k] += cur_robot_point[k];
      }
      // std::cout << ")" << std::endl;
      
    }
  }
  
  // for verification
  objc.set_num_correct(correct);
  objc.set_num_incorrect(incorrect);
  objc.set_verification(!(incorrect / (float) objc.size() > INCORRECT_THRESHOLD));

  // for robot location
  for(int k = 0; k < ROBOT_DIMEN; k++)
    robot_location[k] /= objc.size();
  objc.set_robot_location(robot_location);

  std::cout << "size: " << objc.size() << ", correct: " << std::to_string(correct) << ", incorrect: " 
    << std::to_string(objc.get_incorrect()) << ", robot location: (" << std::to_string(robot_location[0])
    << ", " << std::to_string(objc.get_robot_location()[1]) << ") ";
  if(objc.get_verification())
    std::cout << "CORRECT";
  else 
    std::cout << "INCORRECT";
  std::cout << std::endl;
}

ObjectClusterer::ObjectClusterer(float** object_points, float** robot_points, bool* verification, int size_of_database){
  
  this->size_of_database = size_of_database;
  this->database = new point[size_of_database];

  for(int i = 0; i < size_of_database; i++){
    point temp;
    temp.coordinate = new float[OBJECT_DIMEN];
    for(int j = 0; j < OBJECT_DIMEN; j++)
      temp.coordinate[j] = object_points[i][j];
    temp.label = UNDEFINED;
    database[i] = temp;
  }

  int num_clusters = generate_clusters(EPS, MIN_POINTS);

  for(int i = 0; i < num_clusters; i++){
    ObjectCluster &objc = cluster_list[i];

    set_variables(size_of_database, object_points, objc, verification, robot_points);
  }

}

ObjectClusterer::~ObjectClusterer(){
  delete[] database;
}

std::vector<int> ObjectClusterer::get_incorrect_clusters(){
  std::vector<int> result;
  for(int i = 0; i < cluster_list.size(); i++){
    ObjectCluster objc = cluster_list[i];
    if(objc.get_incorrect() / (float) objc.size() > INCORRECT_THRESHOLD)
      result.push_back(objc.cluster_num());
  }
  return result;
}

std::vector<int> ObjectClusterer::get_correct_clusters(){
  std::vector<int> result;
  for(int i = 0; i < cluster_list.size(); i++){
    ObjectCluster objc = cluster_list[i];
    if(objc.get_correct() / (float) objc.size() > CORRECT_THRESHOLD)
      result.push_back(objc.cluster_num());
  }
  return result;
}

float* ObjectClusterer::closest_correct(float* robot_position){

  float min_distance = FLT_MAX; 
  int cluster_num = -1;
  float* min_distance_position;
  for(int i = 0; i < cluster_list.size(); i++){
    ObjectCluster objc = cluster_list[i];
    if(objc.get_verification()){
      float* robot_location = objc.get_robot_location();
      float distance = calculate_distance(robot_location, robot_position);
      if(distance < min_distance){
        min_distance = distance;
        cluster_num = i;
        min_distance_position = robot_location;
      }
    }
  }
  return min_distance_position;
}

ObjectCluster* ObjectClusterer::get_cluster(int cluster_num){
  return &cluster_list[cluster_num];
}

ObjectCluster* ObjectClusterer::get_largest_cluster(){
  int max = 0;
  ObjectCluster *maxCluster;
  for(int i = 0; i < cluster_list.size(); i++){
    int size = cluster_list[i].size();
    if(size > max){
      max = size;
      maxCluster = &cluster_list[i];
    }
  }
  return maxCluster;
}

bool ObjectClusterer::in_cluster(float* point, int cluster_num){
  ObjectCluster &cluster = cluster_list[cluster_num];
  float dimen[OBJECT_DIMEN];
  for(int i = 0; i < cluster.size(); i++){
    for(int j = 0; j < OBJECT_DIMEN; j++)
      dimen[j] += cluster.get_point(i).coordinate[j];
  }
  
  for(int i = 0; i < OBJECT_DIMEN; i++)
    dimen[i] /= cluster.size();

  float distance = calculate_distance(dimen, point);
  return distance < EPS;
}
