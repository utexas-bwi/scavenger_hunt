#include <bwi_scavenger/dbscan.h>
#include <iostream>
#include <math.h>

#define INCORRECT_THRESHOLD 50
#define CORRECT_THRESHOLD 75

typedef std::vector<point> Neighbors;


/**
  Creates a new Clusterer object that stores the database of poses, radius eps and
  the minimum number of points for a cluster to form
*/
Clusterer::Clusterer(float** db, int num_dimensions, int size_of_database){ 
  this->num_dimensions = num_dimensions;
  this->size_of_database = size_of_database;
  this->database = new point[size_of_database];

  for(int i = 0; i < size_of_database; i++){
    point temp;
    temp.coordinate = new float[num_dimensions];
    for(int j = 0; j < num_dimensions; j++){
      temp.coordinate[j] = db[i][j];
    }
    // temp.verification = db[i].verification;
    temp.label = UNDEFINED;
    // temp.num = i;
    // robot_positions.push_back(db[i].robot_pose.position);
    database[i] = temp;
  }
}

Clusterer::~Clusterer(){
  // delete &database;
  // delete &cluster_list;
}

/**
  Calculates the distance between the two 3D points
*/
static float calculate_distance(float* p1, float* p2, int num_dimensions){
  int total = 0;
  for(int i = 0; i < num_dimensions; i++)
    total += pow((p1[i] - p2[i]), 2);
<<<<<<< HEAD
  return abs(pow(total, 1.0 / p1.size()));
=======
  // std::cout << "distance" <<  std::to_string(abs(pow(total, 1.0 / num_dimensions))) << std::endl;
  return abs(pow(total, 1.0 / num_dimensions));
>>>>>>> e23f41da04eae9f14c5893b1add17288a8e7ff69
}

/**
  Gets all the points that are within radius from the point
*/
Neighbors get_neighbors(point* db, float* p, float eps, int size_of_database, int num_dimensions){
  Neighbors n;
  for(int i = 0; i < size_of_database; i++){
    point p2 = db[i];
    float dist = calculate_distance(p, p2.coordinate, num_dimensions);
    if(dist < eps && dist != 0)
      n.push_back(p2);
  }
  return n;
}

/**
  Expands the cluster for each of the original point's neighbors
*/
void expand_cluster(point* db, cluster current_cluster, Neighbors n, float eps, int minPoints, int size_of_database, int num_dimensions){

  for(int j = 0; j < n.size(); j++){
    point secondary_point = n[j];

    // if already labelled "noise" it cannot be a main cluster point, thus is set to edge point
    if(secondary_point.label == NOISE){
      current_cluster.list.push_back(secondary_point);
      secondary_point.label = IN_CLUSTER;
    }

    // continues if already in a cluster or was previously defined as noise and now is in this cluster (edge point)
    if(secondary_point.label != UNDEFINED)
      continue;
<<<<<<< HEAD

    Neighbors n2 = get_neighbors(db, secondary_point.coordinate, eps);
=======
    
    Neighbors n2 = get_neighbors(db, secondary_point.coordinate, eps, size_of_database, num_dimensions);
>>>>>>> e23f41da04eae9f14c5893b1add17288a8e7ff69
    secondary_point.label = IN_CLUSTER;
    // main cluster gets points that are also with the min points required
    if(n2.size() < minPoints)
      continue;
<<<<<<< HEAD

    // std::cout << "expanding cluster again" << std::endl;
    // // labels each neighbors as in a cluster and add to cluster's list
    // for(int k = 0; k < n2.size(); k++){
    //   n2[k].label = IN_CLUSTER;
    //   current_cluster.list.push_back(n2[k]);
    //   //continue to expand the cluster with this point
    //   expand_cluster(db, current_cluster, get_neighbors(db, n2[k].coordinate, eps), eps, minPoints);

    // }
=======
    
    // labels each neighbors as in a cluster and add to cluster's list
    for(int k = 0; k < n2.size(); k++){
      n2[k].label = IN_CLUSTER;
      current_cluster.list.push_back(n2[k]);
    }
>>>>>>> e23f41da04eae9f14c5893b1add17288a8e7ff69
  }
}

/**
  Clusters the database with the given range of eps and minimum number of points
  Returns the number of clusters for the given parameters

  @param eps radius to check for neighbors at and main cluster radius
  @param minPoints minimum number of points required to create a cluster
*/
int Clusterer::get_clusters(float eps, int minPoints){

  this->eps = eps;
  this->minPoints = minPoints;

  int count = 0; // cluster id
  // goes through the entire dataset to create clusters
  for(int i = 0; i < size_of_database; i++){
    point current_point = database[i];

    // only continue if label has not been placed yet
    if(current_point.label != UNDEFINED)
      continue;
<<<<<<< HEAD

    Neighbors n = get_neighbors(database, current_point.coordinate, eps);
=======
    
    Neighbors n = get_neighbors(database, current_point.coordinate, eps, size_of_database, num_dimensions);
>>>>>>> e23f41da04eae9f14c5893b1add17288a8e7ff69
    // point is "noise" if it does not have enough neighbors;
    if(n.size() < minPoints){
      current_point.label = NOISE;
      continue;
    }

    current_point.label = IN_CLUSTER;
    count++;
    cluster cluster;
    // std::cout << "Count " << std::to_string(count) << std::endl;
    cluster.num = count;
    // original cluster is the point and its neighbors
    cluster.list = n;
    cluster_list.push_back(cluster);
    // expand cluster with the current point's neighbors
    expand_cluster(database, cluster, n, eps, minPoints, size_of_database, num_dimensions);
  }
  return count;
}

<<<<<<< HEAD
/**
  Obtains all of the clusters above a certain threshold of "incorrectness"
  Returns a vector of cluster numbers that are considered "incorrect"
*/
std::vector<int> Clusterer::get_incorrect(){
  std::vector<int> result;
  for(int i = 0; i < cluster_list.size(); i++){
    cluster current_cluster = cluster_list[i];
    if(current_cluster.num_incorrect / current_cluster.list.size() > INCORRECT_THRESHOLD)
      result.push_back(current_cluster.num);
  }
  return result;
}

/**
  Obtains all of the clusters above a certain threshold of "correctness"
  Returns a vector of cluster numbers that are considered "correct"
*/

std::vector<int> Clusterer::get_correct(){
  std::vector<int> result;
  for(int i = 0; i < cluster_list.size(); i++){
    cluster current_cluster = cluster_list[i];
    if(current_cluster.num_correct / current_cluster.list.size() > CORRECT_THRESHOLD)
      result.push_back(current_cluster.num);
  }
  return result;
}

// /**
//   Gets the mean location based on the robot positions from the database
// */
// geometry_msgs::Point mean_robot_location(cluster cluster, std::vector<geometry_msgs::Point> robot_positions){
//   double totalX = 0;
//   double totalY = 0;
//   int length = cluster.list.size();

//   for(int i = 0; i < length; i++){
//     int num = cluster.list[i].num;
//     geometry_msgs::Point pos = robot_positions[i];
//     totalX += pos.x;
//     totalY += pos.y;
//   }
//   geometry_msgs::Point result;
//   result.x = totalX / length;
//   result.y = totalY / length;
//   return result;
// }

// /**
//   Calculates the closest location to the current position of the robot that
//   contains a "correct" point. Returns a pose of the correct position

//   @param robot_pose pose of robot
// */
// geometry_msgs::Point Clusterer::closest_correct(geometry_msgs::Pose robot_pose){

//   double min_distance = DBL_MAX;
//   int cluster_num = -1;
//   geometry_msgs::Point min_distance_point;

//   for(int i = 0; i < cluster_list.size(); i++){
//     geometry_msgs::Point robot_position = mean_robot_location(cluster_list[i], robot_positions);
//     double distance = calculate_distance(robot_position, robot_pose.position);
//     if(distance < min_distance){
//       min_distance = distance;
//       cluster_num = i;
//       min_distance_point = robot_position;
//     }
//   }

//   return min_distance_point;
// }

/**
=======
/** 
>>>>>>> e23f41da04eae9f14c5893b1add17288a8e7ff69
  Returns whether or not a point is contained in a specific cluster

  @param point Point that is being checked
  @param cluster_num cluster number that the point is being checked against
*/
bool Clusterer::in_cluster(float* point, int cluster_num){
  cluster cluster = get_cluster(cluster_num);
  for(int i = 0; i < cluster.list.size(); i++){
    float distance = calculate_distance(cluster.list[i].coordinate, point, num_dimensions);
    if(distance < eps)
      return true;
  }
  return false;
}

/**
  Returns a the cluster associated with the cluster number
*/
cluster Clusterer::get_cluster(int cluster_num){
  return cluster_list[cluster_num];
}
/**
  Returns the cluster number of the largest cluster in the data set
*/
int Clusterer::get_largest_cluster(){
  int max = 0;
  int maxCluster;
  for(int i = 0; i < cluster_list.size(); i++){
    int size = cluster_list[i].list.size();
    if(size > max){
      max = size;
      maxCluster = cluster_list[i].num;
    }
  }
  return maxCluster;
}
