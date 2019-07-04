#include <bwi_scavenger/dbscan.h>

typedef std::vector<point*> Neighbors;

/**
  Creates a new Clusterer object that stores the database of poses, radius eps and
  the minimum number of points for a cluster to form
*/
template <class T> Clusterer<T>::Clusterer(float** db, int num_dimensions, int size_of_database){ 
  this->num_dimensions = num_dimensions;
  this->size_of_database = size_of_database;
  this->database = new point[size_of_database];

  for(int i = 0; i < size_of_database; i++){
    point temp;
    temp.coordinate = new float[num_dimensions];
    for(int j = 0; j < num_dimensions; j++){
      temp.coordinate[j] = db[i][j];
    }
    temp.label = UNDEFINED;
    database[i] = temp;
  }
}

template <class T> Clusterer<T>::~Clusterer(){
  delete database;
  delete &cluster_list;
}

/**
  Calculates the distance between the two 3D points
*/
float calculate_distance(float* p1, float* p2, int num_dimensions){
  int total = 0;
  for(int i = 0; i < num_dimensions; i++){
    total += pow((p1[i] - p2[i]), 2);
  }
  return sqrt(total);
}

/**
  Gets all the points that are within radius from the point
*/
Neighbors get_neighbors(point* db, point &p, float eps, int size_of_database, int num_dimensions){
  Neighbors n;
  for(int i = 0; i < size_of_database; i++){
    point &p2 = db[i];
    float dist = calculate_distance(p.coordinate, p2.coordinate, num_dimensions);
    if(dist < eps && p2.num != p.num)
      n.push_back(&p2);
  }
  return n;
}

/**
  Expands the cluster for each of the original point's neighbors
*/
void expand_cluster(point* db, Cluster &current_cluster, Neighbors n, float eps, int minPoints, int size_of_database, int num_dimensions){
  // std::cout << " expand_cluster " << std::endl;
  for(int j = 0; j < n.size(); j++){
    point &secondary_point = *(n[j]);

    // if already labelled "noise" it cannot be a main cluster point, thus is set to edge point
    if(secondary_point.label == NOISE){
      // std::cout << "noise" << std::endl;
      secondary_point.label = IN_CLUSTER;
      current_cluster.add_to_list(secondary_point);
    }
    
    // continues if already in a cluster or was previously defined as noise and now is in this cluster (edge point)
    if(secondary_point.label != UNDEFINED)
      continue;

    Neighbors n2 = get_neighbors(db, secondary_point, eps, size_of_database, num_dimensions);
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
template <class T> int Clusterer<T>::generate_clusters(float eps, int minPoints){

  this->eps = eps;
  this->minPoints = minPoints;
  
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
      
    Neighbors n = get_neighbors(database, current_point, eps, size_of_database, num_dimensions);
    // point is "noise" if it does not have enough neighbors;
    if(n.size() + 1 < minPoints){
      current_point.label = NOISE;
      continue;
    }

    current_point.label = IN_CLUSTER;
    Cluster cluster(count);
    cluster.add_to_list(current_point);
    count++;
    // expand cluster with the current point's neighbors
    // std::cout << std::to_string(current_point.num);
    expand_cluster(database, cluster, n, eps, minPoints, size_of_database, num_dimensions);
    // std::cout << "adding to cluster list" << std::endl;
    cluster_list.push_back(cluster);
  }
  return count;
}

/**
  Returns a the cluster associated with the cluster number
*/
template <class T> Cluster Clusterer<T>::get_cluster(int cluster_num){
  return cluster_list[cluster_num];
}
/**
  Returns the cluster number of the largest cluster in the data set
*/
template <class T> Cluster Clusterer<T>::get_largest_cluster(){
  int max = 0;
  Cluster *maxCluster;
  for(int i = 0; i < cluster_list.size(); i++){
    int size = cluster_list[i].size();
    if(size > max){
      max = size;
      maxCluster = &cluster_list[i];
    }
  }
  return *maxCluster;
}

/** 
  Returns whether or not a point is contained in a specific cluster

  @param point Point that is being checked
  @param cluster cluster  that the point is being checked against
*/
template <class T> bool Clusterer<T>::in_cluster(float* point, int cluster_num){
  Cluster &cluster = cluster_list[cluster_num];
  float dimen[num_dimensions];
  for(int i = 0; i < cluster.size(); i++){
    for(int j = 0; j < num_dimensions; j++)
      dimen[j] += cluster.get_point(i).coordinate[j];
  }
  
  for(int i = 0; i < num_dimensions; i++){
    // std::cout << dimen[i] << " ";
    dimen[i] /= cluster.size();
  }
  // std::cout << dimen[0] << ", " << dimen[1] << std::endl;
  float distance = calculate_distance(dimen, point, num_dimensions);
  return distance < eps;
}

Cluster::Cluster(int num){
  this->num = num;
  std::vector<point> newList;
  this->list = newList;
}

Cluster::~Cluster(){}


void Cluster::add_to_list(point p){
  list.push_back(p);
}

int Cluster::cluster_num(){
  return num;
}

int Cluster::size(){
  return list.size();
}

point Cluster::get_point(int pos){
  return list[pos];
}

