#include <bwi_scavenger/object_cluster.h> 

ObjectCluster::ObjectCluster(int num){
  this->num = num;
  std::vector<point> newList;
  this->list = newList;

  this->robot_location = new float[ROBOT_DIMEN];
}

ObjectCluster::ObjectCluster(const ObjectCluster& old_cluster){
  num = old_cluster.num;
  num_correct = old_cluster.num_correct;
  num_incorrect = old_cluster.num_incorrect;
  
  float* location = new float[ROBOT_DIMEN];
  for(int i = 0; i < ROBOT_DIMEN; i++)
    location[i] = old_cluster.robot_location[i];
  
  robot_location = location;

  int size = old_cluster.list.size();
  for(int i = 0; i < size; i++){
    point cur_point = old_cluster.list[i];

    point temp_point;
    temp_point.label = cur_point.label;
    temp_point.num = cur_point.num;

    float* temp_coor = new float[OBJECT_DIMEN];
    for(int j = 0; j < OBJECT_DIMEN; j++)
      temp_coor[j] = cur_point.coordinate[j];
    temp_point.coordinate = temp_coor;
    
    list.push_back(temp_point);
  }

  correct = old_cluster.correct;
} 
  
ObjectCluster::~ObjectCluster(){
  delete[] robot_location;
  unsigned int size = list.size();
  for(int i = 0; i < size; i++)
    free(list[i].coordinate);
}

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
  float* location = new float[ROBOT_DIMEN];
  for(int i = 0; i < ROBOT_DIMEN; i++)
    location[i] = robot_location[i];
  return location;
}

/**
  Returns whether or not this cluster is labelled as correct or not
*/
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

/**
  Sets the cluster to be labelled as correct or incorrect
*/
void ObjectCluster::set_verification(bool ver){
  correct = ver;
}

/**
  Sets the robot location associated with this cluster (average)
*/
void ObjectCluster::set_robot_location(float* point){
  for(int i = 0; i < ROBOT_DIMEN; i++)
    robot_location[i] = point[i];
}

/**
  Adds the point to the cluster
*/
void ObjectCluster::add_to_list(point p){
  list.push_back(p);
}

/**
  Returns the cluster number
*/
int ObjectCluster::cluster_num(){
  return num;
}

/**
  Returns the size of the cluster
*/
int ObjectCluster::size(){
  return list.size();
}

/**
  Returns the point from the position passed in
*/
point ObjectCluster::get_point(int pos){
  return list[pos];
}