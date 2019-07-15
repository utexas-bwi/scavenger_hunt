#ifndef OBJECT_CLUSTER_H
#define OBJECT_CLUSTER_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <limits.h>
#include <math.h>

#define OBJECT_DIMEN 2
#define ROBOT_DIMEN 2
#define INCORRECT_THRESHOLD 0.50
#define CORRECT_THRESHOLD 0.75


enum label{
  UNDEFINED,
  NOISE,
  IN_CLUSTER
};

typedef struct {
  int label;
  int num;
  float* coordinate;
} point;

class ObjectCluster{
  protected:
    int num;
    int num_correct;
    int num_incorrect;
    float* robot_location;
    std::vector<point> list;
    bool correct;
  
  public:
    ObjectCluster(int num);

    ObjectCluster(const ObjectCluster& old_cluster); // copy constructor 

    ~ObjectCluster();

    void add_to_list(point p);

    int cluster_num();

    int size();

    point get_point(int pos);

    int get_correct();

    int get_incorrect();
    
    float* get_robot_location();

    bool get_verification();

    void set_num_correct(int num);

    void set_num_incorrect(int num);

    void set_robot_location(float* point);

    void set_verification(bool ver);
};

#endif