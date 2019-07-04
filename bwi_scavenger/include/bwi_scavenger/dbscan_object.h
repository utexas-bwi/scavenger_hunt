#ifndef DBSCAN_OBJECT_H
#define DBSCAN_OBJECT_H


#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <bwi_scavenger/file_editor.h>
#include <limits.h>
#include <math.h>

#define OBJECT_DIMEN 2
#define ROBOT_DIMEN 2
#define EPS 5
#define MIN_POINTS 3
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

class ObjectClusterer{
  protected:
    float** object_points;
    float** robot_points;
    point* database;
    int size_of_database;
    std::vector<ObjectCluster> cluster_list;

  public:
    ObjectClusterer(float** object_points, float** robot_points, bool* verification, int size_of_database);

    ~ObjectClusterer();

    std::vector<int> get_incorrect_clusters();

    std::vector<int> get_correct_clusters();

    float* closest_correct(float* robot_position);

    int generate_clusters(float eps, int minPoints);
  
    ObjectCluster get_cluster(int cluster_num);
  
    ObjectCluster get_largest_cluster();
      
    bool in_cluster(float* point, int cluster_num);
};

#endif
