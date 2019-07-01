#ifndef DBSCAN_H
#define DBSCAN_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <bwi_scavenger/file_editor.h>
#include <limits.h>
#include <math.h>

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

float calculate_distance(float* p1, float* p2, int num_dimensions);

class Cluster{
  protected:
    int num;
    std::vector<point> list;

  public:
    Cluster(int num, point point);

    ~Cluster();

    void add_to_list(point p);

    int cluster_num();

    int size();

    point get_point(int pos);
};

class Clusterer{
protected:
  point* database;
  float eps;
  int minPoints;
  int size_of_database;
  int num_dimensions;
  std::vector<Cluster> cluster_list;

public:
  Clusterer(float** db, int num_dimensions, int size_of_database);

  ~Clusterer();

  int generate_clusters(float eps, int minPoints);

  Cluster get_cluster(int cluster_num);

  Cluster get_largest_cluster();
    
  bool in_cluster(float* point, Cluster cluster);
};

#endif