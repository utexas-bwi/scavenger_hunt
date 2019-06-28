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
  float* coordinate;
  // int verification;
  int label;
  // int num;
} point;

typedef struct{
  std::vector<point> list;
  int num;
} cluster;

class Clusterer{
protected:
  point* database;
  // std::vector<geometry_msgs::Point> robot_positions;
  float eps;
  int minPoints;
  int size_of_database;
  int num_dimensions;
  std::vector<cluster> cluster_list;

public:
  Clusterer(float** db, int sizeOfDatabase, int numDimensions);

  ~Clusterer();

  int get_clusters(float eps, int minPoints);

  bool in_cluster(float* point, int cluster_num);

  cluster get_cluster(int cluster_num);

  int get_largest_cluster();
};

#endif