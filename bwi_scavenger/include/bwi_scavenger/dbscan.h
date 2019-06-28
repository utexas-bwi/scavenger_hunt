#ifndef DBSCAN_H
#define DBSCAN_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <bwi_scavenger/file_editor.h>
#include <limits.h>

enum label{
  UNDEFINED,
  NOISE,
  IN_CLUSTER
};

typedef struct {
  std::vector<double> coordinate;
  // int verification;
  int label;
  // int num;
} point;

typedef struct{
  std::vector<point> list;
  int num;
  int num_correct;
  int num_incorrect;
} cluster;

class Clusterer{
protected:
  std::vector<point> database;
  // std::vector<geometry_msgs::Point> robot_positions;
  double eps;
  int minPoints;
  std::vector<cluster> cluster_list;

public:
  Clusterer(std::vector<std::vector<double>> db, int numDimensions);

  ~Clusterer();

  int get_clusters(double eps, int minPoints);

  std::vector<int> get_incorrect();

  std::vector<int> get_correct();

  // geometry_msgs::Point closest_correct(geometry_msgs::Pose robot_pose);

  bool in_cluster(std::vector<double> point, int cluster_num);

  cluster get_cluster(int cluster_num);

  int get_largest_cluster();
};

#endif