#ifndef DBSCAN_OBJECT_H
#define DBSCAN_OBJECT_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <bwi_scavenger/object_cluster.h>

#define EPS 5
#define MIN_POINTS 3

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
