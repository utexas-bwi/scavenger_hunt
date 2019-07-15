#ifndef DBSCAN_OBJECT_H
#define DBSCAN_OBJECT_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <bwi_scavenger/object_cluster.h>

#define EPS 5 // range of the entire cluster (size)
#define MIN_POINTS 3 // minimum number of points for each cluster

/*
  Clusterer class that can cluster points for object data
*/
class ObjectClusterer{
  protected:
    point* database;
    int size_of_database;
    std::vector<ObjectCluster> cluster_list;

  private:

    /*
      Generates clusters based on the data the user sent in using a 
      DBSCAN algorithm
    */
    int generate_clusters(float eps, int minPoints);

  public:

    /*
      Creates an ObjectClusterer that has generated clusters based on the object_points
      Associates a robot location to the given cluster using robot_points
      Associates the number of correct and incorrect in the cluster using verification
    */
    ObjectClusterer(float** object_points, float** robot_points, bool* verification, int size_of_database);

    ~ObjectClusterer();

    /*
      Returns a list of cluster numbers that have been considered incorrect clusters
    */
    std::vector<int> get_incorrect_clusters();

    /*
      Returns a list of cluster numbers that have been considered correct clusters
    */
    std::vector<int> get_correct_clusters();

    /*
      Returns the location of the closest correct cluster given the 
      current robot position
    */
    float* closest_correct(float* robot_position);
  
    /*
      Returns an ObjectCluster associated with the cluster number
    */
    ObjectCluster* get_cluster(int cluster_num);
  
    /*
      Returns the largest cluster generated with this data set
    */
    ObjectCluster* get_largest_cluster();
      
    /*
      Returns whether or not the point given can be considered a part of this cluster
    */
    bool in_cluster(float* point, int cluster_num);
};

#endif
