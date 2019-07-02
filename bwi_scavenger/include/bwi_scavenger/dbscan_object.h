#include <bwi_scavenger/dbscan.h>

#define OBJECT_DIMEN 2
#define EPS 50
#define MIN_POINTS 80
#define INCORRECT_THRESHOLD 0.50
#define CORRECT_THRESHOLD 0.75

class ObjectCluster: public Cluster{
  protected:
    int num_correct;
    int num_incorrect;
  
  public:
    ObjectCluster(int num);

    ~ObjectCluster();

    int get_correct();

    int get_incorrect();

    void set_num_correct(int num);

    void set_num_incorrect(int num);
};

class ObjectClusterer: public Clusterer{
  protected:
    float** object_points;
    float** robot_points;
    int size_of_database;
    int num_clusters;

  public:
    ObjectClusterer(float** object_points, float** robot_points, bool* verification, int size_of_database);

    ~ObjectClusterer();

    std::vector<int> get_incorrect_clusters();

    std::vector<int> get_correct_clusters();

    float* closest_correct(float* robot_position);
};
