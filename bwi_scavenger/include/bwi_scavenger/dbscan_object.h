#include <bwi_scavenger/dbscan.h>

#define OBJECT_DIMEN 2
#define ROBOT_DIMEN 2
#define EPS 50
#define MIN_POINTS 80
#define INCORRECT_THRESHOLD 0.50
#define CORRECT_THRESHOLD 0.75

class ObjectCluster: public Cluster{
  protected:
    int num_correct;
    int num_incorrect;
    float* robot_location;
    bool correct;
  
  public:
    ObjectCluster(int num);

    ~ObjectCluster();

    int get_correct();

    int get_incorrect();
    
    float* get_robot_location();

    bool get_verification();

    void set_num_correct(int num);

    void set_num_incorrect(int num);

    void set_robot_location(float* point);

    void set_verification(bool ver);
};

class ObjectClusterer: public Clusterer{
  protected:
    float** object_points;
    float** robot_points;
    int size_of_database;

  public:
    ObjectClusterer(float** object_points, float** robot_points, bool* verification, int size_of_database);

    ~ObjectClusterer();

    std::vector<int> get_incorrect_clusters();

    std::vector<int> get_correct_clusters();

    float* closest_correct(float* robot_position);
};
