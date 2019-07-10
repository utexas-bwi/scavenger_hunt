#include <bwi_scavenger/dbscan_object.h>

void generateCluster(float** points, int numPoints, int start){
  for(int i = 0; i < numPoints; i++){
    float* point = new float[2];
    *point = rand() % 75 + start;
    *(point + 1) = rand() % 75 + start;
    *(points + i + start) = point;
  }
}

int main(int argc, char **argv){

  float* points[1500];
  for(int i = 0; i < 10; i++)
    generateCluster(points, 100, 100 * i);
  for(int i = 0 ; i < 500; i++){
    float* point = new float[2];
    *point = rand() % 1000;
    *(point + 1) = rand() % 1000;
    *(points + i + 1000) = point;  
  }

  // Clusterer<Cluster> c(points, 2, 1500);
  // std::cout << "Created clusterer" << std::endl;

  // int num_clusters = c.generate_clusters(50, 80);

  // for(int i = 0; i < num_clusters; i++){
  //   Cluster clus = c.get_cluster(i);
  //   std::cout << "Cluster number " << std::to_string(i)<< " size " << std::to_string(clus.size()) << std::endl;
  //   for(int k = 0; k < clus.size(); k++)
  //     std::cout << "(" << std::to_string(clus.get_point(k).coordinate[0]) << ", " << std::to_string(clus.get_point(k).coordinate[1]) << ") " << 
  //     std::to_string(clus.get_point(k).num) << std::endl;
  //   std::cout << std::endl;
  // }

  // std::cout << "There were " << std::to_string(num_clusters) << " clusters in this data set" << std::endl;

  // int largest_cluster = c.get_largest_cluster().cluster_num();

  // std::cout << "The largest cluster was cluster number " << std::to_string(largest_cluster) << std::endl;


  // for(int i = 0; i < 100; i++){
  //   float* point = new float[2];
  //   *point = rand() % 1000;
  //   *(point + 1) = rand() % 1000;
  //   if(c.in_cluster(point, largest_cluster)){
  //     std::cout << "The point was in the cluster!" << std::endl;
  //   }
  // }

  //ObjectClusterer

  float* robot_points[1500];
  for(int i = 0; i < 10; i++)
    generateCluster(robot_points, 100, 100 * i);
  for(int i = 0 ; i < 500; i++){
    float* point = new float[2];
    *point = rand() % 1000;
    *(point + 1) = rand() % 1000;
    *(robot_points + i + 1000) = point;  
  }

  bool verification[1500];
  for(int i = 0; i < 1500; i++)
    *(verification + i) = rand() % 2;

  ObjectClusterer oc(points, robot_points, verification, 1500);
  std::cout << "Created ObjectClusterer" << std::endl;

  std::vector<int> incorrect = oc.get_incorrect_clusters();
  
  std::cout << "There were " << std::to_string(incorrect.size()) << " 'incorrect' clusters" << std::endl;

  for(int i = 0; i < 20; i++){
    float* point = new float[2];
    *point = rand() % 1000;
    *(point + 1) = rand() % 1000;
    float* close = oc.closest_correct(point);
    std::cout << "The closest correct cluster from (" << std::to_string(point[0]) << ", " << std::to_string(point[1])
    << ") is associated with the robot location (" << std::to_string(close[0]) << ", " << std::to_string(close[1]) << ")" << std::endl;
  }
}