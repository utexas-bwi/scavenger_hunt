#include <bwi_scavenger/dbscan.h>

int main(int argc, char **argv){

  std::vector<std::vector<double>> points;
  std::vector<double> point;
  point.push_back(rand() % 100);
  point.push_back(rand() % 100);
  for(int i = 0; i < 1000; i++){
    point[0] = rand() % 100;
    point[1] = rand() % 100;
    points.push_back(point);
  }

  Clusterer c(points, 2);
  std::cout << "Created clusterer" << std::endl;

  int numClusters = c.get_clusters(5, 10);

  std::cout << "There were " << std::to_string(numClusters) << " clusters in this data set" << std::endl;

  int largest_cluster = c.get_largest_cluster();

  std::cout << "The largest cluster was cluster number " << std::to_string(largest_cluster) << std::endl;


  for(int i = 0; i < 100; i++){
    point[0] = rand() % 100;
    point[1] = rand() % 100;
    if(c.in_cluster(point, largest_cluster)){
      std::cout << "The point was in the cluster!" << std::endl;
    }
  }

}