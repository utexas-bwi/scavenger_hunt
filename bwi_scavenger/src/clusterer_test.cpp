#include <bwi_scavenger/dbscan.h>

int main(int argc, char **argv){

  double* points[1000];
  for(int i = 0; i < 1000; i++){
    double* point = new double[2];
    *point = rand() % 100;
    *(point + 1) = rand() % 100;
    points[i] = point;
  }

  Clusterer c(points, 2, 1000);
  std::cout << "Created clusterer" << std::endl;

  int numClusters = c.get_clusters(5, 10);

  std::cout << "There were " << std::to_string(numClusters) << " clusters in this data set" << std::endl;

  int largest_cluster = c.get_largest_cluster();

  std::cout << "The largest cluster was cluster number " << std::to_string(largest_cluster) << std::endl;


  for(int i = 0; i < 100; i++){
    double* point = new double[2];
    *point = rand() % 100;
    *(point + 1) = rand() % 100;
    if(c.in_cluster(point, largest_cluster)){
      std::cout << "The point was in the cluster!" << std::endl;
    }
  }

}