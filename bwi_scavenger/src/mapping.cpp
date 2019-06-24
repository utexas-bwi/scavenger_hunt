#include <bwi_scavenger/mapping.h>
#include <math.h>
#include <ros/ros.h>

LocationSet::LocationSet() {
  index = 0;
}

void LocationSet::add_location(environment_location l) {
  locations.push_back(l);
}

void LocationSet::start(environment_location l) {
  for (int i = 0; i < locations.size(); i++)
    if (locations[i] == l) {
      index = i;
      return;
    }
}

void LocationSet::start(float x, float y) {
  float shortest_distance = -1;
  int closest_ind = index;

  for (int i = 0; i < locations.size(); i++) {
    std::pair<float, float> coords =
        environment_location_coordinates[locations[i]];
    float dist = sqrt(pow(coords.first - x, 2) + pow(coords.second - y, 2));
    if (shortest_distance == -1 || dist < shortest_distance) {
      closest_ind = i;
      shortest_distance = dist;
    }
  }

  index = closest_ind;
}

environment_location OrderedLocationSet::get_next_location() {
  environment_location l = locations[index % locations.size()];
  index++;
  return l;
}
