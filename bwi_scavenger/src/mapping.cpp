#include <bwi_scavenger/mapping.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <stdexcept>

LocationSet::LocationSet() {
  index = 0;
}

void LocationSet::add_location(environment_location l) {
  coordinate c = environment_location_coordinates[l];
  locations.push_back(c);
}

void LocationSet::add_location(coordinate c) {
  locations.push_back(c);
}

// Returns the number of laps done, will -1 if the set does not contain any locations
int LocationSet::get_laps(){
  if(locations.size())
    return index / locations.size();
  return -1;
}

void LocationSet::start(environment_location l) {
  coordinate c = environment_location_coordinates[l];
  for (int i = 0; i < locations.size(); i++)
    if (locations[i] == c) {
      index = i;
      return;
    }
}

void LocationSet::start(coordinate c) {
  float shortest_distance = -1;
  int closest_ind = index;

  for (int i = 0; i < locations.size(); i++) {
    coordinate cur_coord = locations[i];
    float dist = sqrt(pow(cur_coord.first - c.first, 2) + pow(cur_coord.second - c.second, 2));
    if (shortest_distance == -1 || dist < shortest_distance) {
      closest_ind = i;
      shortest_distance = dist;
    }
  }

  index = closest_ind;
}

coordinate OrderedLocationSet::get_next_location() {
  coordinate c = locations[index % locations.size()];
  index++;
  return c;
}


void PriorityLocationSet::set_location_priority(coordinate c,
    float p) {
  priorities[c] = p;
}

void PriorityLocationSet::prioritize() {
  if (priorities.size() != locations.size())
    throw std::runtime_error("Cannot prioritize; cardinality mismatch");

  std::vector<PriorityLocation> p_locations;

  for (const auto &pair : priorities) {
    PriorityLocation pl;
    pl.coor = pair.first;
    pl.priority = priorities[pair.first];
    p_locations.push_back(pl);
  }

  std::sort(p_locations.begin(), p_locations.end());
  locations.clear();

  for (PriorityLocation pl : p_locations){
    locations.push_back(pl.coor);
    std::cout << "Priority List adding (" << pl.coor.first << ", " << pl.coor.second << ")" << std::endl;
  }
}

void PriorityLocationSet::clear(){
  priorities.clear();
  locations.clear();
  index = 0;
}