#include <bwi_scavenger/mapping.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <stdexcept>

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

int OrderedLocationSet::get_laps() {
  return index / locations.size();
}

void PriorityLocationSet::set_location_priority(environment_location l,
    float p) {
  priorities[l] = p;
}

void PriorityLocationSet::prioritize() {
  if (priorities.size() != locations.size())
    throw std::runtime_error("Cannot prioritize; cardinality mismatch");

  std::vector<PriorityLocation> p_locations;

  for (const auto &pair : priorities) {
    PriorityLocation pl;
    pl.location = pair.first;
    pl.priority = priorities[pair.first];
    p_locations.push_back(pl);
  }

  std::sort(p_locations.begin(), p_locations.end());
  locations.clear();

  for (PriorityLocation pl : p_locations)
    locations.push_back(pl.location);
}
