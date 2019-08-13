#include <algorithm>
#include <iostream>
#include <math.h>
#include <random>

#include "bwi_scavenger/path_planning.h"
#include "bwi_scavenger/world_mapping.h"

LocationSet::LocationSet() {
  index = 0;
}

LocationSet::LocationSet(World w) : LocationSet() {
  if (w == SIM)
    world_waypoints = &WORLD_WAYPOINTS_SIM;
  else if (w == IRL)
    world_waypoints = &WORLD_WAYPOINTS_IRL;
}

void LocationSet::add_location(EnvironmentLocation l) {
  coordinates_t c = (*world_waypoints)[l];
  locations.push_back(c);
}

void LocationSet::add_location(coordinates_t c) {
  locations.push_back(c);
}

int LocationSet::get_laps() {
  return (index - starting_index) / locations.size();
}

void LocationSet::start(EnvironmentLocation l) {
  coordinates_t c = (*world_waypoints)[l];
  for (int i = 0; i < locations.size(); i++)
    if (locations[i].x == c.x && locations[i].y == c.y) {
      index = i;
      return;
    }
}

void LocationSet::start(coordinates_t c) {
  float shortest_distance = -1;
  int closest_ind = index;

  for (int i = 0; i < locations.size(); i++) {
    coordinates_t cur_coord = locations[i];
    float dist = sqrt(pow(cur_coord.x - c.x, 2) + pow(cur_coord.y - c.y, 2));
    if (shortest_distance == -1 || dist < shortest_distance) {
      closest_ind = i;
      shortest_distance = dist;
    }
  }

  starting_index = closest_ind;
  index = starting_index;
}

std::size_t LocationSet::size() {
  return locations.size();
}

OrderedLocationSet::OrderedLocationSet() : LocationSet() {}

OrderedLocationSet::OrderedLocationSet(World w) : LocationSet(w) {}

coordinates_t OrderedLocationSet::get_next_location() {
  coordinates_t c = locations[index % locations.size()];
  index++;
  return c;
}

void OrderedLocationSet::shuffle() {
  auto rng = std::default_random_engine {};
  std::shuffle(locations.begin(), locations.end(), rng);
}
