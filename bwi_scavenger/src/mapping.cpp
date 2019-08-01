#include <bwi_scavenger/mapping.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <stdexcept>

LocationSet::LocationSet() {
  index = 0;
}

void LocationSet::add_location(environment_location l) {
  coordinates c = environment_location_coordinates[l];
  locations.push_back(c);
}

void LocationSet::add_location(coordinates c) {
  locations.push_back(c);
}

int LocationSet::get_laps(){
  return (index - starting_index) / locations.size();
}

void LocationSet::start(environment_location l) {
  coordinates c = environment_location_coordinates[l];
  for (int i = 0; i < locations.size(); i++)
    if (locations[i].x == c.x && locations[i].y == c.y) {
      index = i;
      return;
    }
}

void LocationSet::start(coordinates c) {
  float shortest_distance = -1;
  int closest_ind = index;

  for (int i = 0; i < locations.size(); i++) {
    coordinates cur_coord = locations[i];
    float dist = sqrt(pow(cur_coord.x - c.x, 2) + pow(cur_coord.y - c.y, 2));
    if (shortest_distance == -1 || dist < shortest_distance) {
      closest_ind = i;
      shortest_distance = dist;
    }
  }

  starting_index = closest_ind;
  index = starting_index;
}

coordinates OrderedLocationSet::get_next_location() {
  coordinates c = locations[index % locations.size()];
  index++;
  return c;
}
