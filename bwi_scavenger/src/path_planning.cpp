#include <algorithm>
#include <limits>
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

LocationEvaluator::LocationEvaluator(World w) {
  if (w == SIM)
    world_waypoints = &WORLD_WAYPOINTS_SIM;
  else if (w == IRL)
    world_waypoints = &WORLD_WAYPOINTS_IRL;
}

void LocationEvaluator::add_location(EnvironmentLocation loc) {
  locations.push_back(loc);
}

void LocationEvaluator::add_object(EnvironmentLocation loc, std::string label) {
  object_db[loc].push_back(label);
}

EnvironmentLocation LocationEvaluator::get_closest_location(coordinates_t c,
                                                            bool start)
{
  float closest_dist = std::numeric_limits<float>::infinity();
  std::size_t closest_ind = 0;

  for (std::size_t i = 1; i < locations.size(); i++) {
    coordinates_t coords = (*world_waypoints)[locations[i]];
    float dx = coords.x - c.x;
    float dy = coords.y - c.y;
    float dist = sqrt(dx * dx + dy * dy);

    if (dist < closest_dist) {
      closest_dist = dist;
      closest_ind = i;
    }
  }

  if (start)
    loc_index = closest_ind;

  return locations[closest_ind];
}

StupidLocationEvaluator::StupidLocationEvaluator(World w) :
  LocationEvaluator(w) {}

EnvironmentLocation StupidLocationEvaluator::get_location(
  const std::vector<std::string>& remaining_objects,
  coordinates_t coords_current)
{
  EnvironmentLocation loc = locations[loc_index % locations.size()];
  loc_index++;
  return loc;
}

GreedyLocationEvaluator::GreedyLocationEvaluator(World w) :
    fallback_eval(w),
    LocationEvaluator(w) {}

EnvironmentLocation GreedyLocationEvaluator::get_location(
  const std::vector<std::string>& remaining_objects,
  coordinates_t coords_current)
{
  if (remaining_objects.size() == 0) {
    if (!fallback_started) {
      fallback_eval.get_closest_location(coords_current, true);
      fallback_started = true;
    }

    return fallback_eval.get_location(remaining_objects, coords_current);
  }


  std::map<EnvironmentLocation, unsigned int> occurrences;

  for (const EnvironmentLocation& loc : locations)
    occurrences[loc] = 0;

  for (const EnvironmentLocation& loc : locations)
    for (const std::string& obj : remaining_objects) {
      const std::vector<std::string>& loc_objects = object_db[loc];
      if (std::find(loc_objects.begin(), loc_objects.end(), obj) != loc_objects.end())
        occurrences[loc]++;
    }

  EnvironmentLocation best_loc;
  unsigned int best_loc_score = -1;

  for (const EnvironmentLocation& loc : locations) {
    unsigned int score = occurrences[loc];

    if (score > best_loc_score) {
      best_loc = loc;
      best_loc_score = score;
    }
  }

  return best_loc;
}
