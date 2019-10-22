#include <algorithm>
#include <limits>
#include <iostream>
#include <math.h>
#include <random>

#include "bwi_scavenger/path_planning.h"
#include "bwi_scavenger/world_mapping.h"

LocationEvaluator::LocationEvaluator(World w) {
  if (w == SIM)
    world_waypoints = &WORLD_WAYPOINTS_SIM;
  else if (w == IRL)
    world_waypoints = &WORLD_WAYPOINTS_IRL;
}

void LocationEvaluator::add_location(EnvironmentLocation loc) {
  locations.push_back(loc);
  object_db[loc] = std::vector<std::string>();
}

void LocationEvaluator::add_object(EnvironmentLocation loc, std::string label) {
  object_db[loc].push_back(label);
}

EnvironmentLocation LocationEvaluator::get_closest_location(coordinates_t c,
                                                            bool start)
{
  float closest_dist = std::numeric_limits<float>::infinity();
  std::size_t closest_ind = 0;

  for (std::size_t i = 0; i < locations.size(); i++) {
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

/*******************************************************************************
 * COMPLETE BASELINE ALGORITHM
 * Simply goes to the next location in the path. Does not consider proximity
 * nor probabilities.
 ******************************************************************************/

CompleteLocationEvaluator::CompleteLocationEvaluator(World w) :
  LocationEvaluator(w) {}

EnvironmentLocation CompleteLocationEvaluator::get_location(
  const std::vector<std::string>& remaining_objects,
  coordinates_t coords_current)
{
  EnvironmentLocation loc = locations[loc_index % locations.size()];
  loc_index++;
  return loc;
}

/*******************************************************************************
 * OCCUPANCY GRID BASELINE ALGORITHM
 * Goes to the location with the highest probabilty of finding the rest of 
 * the objects. 
 * Objects are considered to be of the same value, thus the location with the 
 * most objects would be chosen
 ******************************************************************************/

OccupancyGridLocationEvaluator::OccupancyGridLocationEvaluator(World w) :
    fallback_eval(w),
    LocationEvaluator(w) {}

void OccupancyGridLocationEvaluator::add_location(EnvironmentLocation loc) {
  LocationEvaluator::add_location(loc);
  fallback_eval.add_location(loc);
}

void OccupancyGridLocationEvaluator::add_object(EnvironmentLocation loc, std::string label) {
  object_db[loc].push_back(label);
  fallback_eval.add_object(loc, label);
  visited[loc] = false;
}

void OccupancyGridLocationEvaluator::start_fallback(coordinates_t coords_current) {
  if (!fallback_started) {
    fallback_eval.get_closest_location(coords_current, true);
    fallback_started = true;
  }
}

EnvironmentLocation OccupancyGridLocationEvaluator::get_location(
  const std::vector<std::string>& remaining_objects,
  coordinates_t coords_current)
{
  if (remaining_objects.size() == 0)
    start_fallback(coords_current);

  if (fallback_started)
    return fallback_eval.get_location(remaining_objects, coords_current);

  std::map<EnvironmentLocation, unsigned int> occurrences;

  for (const EnvironmentLocation& loc : locations)
    occurrences[loc] = 0;

  for (const EnvironmentLocation& loc : locations) {
    const std::vector<std::string>& loc_objects = object_db[loc];

    for (const std::string& obj : remaining_objects)
      if (std::find(loc_objects.begin(), loc_objects.end(), obj) != loc_objects.end())
        occurrences[loc]++;
  }

  EnvironmentLocation best_loc;
  unsigned int best_loc_score = 0;

  for (const EnvironmentLocation& loc : locations) {
    unsigned int score = occurrences[loc];

    if (score > best_loc_score && !visited[loc]) {
      best_loc = loc;
      best_loc_score = score;
    }
  }

  if (best_loc_score == 0) {
    start_fallback(coords_current);
    EnvironmentLocation l = fallback_eval.get_location(remaining_objects, coords_current);
    return l;
  }

  visited[best_loc] = true;

  return best_loc;
}

/*******************************************************************************
 * PROXIMITY BASELINE ALGORITHM
 * Goes to the next closest location that still contains an object that has not
 * been found.
 ******************************************************************************/

EnvironmentLocation ProximityBasedLocationEvaluator::get_location(
  const std::vector<std::string>& remaining_objects,
  coordinates_t coords_current)
{
  if (remaining_objects.size() == 0)
    start_fallback(coords_current);

  if (fallback_started)
    return fallback_eval.get_location(remaining_objects, coords_current);

  EnvironmentLocation current_location = locations[loc_index];

  // determine the closest location that still contains an unfound object
  float closest_dist = std::numeric_limits<float>::infinity();
  std::size_t closest_ind = 0;

  for (std::size_t i = 0; i < locations.size(); i++) {

    // check that this location has an object that has not yet been found
    bool look_here = false;
    const std::vector<std::string>& loc_objects = object_db[current_location];

    for (const std::string& obj : remaining_objects)
      if (std::find(loc_objects.begin(), loc_objects.end(), obj) != loc_objects.end()){
        look_here = true;
	break;
      }

    if(!look_here)
      continue;

    // determine the distance to the current location
    coordinates_t coords = (*world_waypoints)[current_location];
    float dx = coords.x - coords_current.x;
    float dy = coords.y - coords_current.y;
    float dist = sqrt(dx * dx + dy * dy);

    if (dist < closest_dist) {
      closest_dist = dist;
      closest_ind = i;
    }
  }

  loc_index = closest_ind;

  return locations[closest_ind];

}
