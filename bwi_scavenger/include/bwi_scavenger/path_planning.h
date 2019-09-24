#ifndef BWI_SCAVENGER_PATH_PLANNING_H
#define BWI_SCAVENGER_PATH_PLANNING_H

#include "bwi_scavenger/world_mapping.h"

class LocationEvaluator {
protected:
  std::vector<EnvironmentLocation> locations;
  std::map<EnvironmentLocation, coordinates_t>* world_waypoints;
  std::map<EnvironmentLocation, std::vector<std::string>> object_db;

  std::size_t loc_index;

public:
  LocationEvaluator(World w);

  virtual void add_location(EnvironmentLocation loc);

  virtual void add_object(EnvironmentLocation loc, std::string label);

  virtual EnvironmentLocation get_location(
    const std::vector<std::string>& remaining_objects,
    coordinates_t coords_current
  ) = 0;

  EnvironmentLocation get_closest_location(coordinates_t c, bool start = false);
};

class CompleteLocationEvaluator : public LocationEvaluator {
public:
  CompleteLocationEvaluator(World w);

  EnvironmentLocation get_location(const std::vector<std::string>& remaining_objects,
                                   coordinates_t coords_current);
};

class OccupancyGridLocationEvaluator : public LocationEvaluator {
protected:
  std::map<EnvironmentLocation, bool> visited;
  CompleteLocationEvaluator fallback_eval;
  bool fallback_started = false;

  void start_fallback(coordinates_t coords_current);

public:
  OccupancyGridLocationEvaluator(World w);

  void add_location(EnvironmentLocation loc);

  void add_object(EnvironmentLocation loc, std::string label);

  EnvironmentLocation get_location(const std::vector<std::string>& remaining_objects,
                                   coordinates_t coords_current);
};

class ProximityBasedLocationEvaluator : public OccupancyGridLocationEvaluator {
public:
  EnvironmentLocation get_location(const std::vector<std::string>& remaining_objects,
                                   coordinates_t coords_current);
};

#endif
