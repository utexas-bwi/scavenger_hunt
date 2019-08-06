#ifndef BWI_SCAVENGER_WORLD_MAPPING_H
#define BWI_SCAVENGER_WORLD_MAPPING_H

#include <map>
#include <vector>

typedef struct {
  float x;
  float y;
} coordinates_t;

enum World {
  SIM,
  IRL
};

/**
 * Named locations of interest around the world
 */
enum EnvironmentLocation {
  BWI_LAB_DOOR_NORTH,
  BWI_LAB_DOOR_SOUTH,
  CLEARING_NORTH,
  CLEARING_SOUTH,
  ALCOVE,
  KITCHEN,
  SOCCER_LAB_DOOR_NORTH,
  SOCCER_LAB_DOOR_SOUTH,
  FELLOW_COMPUTERS
 };

/**
 * Simulation waypoint coordinates
 */
extern std::map<EnvironmentLocation, coordinates_t> WORLD_WAYPOINTS_SIM;

/**
 * Real life waypoint coordinates
 */
extern std::map<EnvironmentLocation, coordinates_t> WORLD_WAYPOINTS_IRL;

#endif
