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
  KITCHEN,
  SOCCER_LAB_DOOR_NORTH,
  SOCCER_LAB_DOOR_SOUTH,
  FELLOW_COMPUTERS,

  GRAD_CUBICLES_MIDDLE,
  WHITEBOARD,
  CONFERENCE_ROOM,

  LEFT_GC_0,
  WITHIN_GC_0,
  RIGHT_GC_0,

  LEFT_GC_1,
  WITHIN_GC_1,
  RIGHT_GC_1,

  WITHIN_GC_2,
  RIGHT_GC_2,

  LEFT_GC_3,
  WITHIN_GC_3,
  RIGHT_GC_3
 };

/**
 * Simulation waypoint coordinates
 */
extern std::map<EnvironmentLocation, coordinates_t> WORLD_WAYPOINTS_SIM;

/**
 * Real life waypoint coordinates
 */
extern std::map<EnvironmentLocation, coordinates_t> WORLD_WAYPOINTS_IRL;

/**
 * Marks adjacent locations
 */
extern std::map<EnvironmentLocation, std::vector<EnvironmentLocation>> WORLD_CONNECTIONS;

#endif
