#ifndef BWI_SCAVENGER_PATH_PLANNING_H
#define BWI_SCAVENGER_PATH_PLANNING_H

#include "bwi_scavenger/world_mapping.h"

/**
  Interface for a set of locations, likely a circuit being traveled by a robot.
*/
class LocationSet {
protected:
  std::vector<coordinates_t> locations;
  int index, starting_index;
  std::map<EnvironmentLocation, coordinates_t>* world_waypoints;

public:
  LocationSet();

  LocationSet(World w);

  ~LocationSet() {}

  /**
    @brief adds location l to the set
  */
  virtual void add_location(EnvironmentLocation l);

  /**
    @brief adds a coordinates_t location c to the set
  */
  virtual void add_location(coordinates_t c);

  /**
    @brief sets the initial location to location l
  */
  virtual void start(EnvironmentLocation l);

  /**
    @brief starts the set at the location closest to the some coordinates
  */
  virtual void start(coordinates_t c);

  /**
  @brief advances the current position within the set
  */
  virtual coordinates_t get_next_location() = 0;

  /**
    @brief gets the number of laps traveled so far in the circuit
  */
  int get_laps();

  /**
   * @brief gets the number of locations in the set
   */
  std::size_t size();
};

/**
  A location set that orders its waypoints according to the order in which they
  are added to the set.
*/
class OrderedLocationSet : public LocationSet {
public:
  OrderedLocationSet();

  OrderedLocationSet(World w);

  /**
    @brief gets the next location in order; passing the last location loops back
           to the beginning
  */
  coordinates_t get_next_location() override;

  /**
   * @brief shuffles the location set randomly
   */
  void shuffle();

};

#endif
