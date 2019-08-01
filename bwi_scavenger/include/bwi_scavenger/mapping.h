#ifndef BWI_SCAVENGER_MAPPING_H
#define BWI_SCAVENGER_MAPPING_H

#include <bwi_scavenger/robot_motion.h>
#include <map>
#include <vector>

/**
  Interface for a set of locations, likely a circuit being traveled by a robot.
*/
class LocationSet {
protected:
  std::vector<coordinates> locations;
  int index, starting_index;

public:
  LocationSet();

  ~LocationSet() {}

  /**
    @brief adds location l to the set
  */
  virtual void add_location(environment_location l);

  /**
    @brief adds a coordinates location c to the set
  */
  virtual void add_location(coordinates c);

  /**
    @brief sets the initial location to location l
  */
  virtual void start(environment_location l);

  /**
    @brief starts the set at the location closest to the some coordinates
  */
  virtual void start(coordinates c);

  /**
  @brief advances the current position within the set
  */
  virtual coordinates get_next_location() = 0;

  /**
    @brief gets the number of laps traveled so far in the circuit
  */
  int get_laps();
};

/**
  A location set that orders its waypoints according to the order in which they
  are added to the set.
*/
class OrderedLocationSet : public LocationSet {
public:
  /**
    @brief gets the next location in order; passing the last location loops back
           to the beginning
  */
  coordinates get_next_location() override;

};

#endif
