#include <bwi_scavenger/robot_motion.h>
#include <map>
#include <vector>

/**
  Interface for a set of locations, likely a circuit being traveled by a robot.
*/
class LocationSet {
protected:
  std::vector<coordinate> locations;
  int index;

public:
  LocationSet();

  ~LocationSet() {}

  /**
    @brief adds location l to the set
  */
  virtual void add_location(environment_location l);

  /**
    @brief adds a coordinate location c to the set
  */
  virtual void add_location(coordinate c);

  /**
    @brief sets the initial location to location l
  */
  virtual void start(environment_location l);

  /**
    @brief starts the set at the location closest to the coordinate point
  */
  virtual void start(coordinate c);

  /**
  @brief advances the current position within the set
  */
  virtual coordinate get_next_location() = 0;
  
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
  coordinate get_next_location() override;

};

/**
  A location with some associated priority scalar.
*/
struct PriorityLocation {
  coordinate coor;
  float priority;

  bool operator<(const PriorityLocation& str) const {
    return priority > str.priority;
  }
};

/**
  A location set ordered by priority.
*/
class PriorityLocationSet : public OrderedLocationSet {
protected:
  std::map<coordinate, float> priorities;

public:
  /**
    @brief sets the priority of location l to p
  */
  void set_location_priority(coordinate c, float p);

  /**
    @brief sorts the internal circuit by priority; will fail if not every
           location has been assigned a priority
  */
  void prioritize();

  /**
    @brief deletes the current map of location sets
  */
  void clear();
};
