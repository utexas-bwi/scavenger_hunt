#include <bwi_scavenger/robot_motion.h>
#include <vector>

class LocationSet {
protected:
  std::vector<environment_location> locations;
  int index;

public:
  LocationSet();

  ~LocationSet() {}

  void add_location(environment_location l);

  void start(environment_location l);

  void start(float x, float y);

  virtual environment_location get_next_location() = 0;
};

class OrderedLocationSet : public LocationSet {
public:
  environment_location get_next_location();
};
