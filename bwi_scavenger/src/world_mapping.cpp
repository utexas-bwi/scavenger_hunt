#include "bwi_scavenger/world_mapping.h"

std::map<EnvironmentLocation, coordinates_t> WORLD_WAYPOINTS_SIM {
  {BWI_LAB_DOOR_NORTH,    { 39.6831, 112.569   }},
  {BWI_LAB_DOOR_SOUTH,    { 39.553,  105.192   }},
  {CLEARING_NORTH,        { 14.4326, 112.49    }},
  {CLEARING_SOUTH,        { 14.3866, 105.104   }},
  {ALCOVE,                { 19.6725, 105.155   }},
  {KITCHEN,               { 30.9585, 105.623   }},
  {SOCCER_LAB_DOOR_NORTH, { 48.2878, 112.392   }},
  {SOCCER_LAB_DOOR_SOUTH, { 48.3504, 105.226   }},
  {FELLOW_COMPUTERS,      { 35.8519, 112.632   }}
};

std::map<EnvironmentLocation, coordinates_t> WORLD_WAYPOINTS_IRL {
  {BWI_LAB_DOOR_NORTH,    { -39.3304, -11.7290 }},
  {BWI_LAB_DOOR_SOUTH,    { -38.2976, -4.3244  }},
  {CLEARING_NORTH,        { -13.9990, -12.024  }},
  {CLEARING_SOUTH,        { -13.7639, -5.5648  }},
  {ALCOVE,                { -18.2928, -4.9306  }},
  {KITCHEN,               { -30.1798, -4.5981  }},
  {SOCCER_LAB_DOOR_NORTH, { -47.7036, -11.2160 }},
  {SOCCER_LAB_DOOR_SOUTH, { -47.5810, -4.1472  }},
  {FELLOW_COMPUTERS,      { -35.0182, -11.4877 }},
  {HALLWAY0,              { -41.2976,  -4.3244 }}
};
