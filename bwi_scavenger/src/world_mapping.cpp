#include "bwi_scavenger/world_mapping.h"

std::map<EnvironmentLocation, coordinates_t> WORLD_WAYPOINTS_SIM {
  {KITCHEN,               { 30.9585, 105.623   }},
  {SOCCER_LAB_DOOR_NORTH, { 48.2878, 112.392   }},
  {SOCCER_LAB_DOOR_SOUTH, { 48.3504, 105.226   }},
  {FELLOW_COMPUTERS, {0, 0}},

  {GRAD_CUBICLES_MIDDLE,  { 14.142,  108.77    }},
  {WHITEBOARD,            { 8.1891,  109.09    }},
  {CONFERENCE_ROOM,       { 35.281,  102.1     }}, // x is estimated

  {LEFT_GC_0,             { 12.959,  112.61    }},
  {WITHIN_GC_0,           { 14.932,  117.42    }},
  {RIGHT_GC_0,            { 16.978,  112.5     }},

  {LEFT_GC_1,             { 12.701,  104.81    }},
  {WITHIN_GC_1,           { 14.636,  99.798    }},
  {RIGHT_GC_1,            { 16.44,   105       }},

  // {LEFT_GC_2,             { 47.569,  104.71    }}, // generalized to SOCCER_LAB_DOOR_SOUTH
  {WITHIN_GC_2,           { 45.426,  100.25    }},
  {RIGHT_GC_2,            { 43.271,  104.8     }},

  {LEFT_GC_3,             { 36.157,  112.74    }},
  {WITHIN_GC_3,           { 37.964,  117.56    }},
  {RIGHT_GC_3,            { 39.742,  112.51    }}
};

std::map<EnvironmentLocation, coordinates_t> WORLD_WAYPOINTS_IRL {
  // TODO make this the actual points
  {KITCHEN,               { 30.9585, 105.623   }},
  {SOCCER_LAB_DOOR_NORTH, { 48.2878, 112.392   }},
  {SOCCER_LAB_DOOR_SOUTH, { 48.3504, 105.226   }},
  {FELLOW_COMPUTERS, {0, 0}},

  {GRAD_CUBICLES_MIDDLE,  { 14.142,  108.77    }},
  {WHITEBOARD,            { 8.1891,  109.09    }},
  {CONFERENCE_ROOM,       { 35.281,  102.1     }}, // x is estimated

  {LEFT_GC_0,             { 12.959,  112.61    }},
  {WITHIN_GC_0,           { 14.932,  117.42    }},
  {RIGHT_GC_0,            { 16.978,  112.5     }},

  {LEFT_GC_1,             { 12.701,  104.81    }},
  {WITHIN_GC_1,           { 14.636,  99.798    }},
  {RIGHT_GC_1,            { 16.44,   105       }},

  // {LEFT_GC_2,             { 47.569,  104.71    }}, // generalized to SOCCER_LAB_DOOR_SOUTH
  {WITHIN_GC_2,           { 45.426,  100.25    }},
  {RIGHT_GC_2,            { 43.271,  104.8     }},

  {LEFT_GC_3,             { 36.157,  112.74    }},
  {WITHIN_GC_3,           { 37.964,  117.56    }},
  {RIGHT_GC_3,            { 39.742,  112.51    }}
};
