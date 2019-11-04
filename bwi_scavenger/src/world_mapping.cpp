#include "bwi_scavenger/world_mapping.h"

std::map<std::string, EnvironmentLocation> location_names {
  {"kitchen", KITCHEN},
  {"gc_middle", GRAD_CUBICLES_MIDDLE},
  {"whiteboard", WHITEBOARD},
  {"gc_0", WITHIN_GC_0},
  {"gc_1", WITHIN_GC_1},
  {"gc_2", WITHIN_GC_2},
  {"gc_3", WITHIN_GC_3}
};

std::map<EnvironmentLocation, coordinates_t> WORLD_WAYPOINTS_SIM {
  {KITCHEN,               { 30.9585, 105.623   }},
  {FELLOW_COMPUTERS,      { 35.8519, 112.632   }},

  {GRAD_CUBICLES_MIDDLE,  { 14.142,  108.77    }},
  {WHITEBOARD,            { 8.1891,  109.09    }},

  {WITHIN_GC_0,           { 14.932,  117.42    }},
  {WITHIN_GC_1,           { 14.636,  99.798    }},
  {WITHIN_GC_2,           { 45.426,  100.25    }},
  {WITHIN_GC_3,           { 37.964,  117.56    }},
};


std::map<EnvironmentLocation, coordinates_t> WORLD_WAYPOINTS_IRL {
  {KITCHEN,               { -30.1798, -4.5981  }},
  {FELLOW_COMPUTERS,      { -35.0182, -11.4877 }}, 
  {WHITEBOARD,            { -6.9854, -8.6477    }},

  {GRAD_CUBICLES_MIDDLE,  { -13.516, -7.8491    }}, // TODO split this into 2 coordinates

  {WITHIN_GC_0,           { -14.035, -16.539    }},
  {WITHIN_GC_1,           { -13.774, -0.86683   }},
  {WITHIN_GC_2,           { -45.102, 0.1126     }},
  {WITHIN_GC_3,           { -37.18, -15.252    }},
};

std::map<EnvironmentLocation, std::vector<EnvironmentLocation>> WORLD_CONNECTIONS {

  {KITCHEN,               { WITHIN_GC_2, WITHIN_GC_1 }},
  {FELLOW_COMPUTERS,      { }}, 

  {GRAD_CUBICLES_MIDDLE,  { WITHIN_GC_0, WITHIN_GC_1 }},
  {WHITEBOARD,            { WITHIN_GC_0, WITHIN_GC_1 }},

  {WITHIN_GC_0,           { WHITEBOARD, GRAD_CUBICLES_MIDDLE }},
 
  {WITHIN_GC_1,           { WHITEBOARD, KITCHEN, GRAD_CUBICLES_MIDDLE, WITHIN_GC_3 }},

  {WITHIN_GC_2,           { WITHIN_GC_1, KITCHEN, WITHIN_GC_3 }},

  {WITHIN_GC_3,           { WITHIN_GC_0, WITHIN_GC_2 }},

};

// std::map<EnvironmentLocation, coordinates_t> WORLD_WAYPOINTS_SIM {
//   {KITCHEN,               { 30.9585, 105.623   }},
//   {SOCCER_LAB_DOOR_NORTH, { 48.2878, 112.392   }},
//   {SOCCER_LAB_DOOR_SOUTH, { 48.3504, 105.226   }},
//   {FELLOW_COMPUTERS,      { 35.8519, 112.632   }},

//   {GRAD_CUBICLES_MIDDLE,  { 14.142,  108.77    }},
//   {WHITEBOARD,            { 8.1891,  109.09    }},
//   {CONFERENCE_ROOM,       { 35.281,  102.1     }}, // x is estimated

//   {LEFT_GC_0,             { 12.959,  112.61    }},
//   {WITHIN_GC_0,           { 14.932,  117.42    }},
//   {RIGHT_GC_0,            { 16.978,  112.5     }},

//   {LEFT_GC_1,             { 12.701,  104.81    }},
//   {WITHIN_GC_1,           { 14.636,  99.798    }},
//   {RIGHT_GC_1,            { 16.44,   105       }},

//   // {LEFT_GC_2,             { 47.569,  104.71    }}, // generalized to SOCCER_LAB_DOOR_SOUTH
//   {WITHIN_GC_2,           { 45.426,  100.25    }},
//   {RIGHT_GC_2,            { 43.271,  104.8     }},

//   {LEFT_GC_3,             { 36.157,  112.74    }},
//   {WITHIN_GC_3,           { 37.964,  117.56    }},
//   {RIGHT_GC_3,            { 39.742,  112.51    }}
// };

// std::map<EnvironmentLocation, coordinates_t> WORLD_WAYPOINTS_IRL{
//   {KITCHEN,               { -30.1798, -4.5981  }},
//   {SOCCER_LAB_DOOR_NORTH, { -47.7036, -11.2160 }}, 
//   {SOCCER_LAB_DOOR_SOUTH, { -47.5810, -4.1472  }},  // LEFT GC 2
//   {FELLOW_COMPUTERS,      { -35.0182, -11.4877 }}, 

//   {GRAD_CUBICLES_MIDDLE,  { -13.516, -7.8491    }},
//   {WHITEBOARD,            { -6.9854, -8.6477    }},
//   {CONFERENCE_ROOM,       { -34.103, -0.87973   }},

//   {LEFT_GC_0,             { -11.365, -12.253    }}, // bottom left near justin's office
//   {WITHIN_GC_0,           { -14.035, -16.539    }},
//   {RIGHT_GC_0,            { -16.658, -12.13     }},

//   {LEFT_GC_1,             { -16.578, -4.7619    }}, // bottom right
//   {WITHIN_GC_1,           { -13.774, -0.86683   }},
//   {RIGHT_GC_1,            { -11.084, -4.9393    }},

//   {WITHIN_GC_2,           { -45.102, 0.1126     }}, // top right
//   {RIGHT_GC_2,            { -41.946, -4.2024    }},

//   {LEFT_GC_3,             { -34.426, -11.662   }}, // top left
//   {WITHIN_GC_3,           { -37.18, -15.252    }},
//   {RIGHT_GC_3,            { -40.12, -11.579    }}
// };


// std::map<EnvironmentLocation, std::vector<EnvironmentLocation>> WORLD_CONNECTIONS {

//   {KITCHEN,               { CONFERENCE_ROOM, RIGHT_GC_2, LEFT_GC_1 }},
//   {SOCCER_LAB_DOOR_NORTH, { SOCCER_LAB_DOOR_SOUTH, RIGHT_GC_3 }}, 
//   {SOCCER_LAB_DOOR_SOUTH, { SOCCER_LAB_DOOR_NORTH, WITHIN_GC_2, RIGHT_GC_2 }}, 
//   {FELLOW_COMPUTERS,      { }}, 

//   {GRAD_CUBICLES_MIDDLE,  { LEFT_GC_0, RIGHT_GC_0, LEFT_GC_1, RIGHT_GC_1 }},
//   {WHITEBOARD,            { LEFT_GC_0, RIGHT_GC_1 }},
//   {CONFERENCE_ROOM,       { KITCHEN }},

//   {LEFT_GC_0,             { GRAD_CUBICLES_MIDDLE, WHITEBOARD, WITHIN_GC_0, RIGHT_GC_0 }},
//   {WITHIN_GC_0,           { LEFT_GC_0, RIGHT_GC_0 }},
//   {RIGHT_GC_0,            { GRAD_CUBICLES_MIDDLE, LEFT_GC_0, WITHIN_GC_0, LEFT_GC_3 }},
 
//   {LEFT_GC_1,             { KITCHEN, GRAD_CUBICLES_MIDDLE, WITHIN_GC_1, RIGHT_GC_1, RIGHT_GC_2 }},
//   {WITHIN_GC_1,           { LEFT_GC_1, RIGHT_GC_1 }},
//   {RIGHT_GC_1,            { GRAD_CUBICLES_MIDDLE, WHITEBOARD, LEFT_GC_1 , WITHIN_GC_1 }},

//   {WITHIN_GC_2,           { SOCCER_LAB_DOOR_SOUTH, RIGHT_GC_2 }},
//   {RIGHT_GC_2,            { KITCHEN, SOCCER_LAB_DOOR_SOUTH, LEFT_GC_1 }},

//   {LEFT_GC_3,             { RIGHT_GC_0, WITHIN_GC_3, RIGHT_GC_3 }},
//   {WITHIN_GC_3,           { LEFT_GC_3, RIGHT_GC_3 }},
//   {RIGHT_GC_3,            { SOCCER_LAB_DOOR_NORTH, LEFT_GC_3, WITHIN_GC_3 }}

// };

float** generate_distances(std::map<EnvironmentLocation, coordinates_t> world, ros::ServiceClient client_path){
  const int num_locations = world.size();
  
  geometry_msgs::PoseStamped initial_pose;
  initial_pose.header.stamp = ros::Time::now();
  initial_pose.header.frame_id = "/level_mux_map";

  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.stamp = ros::Time::now();
  goal_pose.header.frame_id = "/level_mux_map";
  
  float** distances = new float* [num_locations]; // array for all of the distances between locations
  for(int i = 0 ; i < num_locations ; i++)
    distances[i] = new float[num_locations];
  
  // Go through every location
  for(int loc_index_1 = 0 ; loc_index_1 < num_locations ; loc_index_1++){
    EnvironmentLocation loc_1 = static_cast<EnvironmentLocation>(loc_index_1);
    coordinates_t loc_coor_1 = world[loc_1];

    distances[loc_index_1][loc_index_1] = 0;

    // Get distances to every other location and set it in the array
    for(int loc_index_2 = loc_index_1 + 1 ; loc_index_2 < num_locations ; loc_index_2++){
      EnvironmentLocation loc_2 = static_cast<EnvironmentLocation>(loc_index_2);
      coordinates_t loc_coor_2 = world[loc_2];

      // Call service to generate plan, returns list of PoseStamped
      std::vector<geometry_msgs::PoseStamped> pose_list;
	    nav_msgs::GetPlan get_plan_srv;

      initial_pose.pose.position.x = loc_coor_1.x;
      initial_pose.pose.position.y = loc_coor_1.y;

      goal_pose.pose.position.x = loc_coor_2.x;
      goal_pose.pose.position.y = loc_coor_2.y;

      get_plan_srv.request.start = initial_pose;
      get_plan_srv.request.goal = goal_pose;
	    get_plan_srv.request.tolerance = -1.0f;

	    client_path.call(get_plan_srv);

	    pose_list = get_plan_srv.response.plan.poses;

      // Get the distance based on the path of defined by the pose list
      int num_poses = pose_list.size();
      const int next_to_last = num_poses - 1;
      float distance = 0;

      for(int i = 0 ; i < next_to_last ; i++){
        geometry_msgs::PoseStamped firstPose = pose_list[i];
        geometry_msgs::PoseStamped secondPose = pose_list[i + 1];

        auto dx = firstPose.pose.position.x - secondPose.pose.position.x;
        auto dy = firstPose.pose.position.y - secondPose.pose.position.y;

        distance += std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
      }

      // Writes distances in array - distance between points is reversible
      distances[loc_index_1][loc_index_2] = distance;
      distances[loc_index_2][loc_index_1] = distance;

    }
  }

  // std::cout << "[generate_distances] " << std::endl;;
  // for(int i = 0 ; i < num_locations ; i++){
  //   for(int j = 0 ; j < num_locations ; j++){
  //     std::cout << distances[i][j] << "  ";
  //   }
  //   std::cout << std::endl;
  // }

  // std::cout << "[generate_distances] done " << std::endl;

  return distances;
}
