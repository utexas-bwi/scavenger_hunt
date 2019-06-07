#include <bwi_scavenger/robot_motion.h>

static std::string gfid;
static bool set_grid_frame_id = false;

void set_id(const nav_msgs::OccupancyGrid::ConstPtr &grid) {
  if (!set_grid_frame_id)
    gfid = grid->header.frame_id;

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_motion_test");
  ros::NodeHandle nh;
  ros::Subscriber map_sub = nh.subscribe("/level_mux/map", 1, set_id);

  while (!set_grid_frame_id);

  RobotMotion rm(gfid);
  rm.turn(90);

  ros::spin();
  return 0;
}
