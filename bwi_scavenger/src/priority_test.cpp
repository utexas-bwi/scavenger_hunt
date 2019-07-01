#include <bwi_scavenger/mapping.h>

int main(int argc, char **argv) {
  PriorityLocationSet set;
  set.add_location(BWI_LAB_RIGHT);
  set.set_location_priority(BWI_LAB_RIGHT, 3);
  set.add_location(KITCHEN);
  set.set_location_priority(KITCHEN, -8);
  set.add_location(FELLOW_COMPUTERS);
  set.set_location_priority(FELLOW_COMPUTERS, 21);
  set.prioritize();
}
