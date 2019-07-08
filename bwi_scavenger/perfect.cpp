// If the file is a header, begin with a define guard of the following format.
#ifndef HEADER_FILE_NAME_H
#define HEADER_FILE_NAME_H

// Library imports come first. They should be alphabetized and in angle
// brackets.
#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

// Message imports come next. They should be split up by package name. Packages
// should be listed in alphabetical order, and the messages within them also in
// alphabetical order. These are also in angle brackets.
#include <bwi_scavenger_msgs/RobotMove.h>
#include <bwi_scavenger_msgs/RobotStop.h>

#include <darknet_ros_msgs/BoundingBox.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

// Finally, imports local to the package are listed, this time in quotes.
// As usual, these are alphabetized.
#include "perfect/bar.h";
#include "perfect/foo.h";

// Define any global variables the file needs. If they don't need to travel
// outside this translation unit, they shall be static.
static std::string node_name = "perfect_node";
static ros::Publisher pub_perfect;

// Define your global methods. If they don't need to travel outside this
// translation unit and this file CAN be imported, put them in an anonymous
// namespace.
namespace {
  void private_global_method(int &a) {
    a = 5;
  }
}

void public_global_method(int &a) {
  a = 5;
}

// Define classes.
class PerfectClass {
// The private/protected distinction will depend on your inheritance plans for
// the class.
protected:
  int *arr;

public:
  PerfectClass() {
    arr = new int[10];
  }

  ~PerfectClass() {
    // CLEAN UP AFTER YOURSELF.
    delete[] arr;
  }
}
