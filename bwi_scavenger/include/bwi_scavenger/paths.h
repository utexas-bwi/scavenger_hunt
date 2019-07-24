#ifndef BWI_SCAVENGER_PARAMS_H
#define BWI_SCAVENGER_PARAMS_H

#include <ros/ros.h>
#include <string>

#include "bwi_scavenger/globals.h"

static ros::NodeHandle* nh = nullptr;
static std::string ws_path;

namespace paths {

static std::string _ws;
static std::string _proof_db;
static std::string _proof_material;
static std::string _proof_materials_repo;
static std::string _darknet;
static std::string _dnros;

namespace {
  void init() {
    if (nh == nullptr) {
      nh = new ros::NodeHandle();
      nh->param("bwi_scavenger/ws_path", _ws, std::string(""));

      _proof_db = _ws + "/" + PROOF_DATABASE_FNAME;
      _proof_material = _ws + "/" + PROOF_MATERIAL_FNAME;
      _proof_materials_repo = _ws + "/" + PROOF_MATERIALS_REPO_DNAME;
      _darknet = _ws + "/src/darknet_ros/darknet";
      _dnros = _ws + "/src/darknet_ros/darknet_ros";
    }
  }
}

std::string ws() {
  init();

  return _ws;
}

std::string proof_db() {
  init();

  return _proof_db;
}

std::string proof_material() {
  init();

  return _proof_material;
}

std::string proof_materials_repo() {
  init();

  return _proof_materials_repo;
}

std::string darknet() {
  return _darknet;
}

std::string dnros() {
  return _dnros;
}

} // namespace paths

#endif
