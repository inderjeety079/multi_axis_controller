#ifndef GLOBAL_MAP_H
#define GLOBAL_MAP_H

#include "RTInterface.h"
#include <inttypes.h>
#include <iostream>
//#include <yaml-cpp/node/impl.h>
#include <yaml-cpp/yaml.h>

class GlobalMap {

private:
  // YAML::Node global_map_filehandle;
  std::vector<global_map_> global_map;
  int global_map_size;
  // global_map_ global_map[938];

public:
  global_position_ global_position;
  GlobalMap();
  ~GlobalMap();
  void LoadMap();
  void GetGlobalPosition(uint8_t &row, uint8_t &col);
};

#endif /*GLOBAL_MAP_H*/
