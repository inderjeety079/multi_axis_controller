#include "GlobalMap.h"
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <string.h>
#include <string>

GlobalMap::GlobalMap() {

  ROS_INFO("GlobalMap Constructor");
  LoadMap();
}

GlobalMap::~GlobalMap() {}

// void GlobalMap::LoadMap() {
//
//   // global_map_filehandle = YAML::LoadFile("global_map.yaml");
//   // YAML::Node characterType;
//
//   std::ifstream fin("global_map.yaml");
//   YAML::Parser parser(fin);
//   YAML::Node doc;
//   parser.GetNextDocument(doc);
//   global_map_ global_map_data;
//
//   for (int i = 0; i < doc.size(); i++) {
//     doc[i] >> global_map_data;
//     global_map_v.push_back(global_map_data);
//   }
// }
//
// global_map_ *global_map = &global_map_v[0];
// global_map_size = sizeof(global_map);
// }

void GlobalMap::LoadMap() {

  ROS_INFO("Load Map");
  global_map_ global_map_data;
  std::ifstream fin("/home/gor/butler_navigation/src/rt_interface/src/"
                    "global_map_for_move.csv");
  // std::ifstream fin("test.csv");
  // std::cout<<fin;
  std::string item;
  std::string line;
  std::getline(fin, line);
  std::cout << "str len: " << strlen(line.c_str()) << std::endl;
  std::cout << "line: " << line << " len : " << line.length() << std::endl;
  while (std::getline(fin, line)) {
    std::istringstream in(line);
    std::cout << "line: " << line << "\n";
    int i = 0;
    while (std::getline(in, item, ',')) {
      i++;
      if (i == 1) {
        global_map_data.row = atoi(item.c_str());
        // S_INFO("item = %s",item.c_str());
      } else if (i == 2) {
        global_map_data.col = atoi(item.c_str());
      } else if (i == 3) {
        global_map_data.x = atof(item.c_str());
      } else if (i == 4) {
        global_map_data.y = atof(item.c_str());
      } else if (i == 5) {
        global_map_data.obstacle = atof(item.c_str());
      }
    }

    global_map.push_back(global_map_data);
  }
  global_map_size = global_map.size();
  ROS_INFO("Global Map size = %d", global_map_size);
  for (int i = 0; i < global_map_size; i++) {
    ROS_INFO("global map: [%d,%d] = [%f]\t[%f]", global_map[i].row,
             global_map[i].col, global_map[i].x, global_map[i].y);
  }
}

void GlobalMap::GetGlobalPosition(uint8_t &row, uint8_t &col) {

  for (int i = 0; i < global_map_size; i++) {

    if (col == global_map[i].col) {

      if (row == global_map[i].row) {

        global_position.x = global_map[i].x;
        global_position.y = global_map[i].y;
        // ROS_INFO("GP : %f\t%f", global_position.x, global_position.y);
      }
    }
  }
}
