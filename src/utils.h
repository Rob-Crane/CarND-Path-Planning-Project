#pragma once
#include <vector>

namespace path_planner {

using std::vector;

struct BasicMap {
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
};

BasicMap load_map();

void exitWithStackTrace();

}  // path_planner
