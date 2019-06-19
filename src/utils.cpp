#include "utils.h"

#include <chrono>
#include <fstream>
#include <sstream>
#include <string>

#include <execinfo.h>
#include <stdio.h>
namespace path_planner {

using std::string;

BasicMap load_map() {
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  BasicMap ret;

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    ret.map_waypoints_x.push_back(x);
    ret.map_waypoints_y.push_back(y);
    ret.map_waypoints_s.push_back(s);
    ret.map_waypoints_dx.push_back(d_x);
    ret.map_waypoints_dy.push_back(d_y);
  }
  return ret;
}

void exitWithStackTrace() {
  void* callstack[128];
  int i, frames = backtrace(callstack, 128);
  char** strs = backtrace_symbols(callstack, frames);
  for (i = 0; i < frames; ++i) {
    printf("%s\n", strs[i]);
  }
  free(strs);
  std::exit(1);
}

}  // path_planner
