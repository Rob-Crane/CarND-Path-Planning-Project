#pragma once
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "traj_types.h"

using std::string;
using std::vector;
namespace path_planner {

struct BasicMap {
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
};

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

struct ControllerInput {
  vector<double> next_x_vals;
  vector<double> next_y_vals;
};

// Controller (simulator) takes input as x,y vehicle position over a time
// horizon.
ControllerInput controller_from_trajectory(const Trajectory& traj,
                                           std::chrono::time_point begin,
                                           milliseconds dt,
                                           milliseconds horizon) {
  ControllerInput input;
}

}  // path_planner
