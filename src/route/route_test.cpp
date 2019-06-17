#include <iostream>
#include "route_frame.h"
#include "route_smoother.h"
#include "utils.h"

using namespace path_planner;
int main() {
  BasicMap map = load_map();
  std::shared_ptr<RouteFrame> rf = std::make_shared<RouteFrame>(
      map.map_waypoints_x, map.map_waypoints_y, map.map_waypoints_s);
  InertialVector inertial(381.96, 1531.71);
  InertialVector vel(2.0, 0);
  TrajectoryVelocity test_v(inertial, vel, rf);
  RouteVector v_res = test_v.routeV();
  std::cout << "vs: " << v_res.s() << " vd: " << v_res.d() << std::endl;
}
