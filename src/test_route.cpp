#include <iostream>
#include "route_frame.h"

using namespace path_planner;
int main() {
  RouteFrame rf({0.0, 1.0, 1.0, 0.0}, {0.0, 0.0, 1.0, 1.0},
                {0.0, 1.0, 2.0, 3.0});
  InertialCoordinate inertial(0.5, 1.1);
  auto opt_route = rf.to_route(inertial);
  if (opt_route) {
    std::cout << "s: " << std::to_string(opt_route->s())
              << " d: " << std::to_string(opt_route->d()) << std::endl;
  } else {
    std::cout << "failed to map";
  }
}