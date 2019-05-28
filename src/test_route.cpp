#include <iostream>
#include "route_frame.h"

using namespace path_planner;
int main() {
  RouteFrame rf({0.0, 10.0, 10.0, 0.0}, {0.0, 0.0, 10.0, 10.0},
                {0.0, 10.0, 20.0, 30.0});
  InertialCoordinate inertial(0.5, 1.1);
  auto opt_route = rf.to_route(inertial);
  if (opt_route) {
    std::cout << "s: " << std::to_string(opt_route->s())
              << " d: " << std::to_string(opt_route->d()) << std::endl;
  } else {
    std::cout << "failed to map"<<std::endl;
  }

  RouteCoordinate route(31, 5);
  InertialCoordinate i = rf.to_inertial(route);
  std::cout<<"x: " << std::to_string(i.x()) << " y: " << std::to_string(i.y())<<std::endl;

}
