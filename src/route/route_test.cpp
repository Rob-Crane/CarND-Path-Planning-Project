#include <iostream>
#include "utils.h"
#include "route_frame.h"
#include "route_smoother.h"

using namespace path_planner;
int main() {
  //RouteFrame rf({0.0, 10.0, 10.0, 0.0}, {0.0, 0.0, 10.0, 10.0},
                //{0.0, 10.0, 20.0, 30.0});
  //InertialCoordinate inertial(0.5, 1.1);
  //auto opt_route = rf.to_route(inertial);
  //if (opt_route) {
    //std::cout << "s: " << std::to_string(opt_route->s())
              //<< " d: " << std::to_string(opt_route->d()) << std::endl;
  //} else {
    //std::cout << "failed to map"<<std::endl;
  //}

  //RouteCoordinate route(31, 5);
  //InertialCoordinate i = rf.to_inertial(route);
  //std::cout<<"x: " << std::to_string(i.x()) << " y: " << std::to_string(i.y())<<std::endl;

  BasicMap map = load_map();
  RouteSmoother rs(map.map_waypoints_x,
                   map.map_waypoints_y,
                   map.map_waypoints_s);
  std::vector<RouteSegment> route(rs.get_smooth_route(10, 0.10));
  //for(int i = 0; i < route.size(); ++i) {
    //std::cout<<float(i)/route.size()<< ", "<<route[i].pt1().route().s()<<std::endl;
  //}
  for(int i = 0; i < map.map_waypoints_s.size(); ++i) {
    std::cout<<float(i)/map.map_waypoints_s.size()<< ", "<<map.map_waypoints_s[i]<<std::endl;
  }
}
