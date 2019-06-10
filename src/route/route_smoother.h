#pragma once

#include <vector>

#include "route_types.h"

namespace path_planner {

class RouteSmoother {
public:
  RouteSmoother(const std::vector<double>& maps_x,
                const std::vector<double>& maps_y,
                const std::vector<double>& maps_s);
  std::vector<RouteSegment> get_smooth_route(unsigned num_bookend, double ds);

 private:
  const unsigned num_base_;
  PointMatrix base_pts_;
  const std::vector<double> base_s_;
};

std::vector<RouteSegment> generate_route(const std::vector<double>& maps_x,
                                         const std::vector<double>& maps_y,
                                         const std::vector<double>& maps_s);

}  // path_planner
