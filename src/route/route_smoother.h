#pragma once

#include <vector>

#include "route_types.h"

namespace path_planner {

// Generate a smooth, continuous route from coarse route waypoints. Using cubic
// spline interpoloation, create a smooth path between each waypoint using
// waypoints before and after to create smooth path.
class RouteSmoother {
 public:
  RouteSmoother(const std::vector<double>& maps_x,
                const std::vector<double>& maps_y,
                const std::vector<double>& maps_s);
  // Generate a smooth route using num_bookend points before and after each
  // coarse segment.  ds describes the step distance along the coarse segment.
  // normalize_s fixes the S values so the segments at the beginning of each
  // base segment have the same S value as those passed in maps_s.
  std::vector<RouteSegment> get_smooth_route(unsigned num_bookend, double ds);

 private:
  const unsigned num_base_;  // n (number of coarse points)
  PointMatrix base_pts_;     // 2xn matrix of course points.
  const std::vector<double> base_s_;
};

// For testing, generate a route from coarse map segments.
std::vector<RouteSegment> generate_route(const std::vector<double>& maps_x,
                                         const std::vector<double>& maps_y,
                                         const std::vector<double>& maps_s);

}  // path_planner
