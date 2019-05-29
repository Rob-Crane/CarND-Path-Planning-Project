#include "route_types.h"

#include <cmath>
#include <limits>

namespace path_planner {

InertialCoordinate::InertialCoordinate(double x, double y) {
  Vector2d pt;
  pt << x, y;
  pt_ = std::move(pt);
};

RouteCoordinate::RouteCoordinate(double s, double d) {
  Vector2d pt;
  pt << s, d;
  pt_ = std::move(pt);
};

boost::optional<RouteCoordinate> RouteSegment::to_route(
    const InertialCoordinate& p) const {
  Vector2d p_vec = p.pt() - pt0_.inertial().pt();
  Vector2d local = project(p_vec);
  double s = local(0);
  if (s > 0 && s < norm()) {
    return RouteCoordinate(pt0_.route().pt() + local);
  }
  // If vector's s-projection is before or after segment
  // vector, then it has no projection on the segment.
  return boost::none;
}

InertialCoordinate RouteSegment::to_inertial(const RouteCoordinate& p) const {
  Vector2d local = p.pt() - pt0_.route().pt();
  return InertialCoordinate(pt0_.inertial().pt() +
                            invert(local));
}

}  // path_planner
