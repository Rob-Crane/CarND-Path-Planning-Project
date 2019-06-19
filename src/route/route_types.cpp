#include "route_types.h"

#include <cmath>
#include <limits>

namespace path_planner {

InertialVector::InertialVector(double x, double y) {
  Vector2d pt;
  pt << x, y;
  pt_ = std::move(pt);
};

RouteVector::RouteVector(double s, double d) {
  Vector2d pt;
  pt << s, d;
  pt_ = std::move(pt);
};

boost::optional<RouteVector> RouteSegment::to_route(
    const InertialVector& p) const {
  Vector2d p_vec = p.pt() - pt0_.inertial().pt();
  Vector2d local = project(p_vec);
  double s = local(0);
  if (s > 0) {
    return RouteVector(pt0_.route().pt() + local);
  }
  // If vector's s-projection is before or after segment
  // vector, then it has no projection on the segment.
  return boost::none;
}

InertialVector RouteSegment::to_inertial(const RouteVector& p) const {
  Vector2d local = p.pt() - pt0_.route().pt();
  return InertialVector(pt0_.inertial().pt() +
                            invert(local));
}

}  // path_planner
