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

RouteSegment::RouteSegment(const Waypoint& pt0, const Waypoint& pt1)
    : pt0_(pt0), pt1_(pt1) {
  segment_ = pt1.inertial().pt() - pt0.inertial().pt();
  norm_ = segment_.norm();
  // Projection matrix is [+s unit vector, +d unit vector].
  route_proj_ << segment_(0) / norm_, segment_(1) / norm_, segment_(1) / norm_,
      -segment_(0) / norm_;
}

boost::optional<RouteCoordinate> RouteSegment::to_route(
    const InertialCoordinate& p) const {
  Vector2d p_vec = p.pt() - pt0_.inertial().pt();
  Vector2d local = route_proj_ * p_vec;
  double s = local(0);
  if (s > 0 && s < norm_) {
    return RouteCoordinate(pt0_.route().pt() + local);
  }
  // If vector's s-projection is before or after segment
  // vector, then it has no projection on the segment.
  return boost::none;
}

InertialCoordinate RouteSegment::to_inertial(const RouteCoordinate& p) const {
  Vector2d local = p.pt() - pt0_.route().pt();
  return InertialCoordinate(pt0_.inertial().pt() +
                            route_proj_.inverse() * local);
}

}  // path_planner
