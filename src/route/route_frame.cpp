#include "route_frame.h"

#include <cassert>
#include <limits>

namespace path_planner {

InertialCoordinate TrajectoryPoint::inertial() const {
  if (!opt_inertial_pt_) {
    assert(opt_route_pt_);
    opt_inertial_pt_ = route_frame_->to_inertial(opt_route_pt_.value());
  }
  return opt_inertial_pt_.get();
}

RouteCoordinate TrajectoryPoint::route() const {
  if (!opt_route_pt_) {
    assert(opt_inertial_pt_);
    opt_route_pt_ = route_frame_->to_route(opt_inertial_pt_.value());
  }
  return opt_route_pt_.get();
}

boost::optional<RouteCoordinate> RouteFrame::to_route(
    const InertialCoordinate& inertial_pt) {
  RouteSegment segment;
  RouteCoordinate route_pt;
  if (lane_index_.closest(inertial_pt, route_pt, segment)) {
    return route_pt;
  }
  return boost::none;
}

InertialCoordinate RouteFrame::to_inertial(const RouteCoordinate& route_pt) {
  RouteSegment closest_segment = lane_index_.closest(route_pt);
  return closest_segment.to_inertial(route_pt);
}

}  // path_planner
