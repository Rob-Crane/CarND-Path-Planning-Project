#include "route_frame.h"
#include "utils.h"

#include <cassert>
#include <limits>

using Eigen::Vector2d;

namespace path_planner {

InertialVector TrajectoryPoint::inertial() const {
  if (!opt_inertial_pt_) {
    assert(opt_route_pt_);
    opt_inertial_pt_ = route_frame_->to_inertial(opt_route_pt_.value());
  }
  return opt_inertial_pt_.get();
}

RouteVector TrajectoryPoint::route() const {
  if (!opt_route_pt_) {
    assert(opt_inertial_pt_);
    auto result = route_frame_->to_route(opt_inertial_pt_.value());

    assert(result);
    opt_route_pt_ = result->vector_;
    opt_route_segment_ = result->segment_;
    assert(opt_route_segment_);
  }
  return opt_route_pt_.get();
}

RouteVector TrajectoryVelocity::routeV() const {
  if (!opt_route_v_) {
    if (!opt_inertial_v_)
        exitWithStackTrace();
    assert(opt_inertial_v_);
    route();  // Populate route segment.
    assert(opt_route_segment_);
    opt_route_v_ =
        RouteVector(opt_route_segment_->project(opt_inertial_v_.get().pt()));
  }
  return opt_route_v_.get();
}

boost::optional<RouteFrame::RouteResult> RouteFrame::to_route(
    const InertialVector& inertial_pt) {
  RouteResult result;
  if (lane_index_.closest(inertial_pt, result.vector_, result.segment_)) {
    return result;
  }
  return boost::none;
}

InertialVector RouteFrame::to_inertial(const RouteVector& route_pt) {
  RouteSegment closest_segment = lane_index_.closest(route_pt);
  return closest_segment.to_inertial(route_pt);
}

}  // path_planner
