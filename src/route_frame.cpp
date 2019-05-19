#include "route_frame.h"

#include <cassert>
#include <limits>

namespace path_planner {

using boost::polygon::construct_voronoi;

InertialCoordinate TrajectoryPoint::inertial() const {
  if (!opt_inertial_pt_) {
    assert(opt_route_pt_);
    InertialCoordinate inertial;
    RouteIter route_iter;
    if (opt_hint_) {
      route_frame_->to_inertial(opt_route_pt_.value(), opt_hint_.value(), inertial,
                                route_iter);
    } else {
      route_frame_->to_inertial(opt_route_pt_.value(), inertial, route_iter);
    }
    opt_inertial_pt_ = std::move(inertial);
    opt_route_iter_ = route_iter;
  }
  return opt_inertial_pt_.get();
}

RouteCoordinate TrajectoryPoint::route() const {
  if (!opt_route_pt_) {
    assert(opt_inertial_pt_);
    // opt_route_pt_ = route_frame_->to_route(opt_inertial_pt_.value());
    RouteCoordinate route_pt;
    RouteIter route_iter;
    if (opt_hint_) {
      route_frame_->to_route(opt_inertial_pt_.value(), opt_hint_.value(), route_pt,
                             route_iter);
    } else {
      route_frame_->to_route(opt_inertial_pt_.value(), route_pt, route_iter);
    }
    opt_route_pt_ = std::move(route_pt);
    opt_route_iter_ = route_iter;
  }
  return opt_route_pt_.get();
}

RouteFrame::RouteFrame(const std::vector<double>& maps_x, const std::vector<double>& maps_y,
           const std::vector<double>& maps_s) {
  // Ensure at least two points and input same length.
  assert(maps_x.size() == maps_y.size());
  assert(maps_y.size() == maps_s.size());
  assert(maps_x.size() > 1);

  // Construct RouteSegments in order.
  auto x_iter = maps_x.cbegin() + 1;
  auto y_iter = maps_y.cbegin() + 1;
  auto s_iter = maps_s.cbegin() + 1;
  double last_s = maps_s.front();
  while (x_iter != maps_x.cend()) {
    // Assert points are given in increasing-s order.
    assert(last_s < *s_iter);
    last_s = *s_iter;

    Waypoint wp1(*(x_iter - 1), *(y_iter - 1), *(s_iter - 1));
    Waypoint wp2(*x_iter, *y_iter, *s_iter);
    route_.emplace_back(std::move(wp1), std::move(wp2));
  }

  // Build Voronoi diagram of segments.
  construct_voronoi(route_.cbegin(), route_.cend(), &route_voronoi_);
  for (VoronoiIter v_it = route_voronoi_.cells().cbegin();
       v_it != route_voronoi_.cells().cend(); ++v_it) {
    std::vector<RouteSegment>::size_type seg_ind = v_it->source_index();
    cell_map_.emplace(route_.begin() + seg_ind, v_it);
  }
}

void RouteFrame::to_route(const InertialCoordinate& inertial_pt,
                          RouteCoordinate& ret, RouteIter& route_iter) {}
void RouteFrame::to_route(const InertialCoordinate& inertial_pt,
                          const RouteIter& hint, RouteCoordinate& ret,
                          RouteIter& route_iter) {}
void RouteFrame::to_inertial(const RouteCoordinate& route_pt,
                             InertialCoordinate& ret, RouteIter& route_iter) {}
void RouteFrame::to_inertial(const RouteCoordinate& route_pt,
                             const RouteIter& hint, InertialCoordinate& ret,
                             RouteIter& route_iter) {}
}  // path_planner
