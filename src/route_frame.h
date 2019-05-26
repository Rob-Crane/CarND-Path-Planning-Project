#pragma once

#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"
#include "boost/optional.hpp"

#include "route_types.h"
#include "route_index.h"

namespace path_planner {

using Eigen::Vector2d;

class RouteFrame;
using RouteFramePtr = std::shared_ptr<RouteFrame>;
using RouteIter = std::vector<RouteSegment>::const_iterator;

class TrajectoryPoint : public Point {
 public:
  TrajectoryPoint() = delete;
  TrajectoryPoint(const InertialCoordinate& inertial_pt,
                  RouteFramePtr route_frame)
      : opt_inertial_pt_(inertial_pt), route_frame_(std::move(route_frame)) {}
  TrajectoryPoint(const RouteCoordinate& route_pt, RouteFramePtr route_frame)
      : opt_route_pt_(route_pt), route_frame_(std::move(route_frame)) {}

  InertialCoordinate inertial() const final;
  RouteCoordinate route() const final;

 private:
  RouteFramePtr route_frame_;
  mutable boost::optional<InertialCoordinate> opt_inertial_pt_;
  mutable boost::optional<RouteCoordinate> opt_route_pt_;
};

class RouteFrame {
 public:
  RouteFrame(const std::vector<double>& maps_x,
             const std::vector<double>& maps_y,
             const std::vector<double>& maps_s)
      : lane_index_(maps_x, maps_y, maps_s) {}
  static RouteFramePtr make(const std::vector<double>& maps_x,
                            const std::vector<double>& maps_y,
                            const std::vector<double>& maps_s) {
    return std::make_shared<RouteFrame>(maps_x, maps_y, maps_s);
  }

  boost::optional<RouteCoordinate> to_route(
      const InertialCoordinate& inertial_pt);
  InertialCoordinate to_inertial(const RouteCoordinate& route_pt);

 private:
  KDTreeRouteSegmentIndex lane_index_;
};

}  // path_planner
