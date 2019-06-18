#pragma once

#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"
#include "boost/optional.hpp"

#include "route_index.h"
#include "route_types.h"

namespace path_planner {

class RouteFrame;
using RouteFramePtr = std::shared_ptr<RouteFrame>;
using RouteIter = std::vector<RouteSegment>::const_iterator;

class TrajectoryPoint {
 public:
  TrajectoryPoint() = delete;
  TrajectoryPoint(const InertialVector& inertial_pt, RouteFramePtr route_frame)
      : opt_inertial_pt_(inertial_pt), route_frame_(std::move(route_frame)) {}
  TrajectoryPoint(const RouteVector& route_pt, RouteFramePtr route_frame)
      : opt_route_pt_(route_pt), route_frame_(std::move(route_frame)) {}

  InertialVector inertial() const;
  RouteVector route() const;

 protected:
  RouteFramePtr route_frame_;
  mutable boost::optional<InertialVector> opt_inertial_pt_;
  mutable boost::optional<RouteVector> opt_route_pt_;
  mutable boost::optional<RouteSegment> opt_route_segment_;
};

class TrajectoryVelocity : public TrajectoryPoint {
 public:
  TrajectoryVelocity(const InertialVector& inertial_pt,
                     const InertialVector inertial_v, RouteFramePtr route_frame)
      : TrajectoryPoint(inertial_pt, std::move(route_frame)),
        opt_inertial_v_(inertial_v) {}

  RouteVector routeV() const;

 private:
  mutable boost::optional<InertialVector> opt_inertial_v_;
  mutable boost::optional<RouteVector> opt_route_v_;
};

class RouteFrame {
 public:
  RouteFrame(const std::vector<double>& maps_x,
             const std::vector<double>& maps_y,
             const std::vector<double>& maps_s)
      : lane_index_(maps_x, maps_y, maps_s) {}
  //static RouteFramePtr make(const std::vector<double>& maps_x,
                            //const std::vector<double>& maps_y,
                            //const std::vector<double>& maps_s) {
    //return std::make_shared<RouteFrame>(maps_x, maps_y, maps_s);
  //}

  struct RouteResult {
    RouteVector vector_;    // coordinate
    RouteSegment segment_;  // route (road) primitive
  };
  boost::optional<RouteResult> to_route(const InertialVector& inertial_pt);
  InertialVector to_inertial(const RouteVector& route_pt);

 private:
  KDTreeRouteSegmentIndex lane_index_;
};

}  // path_planner
