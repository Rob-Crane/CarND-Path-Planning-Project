#pragma once

#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "boost/optional.hpp"
#include "boost/polygon/voronoi.hpp"

#include "route_types.h"

namespace path_planner {

using boost::polygon::voronoi_diagram;
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
  TrajectoryPoint(const RouteCoordinate& route_pt, RouteFramePtr route_frame,
                  RouteIter hint)
      : opt_route_pt_(route_pt),
        route_frame_(std::move(route_frame)),
        opt_hint_(std::move(hint)) {}

  InertialCoordinate inertial() const final;
  RouteCoordinate route() const final;
  const RouteIter& route_iter() const;

 private:
  RouteFramePtr route_frame_;
  mutable boost::optional<InertialCoordinate> opt_inertial_pt_;
  mutable boost::optional<RouteCoordinate> opt_route_pt_;
  // If available, starting route segment for search.
  mutable boost::optional<const RouteIter> opt_hint_;
  // Route segment used to transform coordinate frames.
  mutable boost::optional<RouteIter> opt_route_iter_;
};

class RouteFrame {
 public:
  RouteFrame(const std::vector<double>& maps_x,
             const std::vector<double>& maps_y,
             const std::vector<double>& maps_s);
  static RouteFramePtr make(const std::vector<double>& maps_x,
                            const std::vector<double>& maps_y,
                            const std::vector<double>& maps_s) {
    return std::make_shared<RouteFrame>(maps_x, maps_y, maps_s);
  }

  void to_route(const InertialCoordinate& inertial_pt, RouteCoordinate& ret,
                RouteIter& route_iter);
  void to_route(const InertialCoordinate& inertial_pt, const RouteIter& hint,
                RouteCoordinate& ret, RouteIter& route_iter);
  void to_inertial(const RouteCoordinate& route_pt, InertialCoordinate& ret,
                   RouteIter& route_iter);
  void to_inertial(const RouteCoordinate& route_pt, const RouteIter& hint,
                   InertialCoordinate& ret, RouteIter& route_iter);

 private:
  std::vector<RouteSegment> route_;
  voronoi_diagram<double> route_voronoi_;
  using VoronoiIter = voronoi_diagram<double>::const_cell_iterator;

  // Hash functor for vector iterator.
  struct HashRouteIter {
    size_t operator()(const RouteIter& it) const {
      std::hash<const RouteSegment*>()(&(*it));
    }
  };
  // Map RouteSegment cell iterator to Voronoi iterator.
  std::unordered_map<RouteIter, VoronoiIter, HashRouteIter> cell_map_;
};

}  // path_planner
