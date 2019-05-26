#pragma once

#include "Eigen-3.3/Eigen/Dense"
#include "boost/optional.hpp"
#include "boost/polygon/polygon.hpp"

namespace path_planner {

using Matrix2d = Eigen::Matrix2d;
using Vector2d = Eigen::Vector2d;

struct InertialCoordinate {
  InertialCoordinate() = default;
  InertialCoordinate(double x, double y);
  InertialCoordinate(const Vector2d& pt) : pt_(pt) {}
  InertialCoordinate(Vector2d&& pt) : pt_(pt){};
  const Vector2d& pt() const { return pt_; }
  double x() const { return pt_[0]; }
  double y() const { return pt_[1]; }
  Vector2d pt_;
};

struct RouteCoordinate {
  RouteCoordinate() = default;
  RouteCoordinate(double s, double d);
  RouteCoordinate(const Vector2d& pt) : pt_(pt) {}
  RouteCoordinate(Vector2d&& pt) : pt_(pt) {}
  const Vector2d& pt() const { return pt_; }
  double s() const { return pt_[0]; }
  double d() const { return pt_[1]; }
  Vector2d pt_;
};

class Point {
 public:
  virtual InertialCoordinate inertial() const = 0;
  virtual RouteCoordinate route() const = 0;
};

class Waypoint : public Point {
 public:
  Waypoint() = default;
  Waypoint(double x, double y, double s) : inertial_(x, y), route_(s, 0){};
  InertialCoordinate inertial() const final { return inertial_; }
  RouteCoordinate route() const final { return route_; }

 private:
  InertialCoordinate inertial_;
  RouteCoordinate route_;
};

class RouteSegment {
 public:
  RouteSegment() = default;
  RouteSegment(const Waypoint& pt0, const Waypoint& pt1);

  const Waypoint& pt0() const { return pt0_; }
  const Waypoint& pt1() const { return pt1_; }
  boost::optional<RouteCoordinate> to_route(const InertialCoordinate& p) const;
  InertialCoordinate to_inertial(const RouteCoordinate& p) const;

 private:
  Waypoint pt0_;
  Waypoint pt1_;

  // Cached values to find d-values.
  Vector2d segment_; // Euclidean vector from pt0 to pt1.
  Matrix2d route_proj_; // Projection onto segment_.
  double norm_;
};
}  // path_planner
