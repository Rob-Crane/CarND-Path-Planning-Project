#pragma once

#include "Eigen-3.3/Eigen/Dense"
#include "boost/optional.hpp"
#include "boost/polygon/polygon.hpp"

namespace path_planner {

using Eigen::Matrix2d;
using Eigen::Vector2d;
using PointMatrix = Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor>;

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


class Projection {
 public:
  Projection() = default;
  Projection(const Vector2d& v) {
    norm_ = v.norm();
    proj_ << v(0) / norm_, v(1) / norm_, v(1) / norm_,
        -v(0) / norm_;
  }
  // Project columns of matrix onto vector.
  PointMatrix project(const PointMatrix& pts) const { return proj_ * pts; }
  // Invert the projection on columns of matrix..
  PointMatrix invert(const PointMatrix& pts) const { return proj_.inverse() * pts; }

 protected:
  double norm() const { return norm_; }
 private:
  double norm_;
  Matrix2d proj_;  // Projection onto v.
};

class RouteSegment : protected Projection {
 public:
  RouteSegment() = default;
  RouteSegment(const Waypoint& pt0, const Waypoint& pt1)
      : Projection(pt1.inertial().pt() - pt0.inertial().pt()),
        pt0_(pt0),
        pt1_(pt1) {}

  const Waypoint& pt0() const { return pt0_; }
  const Waypoint& pt1() const { return pt1_; }
  boost::optional<RouteCoordinate> to_route(const InertialCoordinate& p) const;
  InertialCoordinate to_inertial(const RouteCoordinate& p) const;

 private:
  Waypoint pt0_;
  Waypoint pt1_;
};
}  // path_planner
