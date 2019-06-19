#pragma once

#include "Eigen-3.3/Eigen/Dense"
#include "boost/optional/optional.hpp"

namespace path_planner {

using Eigen::Matrix2d;
using Eigen::Vector2d;
using PointMatrix = Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor>;

struct InertialVector {
  InertialVector() = default;
  InertialVector(double x, double y);
  InertialVector(const Vector2d& pt) : pt_(pt) {}
  InertialVector(Vector2d&& pt) : pt_(pt){};
  const Vector2d& pt() const { return pt_; }
  double x() const { return pt_[0]; }
  double y() const { return pt_[1]; }
  Vector2d pt_;
};

struct RouteVector {
  RouteVector() = default;
  RouteVector(double s, double d);
  RouteVector(const Vector2d& pt) : pt_(pt) {}
  RouteVector(Vector2d&& pt) : pt_(pt) {}
  const Vector2d& pt() const { return pt_; }
  double s() const { return pt_[0]; }
  double d() const { return pt_[1]; }
  Vector2d pt_;
};

class Point {
 public:
  virtual InertialVector inertial() const = 0;
  virtual RouteVector route() const = 0;
};

class Waypoint : public Point {
 public:
  Waypoint() = default;
  Waypoint(double x, double y, double s) : inertial_(x, y), route_(s, 0){};
  InertialVector inertial() const final { return inertial_; }
  RouteVector route() const final { return route_; }

 private:
  InertialVector inertial_;
  RouteVector route_;
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

class RouteSegment : public Projection {
 public:
  RouteSegment() = default;
  RouteSegment(const Waypoint& pt0, const Waypoint& pt1)
      : Projection(pt1.inertial().pt() - pt0.inertial().pt()),
        pt0_(pt0),
        pt1_(pt1) {}

  const Waypoint& pt0() const { return pt0_; }
  const Waypoint& pt1() const { return pt1_; }
  boost::optional<RouteVector> to_route(const InertialVector& p) const;
  InertialVector to_inertial(const RouteVector& p) const;

 private:
  Waypoint pt0_;
  Waypoint pt1_;
};
}  // path_planner
