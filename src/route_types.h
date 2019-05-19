#pragma once

#include "Eigen-3.3/Eigen/Core"
#include "boost/polygon/polygon.hpp"

namespace path_planner {

using Vector2d = Eigen::Vector2d;

struct InertialCoordinate {
  InertialCoordinate() = default;
  InertialCoordinate(double x, double y);
  InertialCoordinate(const Vector2d& pt) : pt_(pt) {}
  InertialCoordinate(Vector2d&& pt) : pt_(pt){};
  operator Vector2d() const { return pt_; }
  Vector2d pt_;
};

struct RouteCoordinate {
  RouteCoordinate() = default;
  RouteCoordinate(double s, double d);
  RouteCoordinate(const Vector2d& pt) : pt_(pt) {}
  RouteCoordinate(Vector2d&& pt) : pt_(pt) {}
  operator Vector2d() const { return pt_; }
  Vector2d pt_;
};

class Point {
 public:
  virtual InertialCoordinate inertial() const = 0;
  virtual RouteCoordinate route() const = 0;
};

class Waypoint : public Point {
 public:
  Waypoint(double x, double y, double s) : inertial_(x, y), route_(s, 0){};
  InertialCoordinate inertial() const final { return inertial_; }
  RouteCoordinate route() const final { return route_; }

 private:
  InertialCoordinate inertial_;
  RouteCoordinate route_;
};

class RouteSegment {
 public:
  RouteSegment(const Waypoint& pt0, const Waypoint& pt1)
      : pt0_(pt0), pt1_(pt1) {}
  RouteSegment(Waypoint&& pt0, Waypoint&& pt1) : pt0_(pt0), pt1_(pt1) {}

  const Waypoint& pt0() const { return pt0_; }
  const Waypoint& pt1() const { return pt1_; }
  // double distance(const Point& p) const;

 private:
  Waypoint pt0_;
  Waypoint pt1_;
  // Inertial midpoint.
  // volatile std::optional<InertialCoordinate> midpoint_;
};
}  // path_planner

// Register InertialCoordinate with Boost Polygon.
namespace boost {
namespace polygon {
using path_planner::InertialCoordinate;
using path_planner::RouteSegment;
template <>
struct geometry_concept<InertialCoordinate> {
  typedef point_concept type;
};

template <>
struct point_traits<InertialCoordinate> {
  typedef double coordinate_type;

  static inline coordinate_type get(const InertialCoordinate& point,
                                    orientation_2d orient) {
    return (orient == HORIZONTAL) ? point.pt_[0] : point.pt_[1];
  }
};

template <>
struct geometry_concept<RouteSegment> {
  typedef segment_concept type;
};

template <>
struct segment_traits<RouteSegment> {
  typedef double coordinate_type;
  typedef InertialCoordinate point_type;

  static inline point_type get(const RouteSegment& segment, direction_1d dir) {
    return dir.to_int() ? segment.pt1().inertial() : segment.pt0().inertial();
  }
};

}  // boost
}  // polygon
