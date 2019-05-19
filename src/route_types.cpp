#include "route_types.h"

#include <cmath>

namespace path_planner {

InertialCoordinate::InertialCoordinate(double x, double y) {
  Vector2d pt;
  pt << x, y;
  pt_ = std::move(pt);
};

RouteCoordinate::RouteCoordinate(double s, double d) {
  Vector2d pt;
  pt << s, d;
  pt_ = std::move(pt);
};

//double RouteSegment::distance(const Point& p) {
  //if (!midpoint_) {
    //midpoint_ = (pt0.inertial() + pt1.inertial()) / 2.0;
  //}
  //Vector2d diff = p.inertial() - midpoint_;
  //return std::sqrt(diff.transpose() * diff);
//}

}  // path_planner
