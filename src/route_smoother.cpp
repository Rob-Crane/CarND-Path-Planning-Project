#include "route_smoother.h"

#include "spline.h"

namespace path_planner {

RouteSmoother::RouteSmoother(const std::vector<double>& maps_x,
                             const std::vector<double>& maps_y,
                             const std::vector<double>& maps_s)
    : num_base_(maps_x.size()), base_pts_(2, num_base_), base_s_(maps_s) {
  assert(maps_x.size() == maps_y.size());
  assert(maps_y.size() == maps_s.size());

  for (size_t i = 0; i < num_base_; ++i) {
    Vector2d pt;
    pt << maps_x[i], maps_y[i];
    base_pts_.col(i) = pt;
  }
}

std::vector<RouteSegment> RouteSmoother::get_smooth_route(unsigned num_bookend,
                                                          double ds) {
  std::vector<RouteSegment> route;
  for (unsigned i = 0; i < num_base_; ++i) {
    // Get a base vector from route point i to point i+1.
    Vector2d v = base_pts_.col(i) - base_pts_.col((i - 1) % num_base_);
    // Get a projection onto base vector.
    Projection proj(v);
    // Take the points that make up the vector and bookend points to left and
    // right.
    // Include spline_pts_[left, right)
    unsigned left_bookend = (i - num_bookend) % num_base_;
    unsigned right_bookend = (i + 1 + num_bookend) % num_base_;
    unsigned num_spline = 2 + 2 * num_bookend;
    // TODO figure out the wrapping block
    // Get them in local cooridinate frame  of base vector.
    //PointMatrix spline_pts = proj.project(
        //base_pts_.block(0, left_bookend, 2, num_spline).colwise() - base_pts_.col(i));
    PointMatrix spline_pts = base_pts_.block(0, 0, 2, 0);
    std::cout<<spline_pts.size()<<std::endl;;

    // Fit spline to those points.
    //tk::spline s;
    //auto begin = spline_pts.data();
    //auto begin_y = begin + num_spline;
    //auto end = begin_y + num_spline;
    //std::vector<double> x(begin, begin_y);
    //std::vector<double> y(begin_y, end);
    //s.set_points(x, y);
  }
  return route;
}

std::vector<RouteSegment> generate_base_route(
    const std::vector<double>& maps_x, const std::vector<double>& maps_y,
    const std::vector<double>& maps_s) {
  std::vector<RouteSegment> route;

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
    route.emplace_back(std::move(wp1), std::move(wp2));
    x_iter += 1;
    y_iter += 1;
    s_iter += 1;
  }
  // Last segment connnects to first waypoint.
  Waypoint penultimate(maps_x.back(), maps_y.back(), maps_s.back());

  Eigen::Vector2d diff =
      penultimate.inertial().pt() - route.front().pt0().inertial().pt();
  double last_leg_mag = std::sqrt(diff.transpose() * diff);
  Waypoint last(maps_x.front(), maps_y.front(),
                penultimate.route().s() + last_leg_mag);
  route.emplace_back(std::move(penultimate), std::move(last));
  return route;
}
}  // path_planner
