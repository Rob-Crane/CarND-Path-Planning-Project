#include "route_smoother.h"

#include <cmath>

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
  unsigned num_spline = 2 + 2 * num_bookend;
  assert(num_spline <= num_base_);
  std::vector<Vector2d> printvs;
  for (int i = 0; i < num_base_; ++i) {
    // Grab the points before and after the route vector starting
    // at position i.
    int left_mod = (i - int(num_bookend)) % int(num_base_);
    unsigned left_bookend = left_mod < 0 ? left_mod + num_base_ : left_mod;
    unsigned right_bookend = (i + 1 + num_bookend) % num_base_;
    PointMatrix fit_pts(2, num_spline);
    if (right_bookend < left_bookend) {
      fit_pts << base_pts_.block(0, left_bookend, 2, num_base_ - left_bookend),
          base_pts_.block(0, 0, 2, right_bookend + 1);
    } else {
      fit_pts << base_pts_.block(0, left_bookend, 2, num_spline);
    }

    Vector2d base_begin = base_pts_.col(i);
    Vector2d base_end = base_pts_.col((i + 1) % num_base_);
    // Get a base vector from route point i to point i+1.
    Vector2d v =  base_end - base_begin;
    // Get a projection matrix onto base vector.
    Projection proj(v);
    // Project fit points onto vector to get in local frame.
    PointMatrix spline_pts = proj.project(fit_pts.colwise() - base_begin);
    auto begin = spline_pts.data();
    auto begin_y = begin + num_spline;
    auto end = begin_y + num_spline;
    std::vector<double> x(begin, begin_y);
    std::vector<double> y(begin_y, end);

    // Fit spline to those points.
    tk::spline s;
    s.set_points(x, y);
    double end_x = spline_pts.col(num_bookend + 1)[0];
    int num_interstitials = std::floor(end_x / ds);
    PointMatrix interstitials(2, num_interstitials);
    for (int j = 1; j <= num_interstitials; ++j) {
      Vector2d interstitial;
      double sx = ds * j;
      interstitial << sx, s(sx);
      interstitials.col(j-1) = interstitial;
    }
    // Transform interstials to global frame.
    PointMatrix global_interstitials =
        proj.invert(interstitials).colwise() + base_begin;
    // Add base points to beginning and end.
    PointMatrix route_pts(2, num_interstitials+2);
    route_pts << base_begin, global_interstitials, base_end;
    for (int j = 0; j < route_pts.cols(); ++j) {
      Vector2d v = route_pts.col(j);
      printvs.push_back(v);
    }
  }
  for (int j = 0; j < printvs.size(); ++j) {
    std::cout<<printvs[j][0] << ", " << printvs[j][1] << std::endl;
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
