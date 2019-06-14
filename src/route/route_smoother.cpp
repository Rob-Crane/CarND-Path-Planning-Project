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
  // Number of points used to fit each spline.
  unsigned num_spline = 2 + 2 * num_bookend;
  assert(num_spline <= num_base_);
  
  double total_s = base_s_[0]; // running total of S value.
  // Iterate along each "base segment" of the course route.
  for (int i = 0; i < num_base_; ++i) {
    // Get index of left and right-most spline fit points.
    int left_mod = (i - int(num_bookend)) % int(num_base_);
    unsigned left_bookend = left_mod < 0 ? left_mod + num_base_ : left_mod;
    unsigned right_bookend = (i + 1 + num_bookend) % num_base_;
    // Fill PointMatrix with fitting points.
    PointMatrix fit_pts(2, num_spline);
    if (right_bookend < left_bookend) {
      fit_pts << base_pts_.block(0, left_bookend, 2, num_base_ - left_bookend),
          base_pts_.block(0, 0, 2, right_bookend + 1);
    } else {
      fit_pts << base_pts_.block(0, left_bookend, 2, num_spline);
    }

    // Transform fit points to a coordinate frame around the base segment.
    Vector2d base_begin = base_pts_.col(i);
    Vector2d base_end = base_pts_.col((i + 1) % num_base_);
    Vector2d v = base_end - base_begin;
    // Get a projection matrix onto base vector.
    Projection proj(v);
    PointMatrix spline_pts = proj.project(fit_pts.colwise() - base_begin);

    // Fit spline to those points.
    auto begin = spline_pts.data();
    auto begin_y = begin + num_spline;
    auto end = begin_y + num_spline;
    std::vector<double> x(begin, begin_y);
    std::vector<double> y(begin_y, end);
    tk::spline s;
    s.set_points(x, y);

    // Use fitted spline to interpolate points along the
    // base segment.
    double end_x = spline_pts.col(num_bookend + 1)[0];
    int num_interstitials = std::floor(end_x / ds);
    PointMatrix interstitials(2, num_interstitials);
    for (int j = 1; j <= num_interstitials; ++j) {
      Vector2d interstitial;
      double sx = ds * j;
      interstitial << sx, s(sx);
      interstitials.col(j - 1) = interstitial;
    }
    // Transform interstials to global frame.
    PointMatrix global_interstitials =
        proj.invert(interstitials).colwise() + base_begin;
    // Add base points to beginning and end.
    PointMatrix route_pts(2, num_interstitials + 2);
    route_pts << base_begin, global_interstitials, base_end;
    // Generate Route
    for (int j = 0; j < route_pts.cols() - 1; ++j) {
      Vector2d p0 = route_pts.col(j);
      Vector2d p1 = route_pts.col(j + 1);
      double ds = (p1 - p0).norm();
      Waypoint w0(p0[0], p0[1], total_s);
      total_s += ds;
      Waypoint w1(p1[0], p1[1], total_s);
      route.emplace_back(std::move(w0), std::move(w1));
    }
  }
  return route;
}

std::vector<RouteSegment> generate_route(const std::vector<double>& maps_x,
                                         const std::vector<double>& maps_y,
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
  const Waypoint& last = route.back().pt1();
  const Waypoint& front = route.front().pt0();
  double ds = (last.inertial().pt() - front.inertial().pt()).norm();
  Waypoint w0(last.inertial().x(), last.inertial().y(), last.route().s());
  Waypoint w1(front.inertial().x(), front.inertial().y(),
              last.route().s() + ds);
  route.emplace_back(std::move(w0), std::move(w1));
  return route;
}

}  // path_planner
