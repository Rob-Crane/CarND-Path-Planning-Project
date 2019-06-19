#include "route_index.h"

#include <cassert>
#include <cmath>
#include <unordered_set>

#include "route_smoother.h"

namespace path_planner {

RouteIndex::RouteIndex(const std::vector<RouteSegment>& route) {
  for (size_t i = 0; i < route.size(); ++i) {
    const RouteSegment& segment = route[i];
    double end_s = segment.pt1().route().s();
    segment_ends_.push_back(std::make_pair(end_s, i));
  }
}

size_t RouteIndex::get_segment_ind(double s) const {
  double route_end = segment_ends_.back().first;
  auto comp = [](const SegmentPair& p1, const SegmentPair& p2) {
    return p1.first < p2.first;
  };
  SegmentPair query(std::make_pair(std::fmod(s, route_end), -1));
  auto it = std::lower_bound(segment_ends_.cbegin(), segment_ends_.cend(),
                             query, comp);
  assert(it != segment_ends_.cend());
  return it->second;
}

KDTreeRouteSegmentIndex::KDTreeRouteSegmentIndex(
    const std::vector<double>& maps_x, const std::vector<double>& maps_y,
    const std::vector<double>& maps_s) {
  RouteSmoother smoother(maps_x, maps_y, maps_s);
  const unsigned num_bookend = 10;
  const double ds = 0.1;
  route_ = smoother.get_smooth_route(num_bookend, ds);

  // Construct RouteIndex.
  route_index_ = RouteIndex(route_);

  // Construct KDTree of segment midpoints.
  constexpr int leafs_max_size = 10;
  inertial_index_ = std::unique_ptr<adapter_t>(new adapter_t(
      2, *this, nanoflann::KDTreeSingleIndexAdaptorParams(leafs_max_size)));
  inertial_index_->buildIndex();
}

bool KDTreeRouteSegmentIndex::closest(const InertialVector& query_pt,
                                      RouteVector& ret_pt,
                                      RouteSegment& ret_seg) const {
  // Query for the nearest_n segment endpoints.
  constexpr size_t nearest_n = 4;
  std::vector<size_t> ret_indexes(nearest_n);
  std::vector<double> out_dists_sqr(nearest_n);
  nanoflann::KNNResultSet<double> result_set(nearest_n);
  result_set.init(&ret_indexes[0], &out_dists_sqr[0]);
  assert(inertial_index_->findNeighbors(result_set,
                                        &Eigen::Vector2d(query_pt.pt())[0],
                                        nanoflann::SearchParams(10)));

  // Find segment with minimal d value.
  constexpr double kMaxDouble(std::numeric_limits<double>::max());
  double min_d = kMaxDouble;
  auto best_segment = ret_indexes.cend();
  RouteVector best_pt;
  for (auto it = ret_indexes.cbegin(); it != ret_indexes.cend(); ++it) {
    const RouteSegment& segment(route_[*it]);
    auto opt_route_pt(segment.to_route(query_pt));
    if (opt_route_pt && std::abs(opt_route_pt->d()) < min_d) {
      min_d = opt_route_pt->d();
      best_segment = it;
      best_pt = opt_route_pt.get();
    }
  }
  if (best_segment == ret_indexes.cend()) {
    return false;
  } else {
    ret_pt = std::move(best_pt);
    ret_seg = route_[*best_segment];
    return true;
  }
}

RouteSegment KDTreeRouteSegmentIndex::closest(
    const RouteVector& query_pt) const {
  size_t ind = route_index_.get_segment_ind(query_pt.s());
  assert(ind < route_.size());
  return route_[ind];
}
}  // path_planner
