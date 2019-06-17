#pragma once

#include <memory>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"
#include "nanoflann/nanoflann.hpp"

#include "route_types.h"

namespace path_planner {

class RouteIndex {
 public:
  RouteIndex() = default;
  RouteIndex(const std::vector<RouteSegment>& route);
  size_t get_segment_ind(double s) const;

 private:
  using SegmentPair = std::pair<double, size_t>;
  std::vector<SegmentPair> segment_ends_;
};

class KDTreeRouteSegmentIndex {
 public:
  KDTreeRouteSegmentIndex(const std::vector<double>& maps_x,
                          const std::vector<double>& maps_y,
                          const std::vector<double>& maps_s);

  // Query for route segment nearest inertial query point.
  bool closest(const InertialVector& query_pt, RouteVector& ret_pt,
               RouteSegment& ret_seg) const;
  // Query for route segment nearest route query point.
  RouteSegment closest(const RouteVector& query_pt) const;

  // Interface expected for DatasetAdapter
  inline size_t kdtree_get_point_count() const { return route_.size(); }

  inline double kdtree_get_pt(const size_t idx, int dim) const {
    Eigen::Vector2d midpoint = (route_[idx].pt0().inertial().pt() +
                                route_[idx].pt1().inertial().pt()) /
                               2.0;
    return midpoint[dim];
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX& bb) const {
    return false;
  }

 private:
  std::vector<RouteSegment> route_;
  using adapter_t = nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<double, KDTreeRouteSegmentIndex>,
      KDTreeRouteSegmentIndex, 2>;
  std::unique_ptr<adapter_t> inertial_index_;
  RouteIndex route_index_;
};

}  // path_planner
