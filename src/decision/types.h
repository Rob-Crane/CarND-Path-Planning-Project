#pragma once
#include "trajectory/types.h"
#include "route/route_frame.h"

namespace path_planner {

struct AdversaryObservation {
  AdversaryObservation(double x, double y, double dx, double dy)
      : x_(x), y_(y), dx_(dx), dy_(dy) {}
  double x_;
  double y_;
  double dx_;
  double dy_;
};

class TrajectoryState {
 public:
  TrajectoryState(TrajectoryPoint pt, double sv, double sa)
      : pt_(std::move(pt)), sv_(sv), sa_(sa) {}
  Sa sa() const { return sa_; }
  Sv sv() const { return sv_; }
  double s() const { return pt_.route().s(); }
  double d() const { return pt_.route().d(); }
  double x() const { return pt_.inertial().x(); }
  double y() const { return pt_.inertial().y(); }

 private:
  TrajectoryPoint pt_;
  Sv sv_;
  Sa sa_;
};
}  // path_planner
