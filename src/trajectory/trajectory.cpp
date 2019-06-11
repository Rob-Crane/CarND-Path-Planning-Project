#include "trajectory.h"
#include "utils.h"

#include <cassert>
#include <cmath>

namespace path_planner {

Dx ConstantSpeedLateralTrajectory::at(time_point t) const {
  assert(t >= begin_t());
  // Special case (stay in lane)
  if (begin_v() == 0.0) {
    return begin_d();
  }
  seconds dur = t - begin_t();
  Dx x = begin_d() + dur.count() * begin_v();
  if (begin_v() < 0.0) {
    return std::max(x, prev_lane_midpoint(x));
  } else {
    return std::min(x, next_lane_midpoint(x));
  }
}

constexpr double kRouteLength = 6945.554;
Sx ConstantSpeedLongitudinalTrajectory::at(time_point t) const {
  assert(t >= begin_t());
  seconds dur = t - begin_t();
  return begin_s() + dur.count() * begin_v();
}

constexpr double kAvgDecel = -5.0;
constexpr double kAvgAccel = 4.0;
constexpr double kMaxVel = 30.0;
constexpr double kMaintainDistance = 10.0;
constexpr double kTrapDistance = 15.0;
static_assert(kTrapDistance > kMaintainDistance);

Sx SmoothLongitudinalTrajectory::at(time_point t) const {
  assert(t > begin_t());
  if (blocking_ && blocking_->begin_v() < kMaxVel &&
      std::abs(begin_s() - blocking_->begin_s()) < kTrapDistance) {
    return blocked(t);
  } 
  return unblocked(t);
}

  // double slowdown_time = blocking->begin_v() - begin_v() / kAvgDecel;
  // assert(slowdown_time > 0);
  // double slowdown_distance = begin_v() * slowdown_time +
  // 0.5 * kAvgDecel * slowdown_time * slowdown_time;

Sx SmoothLongitudinalTrajectory::unblocked(time_point t) const {
  assert(begin_v() <= kMaxVel);
  double ta = (kMaxVel - begin_v()) / kAvgAccel;
  assert(ta => 0.0);
  double delta_t = seconds(t - begin_t()).count();
  if (ta < delta_t) {
    return begin_s() + begin_v() * delta_t + 0.5 *
  }
}

}  // path_planner
