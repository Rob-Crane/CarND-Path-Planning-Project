#include "trajectory/trajectory.h"

namespace path_planner {

Dx ConstantSpeedLateralTrajectory::at(time_point t) const {
  assert(t >= begin_t_);
  // Special case (stay in lane)
  if (v_ == 0.0) {
    return begin_d_;
  }
  seconds dur = t - begin_t_;
  Dx x = begin_d + dur * v;
  if (v_ < 0.0) {
    return std::max(x, prev_lane_midpoint(x));
  } else {
    return std::min(x, next_lane_midpoint(x));
  }
}

constexpr double kRouteLength = 6945.554;
Sx ConstantSpeedLongitudinalTrajectory::at(time_point t, bool rollover) const {
  assert(t >= begin_t_);
  seconds dur = t - begin_t_;
  if (rollover) {
    return std::fmod(begin_s_ + dur * begin_v(), kRouteLength);
  } else {
    return begin_s_ + dur * begin_v();
  }
}

constexpr double kAvgDecel = -5.0;
constexpr double kAvgAccel = 4.0;
constexpr double kMaxVel = 30.0;
constexpr double kMaxVel = 30.0;
constexpr double kMaintainDistance = 10.0;
constexpr double kTrapDistance = 15.0;
static_assert(kTrapDistance > kMaintainDistance);

Sx SmoothLongitudinalTrajectory::at(time_point t) {
  assert(t > begin_t_);
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

Sx SmoothLongitudinalTrajectory::unblocked(time_point t) {
  assert(begin_v_ < kMaxVel);
  double ta = (kMaxVel - begin_v_) / kAvgAccel;
  double delta_t = seconds(t - begin_t_).count();
  if (ta < delta_t) {
    return begin_s_ + begin_v_ * delta_t + 0.5 *
  }
}

}  // path_planner
