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
constexpr double kVClose = 30.0;
constexpr double kMaintainDistance = 10.0;
constexpr double kNominalCloseTime = 5.0;
// constexpr double kTrapDistance = 15.0;
// static_assert(kTrapDistance > kMaintainDistance);

FollowCarTrajectory::FollowCarTrajectory(
    Sx begin_s, Sv begin_v, Sa begin_a, time_point begin_t,
    const ConstantSpeedLongitudinalTrajectory& blocking_traj);
  : LongitudinalTrajectory(begin_s, begin_v, begin_a, begin_t){
    assert(blocking_traj.begin_t() ==
           begin_t());  // Verify trajectories generated from current time base.
    KinematicPoint curr(begin_s(), begin_v(), begin_a(), begin_t());
    KinematicPoint blocking(blocking_traj.begin_s(), blocking_traj.begin_v(),
                            blocking_traj.begin_a(), blocking_traj.begin_t());
    KinematicPoint intercept =
        get_time_to_target(curr, blocking, kAvgAccel, kAvgDecel, kVClose,
                           kMaintainDistance, kNominalCloseTime);
    traj_ = JerkMinimizingTrajectory(curr, intercept);
  }

  Sx FollowCarTrajectory::at(time_point t) const {
    assert(t > begin_t());
    KinematicPoint p = traj_(t);
    return p.x_;
  }

  // double slowdown_time = blocking->begin_v() - begin_v() / kAvgDecel;
  // assert(slowdown_time > 0);
  // double slowdown_distance = begin_v() * slowdown_time +
  // 0.5 * kAvgDecel * slowdown_time * slowdown_time;

  }  // path_planner
