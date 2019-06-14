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

constexpr double kLaneWidth = 2.0;
constexpr double kLaneChangeTime = 3.0;
SmoothLateralTrajectory::SmoothLateralTrajectory(Dx begin_d, time_point begin_t, LaneChangeDirection dir) {
  KinematicPoint curr(begin_d(), 0.0, 0.0, begin_t());
  Dx end_d;
  if (dir == LaneChangedirection::left) {
    end_d = prev_lane_midpoint(begin_d());
  } else {
    end_d = next_lane_midpoint(begin_d());
  }
  time_point end_t = begin_t() + seconds(kLaneChangeTime);
  KinematicPoint end(end_d, 0.0, 0.0, end_t);
  traj_ = JerkMinimizingTrajectory(curr, end);
}

Dx SmoothLateralTrajectory::at(time_point t) const {
  assert(t > begin_t());
  KinematicPoint p = traj_(t);
  return p.x_;
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
        steady_state_follow_estimate(curr, blocking, kAvgAccel, kAvgDecel, kVClose,
                           kMaintainDistance, kNominalCloseTime);
    traj_ = JerkMinimizingTrajectory(curr, intercept);
  }

  Sx FollowCarTrajectory::at(time_point t) const {
    assert(t > begin_t());
    KinematicPoint p = traj_(t);
    return p.x_;
  }

constexpr double dVMax = 30.0;
UnblockedLongitudinalTrajectory:UnblockedLongitudinalTrajectory(
Sx begin_s, Sv begin_v, Sa begin_a, time_point begin_t) {
  KinematicPoint curr(begin_s(), begin_v(), begin_a(), begin_t());
  KinematicPoint steady = steady_state_max_speed_estimate(curr, kAvgAccel, kVMax);
  traj_ = JerkMinimizingTrajectory(curr, steady);
}

Sx UnblockedLongitudinalTrajectory::at(time_point t) {
assert(t > begin_t());
KinematicPoint p = traj_(t);
return p.x_;
}


}  // path_planner
