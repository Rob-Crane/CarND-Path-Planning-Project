#include "trajectory.h"
#include "utils.h"

#include <cassert>
#include <cmath>

#include "config.h"

#include <iostream>

namespace path_planner {

// Config section
constexpr double kLaneChangeTime = 3.2;  // Time to execute a lane change.

JerkMinimizingTrajectory::JerkMinimizingTrajectory(const KinematicPoint& p0,
                                                   const KinematicPoint& p1)
    : p0_(p0) {
  double dt = seconds(p1.t_ - p0.t_).count();
  Eigen::Matrix3d tmat(get_tmat(dt));
  Eigen::Vector3d s_terms;
  s_terms << p1.x_ - (p0.x_ + p0.v_ * dt + 0.5 * p0.a_ * dt * dt),
      p1.v_ - (p0.v_ + p0.a_ * dt), p1.a_ - p0.a_;
  coef_ = tmat.colPivHouseholderQr().solve(s_terms);
}

KinematicPoint JerkMinimizingTrajectory::operator()(time_point t) const {
  double dt = seconds(t - p0_.t_).count();
  Eigen::Vector3d smoothing = get_tmat(dt) * coef_;
  KinematicPoint ret;
  ret.x_ = p0_.x_ + p0_.v_ * dt + p0_.a_ * dt * dt + smoothing[0];
  ret.v_ = p0_.v_ + p0_.a_ * dt + smoothing[1];
  ret.a_ = p0_.a_ + smoothing[2];
  ret.t_ = t;
  return ret;
}

KinematicPoint ConstantSpeedLateralTrajectory::at(time_point t) const {
  assert(t >= begin_t());
  // Special case (stay in lane)
  if (begin_v() == 0.0) {
    return KinematicPoint(begin_d(), begin_v(), 0.0, t);
  }
  seconds dur = t - begin_t();
  Dx x = begin_d() + dur.count() * begin_v();
  Dx dx;
  if (begin_v() < 0.0) {
    dx = std::max(x, prev_lane_midpoint(x));
  } else {
    dx = std::min(x, next_lane_midpoint(x));
  }
  return KinematicPoint(dx, begin_v(), 0.0, t);
}

KinematicPoint ConstantSpeedLongitudinalTrajectory::at(time_point t) const {
  assert(t >= begin_t());
  seconds dur = t - begin_t();
  return KinematicPoint(begin_s() + dur.count() * begin_v(), begin_v(), 0.0, t);
}

SmoothLateralTrajectory::SmoothLateralTrajectory(Dx beg_d, time_point beg_t,
                                                 LaneChangeDirection dir)
    : LateralTrajectory(beg_d, beg_t) {
  KinematicPoint curr(begin_d(), 0.0, 0.0, begin_t());
  Dx end_d;
  if (dir == LaneChangeDirection::kLeft) {
    end_d = prev_lane_midpoint(begin_d());
  } else {
    end_d = next_lane_midpoint(begin_d());
  }
  time_point end_t = begin_t() + seconds(kLaneChangeTime);
  KinematicPoint end(end_d, 0.0, 0.0, end_t);
  traj_ = JerkMinimizingTrajectory(curr, end);
}

KinematicPoint SmoothLateralTrajectory::at(time_point t) const {
  assert(t >= begin_t());
  KinematicPoint p = traj_(t);
  return p;
}

FollowCarTrajectory::FollowCarTrajectory(
    Sx beg_s, Sv beg_v, Sa beg_a, time_point beg_t,
    const ConstantSpeedLongitudinalTrajectory& blocking_traj)
    : LongitudinalTrajectory(beg_s, beg_v, beg_a, beg_t) {
  KinematicPoint curr(begin_s(), begin_v(), begin_a(), begin_t());
  KinematicPoint blocking(blocking_traj.begin_s(), blocking_traj.begin_v(),
                          blocking_traj.begin_a(), blocking_traj.begin_t());
  KinematicPoint intercept = steady_state_follow_estimate(
      curr, blocking);
  traj_ = JerkMinimizingTrajectory(curr, intercept);
  steady_ =
      ConstantSpeedLongitudinalTrajectory(intercept.x_, intercept.v_, intercept.t_);
}

KinematicPoint FollowCarTrajectory::at(time_point t) const {
  assert(t >= begin_t());
  if (t >= steady_.begin_t()) {
    return steady_.at(t);
  }
  KinematicPoint p = traj_(t);
  return p;
}

UnblockedLongitudinalTrajectory::UnblockedLongitudinalTrajectory(
    Sx beg_s, Sv beg_v, Sa beg_a, time_point beg_t)
    : LongitudinalTrajectory(beg_s, beg_v, beg_a, beg_t) {
  KinematicPoint curr(begin_s(), begin_v(), begin_a(), begin_t());
  KinematicPoint steady =
      steady_state_max_speed_estimate(curr);
  traj_ = JerkMinimizingTrajectory(curr, steady);
  steady_ =
      ConstantSpeedLongitudinalTrajectory(steady.x_, steady.v_, steady.t_);
}

KinematicPoint UnblockedLongitudinalTrajectory::at(time_point t) const {
  assert(t >= begin_t());
  if (t >= steady_.begin_t()) {
    return steady_.at(t);
  }
  KinematicPoint p = traj_(t);
  return p;
}

}  // path_planner
