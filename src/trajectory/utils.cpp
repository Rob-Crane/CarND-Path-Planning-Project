#include "utils.h"
#include <iostream>

#include "config.h"
namespace path_planner {

int lane_number(Dx x) { return std::floor(x / kLaneWidth); }

bool inEnvelope(Dx ego, Dx adv) {
    double min = ego-kHalfEnvelopeWidth;
    double max = ego+kHalfEnvelopeWidth;
    return adv > min && adv < max;
}

Dx next_lane_midpoint(Dx x) {
  unsigned current_lane = lane_number(x);
  int next_lane = std::min(kMaxLane, current_lane + 1);
  return 2.0 + next_lane * kLaneWidth;
}

Dx prev_lane_midpoint(Dx x) {
  int current_lane = lane_number(x);
  int prev_lane = std::max(0, current_lane - 1);
  return 2.0 + prev_lane * kLaneWidth;
}

double midpoint(int lane_number) {
    return 2.0 + lane_number*kLaneWidth;
}

Eigen::Matrix3d get_tmat(double dt) {
  double t2 = dt * dt;
  double t3 = t2 * dt;
  double t4 = t2 * t2;
  double t5 = t4 * dt;
  Eigen::Matrix3d tmat;
  tmat << t3, t4, t5, 3 * t2, 4 * t3, 5 * t4, 6 * dt, 12 * t2, 20 * t3;
  return std::move(tmat);
}

KinematicPoint steady_state_follow_estimate(KinematicPoint p0,
                                            KinematicPoint blocking) {
  assert(p0.x_ < (blocking.x_ - kMaintainDistance));
  assert(p0.t_ == blocking.t_);
  assert(kFollowAvgAcc > 0);

  double dist = blocking.x_ - p0.x_;
  double dilated_blocking = blocking.v_*kFollowVelGain;
  double rel_vel = dilated_blocking - p0.v_;

  KinematicPoint ret;
  if (rel_vel > 0) {  // Blocking travelling faster than ego.
    double Ta = rel_vel / kFollowAvgAcc;
    assert(Ta > 0);
    ret.x_ = p0.x_ + p0.v_ * Ta + 0.5 * kFollowAvgAcc * Ta * Ta;
    ret.v_ = dilated_blocking;
    ret.a_ = 0.0;
    ret.t_ = p0.t_ + seconds(Ta);
    return ret;
  }
  double Td = rel_vel/kDecel;
  assert(Td > 0);
  ret.x_ = p0.x_ + p0.v_ * Td + 0.5 * kDecel * Td * Td;
  ret.v_ = dilated_blocking;
  ret.a_ = 0.0;
  ret.t_ = p0.t_ + seconds(Td);
  return ret;
}

KinematicPoint steady_state_max_speed_estimate(KinematicPoint p0) {
  assert(p0.v_ < kVMax + kVmaxBuffer);
  double Ta = (kVMax - p0.v_) / kUnblockedAvgAcc;
  KinematicPoint ret;
  ret.x_ = p0.x_ + p0.v_ * Ta + 0.5 * kUnblockedAvgAcc * Ta * Ta;
  ret.v_ = kVMax;
  ret.a_ = 0.0;
  ret.t_ = p0.t_ + seconds(Ta);
  return ret;
}

}  // path_planner
