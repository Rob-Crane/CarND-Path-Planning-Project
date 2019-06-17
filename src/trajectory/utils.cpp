#include "utils.h"
#include <iostream>
namespace path_planner {

int lane_number(Dx x, const double kLaneWidth) {
  return std::floor(x / kLaneWidth);
}

Dx next_lane_midpoint(Dx x, const double kLaneWidth, const int kMaxLane) {
  int current_lane = lane_number(x, kLaneWidth);
  int next_lane = std::min(kMaxLane, current_lane + 1);
  return 2.0 + next_lane * kLaneWidth;
}

Dx prev_lane_midpoint(Dx x, const double kLaneWidth) {
  int current_lane = lane_number(x, kLaneWidth);
  int prev_lane = std::max(0, current_lane - 1);
  return 2.0 + prev_lane * kLaneWidth;
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
                                            KinematicPoint blocking,
                                            double a_acc, double a_dec,
                                            double vclose, double xbuff,
                                            double nominal_intercept) {
  assert(p0.x_ < blocking.x_);
  assert(p0.t_ == blocking.t_);
  assert(a_dec < 0);
  double Ta = -(p0.v_ - vclose) / a_acc;  // time spend accelerating
  double Tc =
      -(a_acc * blocking.v_ * blocking.v_ +  // time spent at constant speed
        a_dec * p0.v_ * p0.v_ + (a_acc - a_dec) * vclose * vclose -
        2 * a_acc * a_dec * (p0.x_ - blocking.x_ + xbuff) +
        2 * a_dec * blocking.v_ * (vclose - p0.v_) -
        2 * a_acc * blocking.v_ * vclose) /
      (2.0 * a_acc * a_dec * (blocking.v_ - vclose));
  double Td = (blocking.v_ - vclose) / a_dec;  // time spent decelerating

  double time = nominal_intercept;
  if (Ta > 0.0 && Tc > 0.0 && Td > 0.0) {
    time = Ta + Tc + Td;
  }
  double x_intercept = blocking.x_ + blocking.v_ * time - xbuff;
  KinematicPoint ret;
  ret.x_ = x_intercept;
  ret.t_ = p0.t_ + seconds(time);
  ret.v_ = blocking.v_;
  ret.a_ = 0.0;
  return ret;
}

KinematicPoint steady_state_max_speed_estimate(KinematicPoint p0, double a_acc,
                                               double vmax) {
  assert(a_acc > 0);
  assert(vmax > 0);
  constexpr double kVmaxBuffer = 0.5;
  assert(p0.v_<vmax+kVmaxBuffer);
  double Ta = (vmax - p0.v_) / a_acc;
  KinematicPoint ret;
  ret.x_ = p0.x_ + p0.v_ * Ta + 0.5 * a_acc * Ta * Ta;
  ret.v_ = vmax;
  ret.a_ = 0.0;
  ret.t_ = p0.t_ + seconds(Ta);
  return ret;
}

} // path_planner
