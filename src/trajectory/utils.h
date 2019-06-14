#pragma once

#include <cmath>

#include "Eigen-3.3/Eigen/Dense"

#include "traj_types.h"

namespace path_planner {

// constexpr Lane[] kLanes = {Lane(0, 0.0, 2.0, 4.0), Lane(1, 4.0, 6.0, 8.0),
// Lane(2, 8.0, 10.0, 12.0), Lane(3, 12.0, 14.0, 16.0)};

namespace {
constexpr double kLaneWidth = 4.0;
constexpr int kMaxLane = 4;
}

Dx next_lane_midpoint(Dx x) {
  int current_lane = std::floor(x / kLaneWidth);
  int next_lane = std::min(kMaxLane, current_lane + 1);
  return 2.0 + next_lane * kLaneWidth;
}

Dx prev_lane_midpoint(Dx x) {
  int current_lane = std::floor(x / kLaneWidth);
  int prev_lane = std::max(0, current_lane - 1);
  return 2.0 + prev_lane * kLaneWidth;
}

struct KinematicPoint {
  KinematicPoint(double x, double v, double a, time_point t)
      : x_(x), v_(v), a_(a), t_(t) {}
  double x_;
  double v_;
  double a_;
  time_point t_;
};

class JerkMinimizingTrajectory {
 public:
  JerkMinimizingTrajectory() = default;
  JerkMinizingTrajectory(const KinematicPoint& p0, const KinematicPoint& p1)
      : p0_(p0) {
    double dt = seconds(p1.t_ - p0.t_).count();
    Eigen::Matri3d tmat(get_tmat(dt));
    Eigen::Vector3d s_terms;
    s_terms << p1.x_ - (p0.x_ + p0.v_ * dt + 0.5 * p0.a_ * dt * dt),
        p1.v_ - (p0.v_ + p0.a_ * dt), p1.a_ - p0.a_;
    coeff_ = tmat.colPivHouseholderQr().solve(s_terms);
  }
  KinematicPoint operator()(time_point t) {
    double dt = seconds(t - p0_.t_).count();
    Eigen::Vector3d smoothing = get_tmat(dt) * coef_;
    KinematicPoint ret;
    ret.x_ = p0_.x_ + p0_.v_ * dt + p0_.a_ * dt * dt + smoothing[0];
    ret.v_ = p0_.v_ + p0_.v_ * dt + smoothing[1];
    ret.a_ = p0_.a_ + smoothing[2];
    return ret;
  }

 private:
  KinematicPoint p0_;
  Eigen::Vector3d coef_;
  Eigen::Matrix3d get_tmat(double dt) {
    double t2 = dt * dt;
    double t3 = t2 * dt;
    double t4 = t2 * t2;
    double t5 = t4 * dt;
    Eigen::Matrix3d tmat;
    tmat << t3, t4, t5, 3 * t2, 4 * t3, 5 * t4, 6 * dt, 12 * t2, 20 * t3;
    return std::move(tmat);
  }
};

// Get approximate closing time to a target driving in lane.  Estimate by
// assuming simple model of instant acceleration to vclose, maintaining
// vclose, then decelerating to match target speed.  If solution doens't exist,
// use nominal value (target is close to Ego).
KinematicPoint steady_state_follow_estimate(KinematicPoint p0,
                                            KinematicPoint blocking,
                                            double a_acc, double a_dec,
                                            double vclose, double xbuff,
                                            double nominal_intercept) {
  assert(p0.x_ < blocking.x_);
  assert(p0.t_ == p1.t_);
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
  double x_intercept = blocking.x_ + blocking.v_*time - xbuff;
  KinematicPoint ret;
  ret.x_ = x_intercept;
  ret.t_ = p0.t_ + seconds(time);
  ret.v_ = p1.v_;
  ret.a_ = 0.0;
  return ret;
}

KinematicPoint steady_state_max_speed_estimate(KinematicPoint p0,
                                     double a_acc, double vmax) {
  assert(a_acc > 0);
  assert(vmax > 0);
  double Ta = (vmax - p0.v_) / a_acc;
  KinematicPoint ret;
  ret.x_ = p0.x_ + p0.v_*Ta + 0.5*a_acc*Ta*Ta;
  ret.v_ = vmax;
  ret.a_ = 0.0;
  ret.t_ = p0.t_ + seconds(Ta);
}

}  // path_planner
