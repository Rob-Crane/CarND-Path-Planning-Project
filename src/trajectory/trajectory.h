#pragma once

#include <memory>

#include "Eigen-3.3/Eigen/Dense"
#include "boost/optional.hpp"

#include "types.h"

namespace path_planner {

// TODO - for trajectories that use this, create flag for stale/finished.
class JerkMinimizingTrajectory {
 public:
  JerkMinimizingTrajectory() = default;
  JerkMinimizingTrajectory(const KinematicPoint& p0, const KinematicPoint& p1);
  KinematicPoint operator()(time_point) const;

 private:
  KinematicPoint p0_;
  Eigen::Vector3d coef_;
};

class LateralTrajectory {
 public:
  LateralTrajectory(Dx beg_d, Dv beg_v, Da beg_a, time_point beg_t)
      : begin_d_(beg_d), begin_v_(beg_v), begin_a_(beg_a), begin_t_(beg_t){};
  virtual KinematicPoint at(time_point t) const = 0;
 protected:
  Dx begin_d() const { return begin_d_; }
  Dv begin_v() const { return begin_v_; }
  Da begin_a() const { return begin_a_; }
  time_point begin_t() const { return begin_t_; }

 private:
  Dx begin_d_;
  Dv begin_v_;
  Da begin_a_;
  time_point begin_t_;
};

enum class LaneChangeDirection { kLeft, kRight };

//// Maintain constant lateral velocity until next lane mid point reached.
// class ConstantSpeedLateralTrajectory : public LateralTrajectory {
// public:
// ConstantSpeedLateralTrajectory(Dx beg_d, time_point beg_t, Dv v)
//: LateralTrajectory(beg_d, beg_t), begin_v_(v){};
// KinematicPoint at(time_point t) const override;
// Sv begin_v() const { return begin_v_; }

// private:
// Dv begin_v_;
//};

class SmoothLateralTrajectory : public LateralTrajectory {
 public:
  SmoothLateralTrajectory(Dx beg_d, Dv beg_v, Da beg_a, time_point beg_t,
                          int lane_ind);
  KinematicPoint at(time_point t) const override;

 private:
  JerkMinimizingTrajectory traj_;
  time_point end_t_;
  Dx end_d_;
};


class LongitudinalTrajectory {
 public:
  LongitudinalTrajectory() = default;
  LongitudinalTrajectory(Sx beg_s, Sv beg_v, Sa beg_a, time_point beg_t)
      : begin_s_(beg_s), begin_v_(beg_v), begin_a_(beg_a), begin_t_(beg_t){};
  virtual KinematicPoint at(time_point t) const = 0;
  Sx begin_s() const { return begin_s_; }
  Sv begin_v() const { return begin_v_; }
  Sa begin_a() const { return begin_a_; }
  time_point begin_t() const { return begin_t_; }

 private:
  Sx begin_s_;
  Sv begin_v_;
  Sa begin_a_;
  time_point begin_t_;
};

// Maintain constant forward velocity.
class ConstantSpeedLongitudinalTrajectory : public LongitudinalTrajectory {
 public:
  ConstantSpeedLongitudinalTrajectory() = default;
  ConstantSpeedLongitudinalTrajectory(Sx beg_s, Sv beg_v, time_point beg_t)
      : LongitudinalTrajectory(beg_s, beg_v, 0.0, beg_t){};
  KinematicPoint at(time_point t) const override;
};

class FollowCarTrajectory : public LongitudinalTrajectory {
 public:
  // Blocking vehicle ahead.  Accelerate to goal speed then deccelerate
  // to match speed of blocking vehicle.
  FollowCarTrajectory(Sx beg_s, Sv beg_v, Sa beg_a, time_point beg_t,
                      const ConstantSpeedLongitudinalTrajectory& blocking_traj);
  KinematicPoint at(time_point t) const override;

 private:
  JerkMinimizingTrajectory traj_;
  ConstantSpeedLongitudinalTrajectory steady_;
};

// Accelerate to speed limit and maintain.
class UnblockedLongitudinalTrajectory : public LongitudinalTrajectory {
 public:
  UnblockedLongitudinalTrajectory(Sx beg_s, Sv beg_v, Sa beg_a,
                                  time_point beg_t);
  KinematicPoint at(time_point t) const override;

 private:
  JerkMinimizingTrajectory traj_;
  ConstantSpeedLongitudinalTrajectory steady_;
};

}  // path_planner
