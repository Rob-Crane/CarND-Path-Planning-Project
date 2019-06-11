#pragma once

#include <memory>

#include "boost/optional.hpp"

#include "traj_types.h"

namespace path_planner {

class LateralTrajectory {
 protected:
  LateralTrajectory(Dx begin_d, time_point begin_t)
      : begin_d_(begin_d), begin_t_(begin_t){};
  virtual Dx at(time_point t) const = 0;
  Dx begin_d() const { return begin_d_; }
  time_point begin_t() const { return begin_t_; }

 private:
  Dx begin_d_;
  time_point begin_t_;
};

// Maintain constant lateral velocity until next lane mid point reached.
class ConstantSpeedLateralTrajectory : public LateralTrajectory {
 public:
  ConstantSpeedLateralTrajectory(Dx begin_d, time_point begin_t, Dv v)
      : LateralTrajectory(begin_d, begin_t), begin_v_(v){};
  Dx at(time_point t) const override;
  Sv begin_v() const { return begin_v_; }

 private:
  Dv begin_v_;
};

// Follow a jerk-minimizing lateral trajectory to target lane.
// class SmoothLateralTrajectory : public LateralTrajectory {
//};

class LongitudinalTrajectory {
 public:
  LongitudinalTrajectory(Sx begin_s, Sv begin_v, time_point begin_t)
      : begin_s_(begin_s), begin_v_(begin_v), begin_t_(begin_t){};
  virtual Sx at(time_point t) const = 0;
  Sx begin_s() const { return begin_s_; }
  Sv begin_v() const { return begin_v_; }
  time_point begin_t() const { return begin_t_; }

 private:
  Sx begin_s_;
  Sv begin_v_;
  time_point begin_t_;
};

// Maintain constant forward velocity.
class ConstantSpeedLongitudinalTrajectory : public LongitudinalTrajectory {
 public:
  ConstantSpeedLongitudinalTrajectory(Sx begin_s, Sv begin_v,
                                      time_point begin_t)
      : LongitudinalTrajectory(begin_s, begin_v, begin_t){};
  Sx at(time_point t) const override;
};

class SmoothLongitudinalTrajectory : public LongitudinalTrajectory {
  // No blocking vehicle ahead.  Accelerate to goal speed and maintain.
  SmoothLongitudinalTrajectory(Sx begin_s, Sv begin_v, time_point begin_t)
      : LongitudinalTrajectory(begin_s, begin_v, begin_t) {}
  // Blocking vehicle ahead.  Accelerate to goal speed then deccelerate
  // to match speed of blocking vehicle.
  SmoothLongitudinalTrajectory(
      Sx begin_s, Sv begin_v, time_point begin_t,
      const ConstantSpeedLongitudinalTrajectory& blocking_traj)
      : LongitudinalTrajectory(begin_s, begin_v, begin_t),
        blocking_(blocking_traj) {}
  Sx at(time_point t) const override;

 private:
  Sx blocked(time_point t) const;
  boost::optional<ConstantSpeedLongitudinalTrajectory> blocking_;
  Sx unblocked(time_point t) const;
};

//class Trajectory {
 //public:
  //Trajectory(std::unique_ptr<LateralTrajectory> lateral,
             //std::unique_ptr<LongitudinalTrajectory> longitudinal)
      //: lateral_(std::move(lateral)), longitduinal_(std::move(longitudinal)) {}
  //TrajectoryPoint at(time_point t);

 //private:
  //std::unique_ptr<LateralTrajectory> lateral_;
  //std::unique_ptr<LongitudinalTrajectory> longitundinal_;
//};
}  // path_planner
