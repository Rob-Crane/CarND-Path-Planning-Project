#pragma once
#include <chrono>

namespace path_planner {

using std::chrono::steady_clock;
using seconds = std::chrono::duration<double>;
using time_point = std::chrono::time_point<steady_clock, seconds>;
using milliseconds = std::chrono::duration<double, std::milli>;

using Sx = double;
using Sv = double;
using Sa = double;

using Dx = double;
using Dv = double;
using Da = double;

// One-dimensional kinematics state..
struct KinematicPoint {
  KinematicPoint() = default;
  KinematicPoint(double x, double v, double a, time_point t)
      : x_(x), v_(v), a_(a), t_(t) {}
  double x_;
  double v_;
  double a_;
  time_point t_;
};

}  // path_planner
