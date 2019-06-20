#include "trajectory.h"

#include <iomanip>
#include <iostream>

#include "types.h"
#include "utils.h"

namespace path_planner {

void test() {
  time_point now = std::chrono::steady_clock::now();
  Sx beg_x = 144.233;
  Sv beg_v = 19.363;
  Sa beg_a = 0.66595;
  time_point beg_t = now;

  constexpr double kAvgAccel = 1.0;  // Est. avg accel to leading veh.
  constexpr double kVMax = 20.00;    // Speed limit
  KinematicPoint curr(beg_x, beg_v, beg_a, now);
  KinematicPoint steady =
      steady_state_max_speed_estimate(curr);
  auto traj = JerkMinimizingTrajectory(curr, steady);
  for (seconds t = seconds(0.02); t <= seconds(1.0); t += seconds(0.02)) {
    if (now+t > steady.t_) break;
    KinematicPoint p = traj(now + t);
    std::cout << "Test s: " << p.x_ << " v: " << p.v_ << " a: " << p.a_
              << " t: " << p.t_.time_since_epoch().count() << std::endl;
  }
}
}  // path_planner

int main() { path_planner::test(); }
