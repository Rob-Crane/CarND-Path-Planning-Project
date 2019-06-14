#include "trajectory.h"

#include <iomanip>
#include <iostream>

#include "types.h"
#include "utils.h"

namespace path_planner {

void test() {
  time_point now = std::chrono::steady_clock::now();
  Sx beg_x = 49.0;
  Sv beg_v = 2.0;
  Sa beg_a = 0.0;
  time_point beg_t = now;

  Sx Xao = 50.0;
  Sv Va = 10.0;
  ConstantSpeedLongitudinalTrajectory blocking(Xao, Va, now);
  FollowCarTrajectory following(beg_x, beg_v, beg_a, now, blocking);
  for (seconds t = seconds(0.2); t < seconds(7.0); t += seconds(0.2)) {
    std::cout << t.count() << ", " << blocking.at(now + t) << ", "
              << following.at(now + t) << std::endl;
  }
}
} // path_planner

int main() { path_planner::test(); }
