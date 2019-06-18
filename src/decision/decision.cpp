#include "decision/decision.h"
#include "config.h"

#include "trajectory/utils.h"

namespace path_planner {
Trajectory Decision::plan(const std::vector<AdversaryObservation> adversaries) {
  TrajectoryPoint curr_pos(egoPos, routeFrame);
  // Find current position in last trajectory.
  int i = 0;
  for(;i < last_trajectory.size() &&
         last_trajectory[i].s() < curr_pos.route().s(); ++i) {}
  std::vector<TrajectoryState> new_trajectory;
  for (int j = std::max(i - 1, 0);j < last_trajectory.size() && j < i + kBuffer;
       ++j) {
    new_trajectory.push_back(last_trajectory[j]);
  }
  if (new_trajectory.empty()) {
    new_trajectory.emplace_back(curr_pos, 0.0, 0.0);
  }
  // Instead of using "current position", base new trajectories on
  // 'ref_state' - the last point of the  reused trajectory from the
  // previous cycle.
  TrajectoryState ref_state(new_trajectory.back());

  std::vector<TrajectoryVelocity> adversaryTrajectories;
  for (const auto& adv : adversaries) {
    InertialVector position(adv.x_, adv.y_);
    InertialVector velocity(adv.dx_, adv.dy_);
    adversaryTrajectories.push_back((position, velocity, routeFrame);
  }

  int currentLane = lane_number(ref_state.d());
  const double maxS = std::fmod(ref_state.d()+kAheadFilter),kRouteLength);
  AdversaryTrajectory* minAhead = nullptr;
  for (const auto adv : adversaryTrajectories) {
    Dx dx = adv.route().d();
    Sx sx = adv.route().s();
    if (lane_number(dx) == currentLane && sx > ref_state.d() && sx < maxS) {
      if (minAhead) {
        if (dx < minAhead->d()) {
          minAhead = &adv;
        }
      } else {
        minAhead = &adv;
      }
    }
  }


  // Use observations to plan.
  UnblockedLongitudinalTrajectory traj(ref_state.s(), ref_state.sv(),
                                       ref_state.sa(), start_time);
  // Generate longitudinal trajectory.
  for (seconds t = seconds(0.02); t <= seconds(1.0); t += seconds(0.02)) {
    KinematicPoint kp(traj.at(t + start_time));
    RouteVector r(kp.x_, ref_state.d());
    TrajectoryPoint pt(r, routeFrame);
    new_trajectory.emplace_back(pt, kp.v_, kp.a_);
  }
}
}  // path_planner
