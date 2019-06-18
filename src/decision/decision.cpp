#include "decision/decision.h"
#include "config.h"

#include "trajectory/utils.h"

namespace path_planner {

std::vector<TrajectoryState> Decision::plan(
    const std::vector<AdversaryObservation> adversaries,
    InertialVector egoPos) {
  TrajectoryPoint curr_pos(egoPos, route_frame_);
  // Find current position in last trajectory.
  int i = 0;
  for (; i < last_trajectory_.size() &&
         last_trajectory_[i].s() < curr_pos.route().s();
       ++i) {
  }
  std::vector<TrajectoryState> new_trajectory;
  for (int j = std::max(i - 1, 0);
       j < last_trajectory_.size() && j < i + kBuffer; ++j) {
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
    adversaryTrajectories.emplace_back(position, velocity, route_frame_);
  }

  // TODO add route length to adversary S if ...
  int currentLane = lane_number(ref_state.d());
  const double maxS = std::fmod(ref_state.d()+kAheadFilter,kRouteLength);
  TrajectoryVelocity const* minAhead = nullptr;
  for (const auto adv : adversaryTrajectories) {
    Dx dx = adv.route().d();
    Sx sx = adv.route().s();
    if (lane_number(dx) == currentLane && sx > ref_state.d() && sx < maxS) {
      if (minAhead) {
        if (dx < minAhead->route().d()) {
          minAhead = &adv;
        }
      } else {
        minAhead = &adv;
      }
    }
  }

  // Create longitudinal trajectory from most limiting adversary.
  time_point start_time = steady_clock::now();
  std::unique_ptr<LongitudinalTrajectory> longitudinal_traj;
  if (minAhead) {
    ConstantSpeedLongitudinalTrajectory blocking(
        minAhead->route().s(), minAhead->routeV().s(), start_time);

    longitudinal_traj = std::make_unique<FollowCarTrajectory>(
        ref_state.s(), ref_state.sv(), ref_state.sa(), start_time, blocking);
  } else {
    longitudinal_traj = std::make_unique<UnblockedLongitudinalTrajectory>(
        ref_state.s(), ref_state.sv(), ref_state.sa(), start_time);
  }

  // Sample trajectory points.
  for (seconds t = seconds(0.02); t <= seconds(1.0); t += seconds(0.02)) {
    KinematicPoint kp(longitudinal_traj->at(t + start_time));
    RouteVector r(kp.x_, ref_state.d());
    // TODO applty min-S filter here and remove from main.cpp if (r.s() - 
    TrajectoryPoint pt(r, route_frame_);
    new_trajectory.emplace_back(pt, kp.v_, kp.a_);
  }
  last_trajectory_ = new_trajectory;
  return std::move(new_trajectory);
}
}  // path_planner
