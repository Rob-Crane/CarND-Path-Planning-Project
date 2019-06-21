#include "decision/decision.h"
#include "config.h"

#include <iostream>  // delete!

#include "trajectory/utils.h"

namespace path_planner {

std::vector<TrajectoryState> Decision::plan(
    const std::vector<AdversaryObservation> adversaries,
    InertialVector egoPos) {
  TrajectoryPoint curr_pos(egoPos, route_frame_);
  if (last_x_ < 0) {
    last_x_ = egoPos.x();
  }
  int i = 0;  // Starting position in previous_trajectory to reuse.
  if (last_x_ > 0.0 && std::abs(last_x_ - egoPos.x()) > 100.0) {
    i = 2;  // Use a nominal position in last trajectory as starting point.
  } else {
    last_x_ = egoPos.x();
    // Find current position in last trajectory.
    for (; i < last_trajectory_.size() &&
           std::fmod(last_trajectory_[i].s(), kRouteLength) <
               curr_pos.route().s();
         ++i) {
    }
  }
  std::vector<TrajectoryState> new_trajectory;
  for (int j = std::max(i - 1, 0);
       j < last_trajectory_.size() && j < i + kBuffer; ++j) {
    new_trajectory.push_back(last_trajectory_[j]);
  }
  if (new_trajectory.empty()) {
    new_trajectory.emplace_back(curr_pos, 0.0, 0.0);
  }
  int num_recycle = new_trajectory.size();
  // Instead of using "current position", base new trajectories on
  // 'ref_state' - the last point of the  reused trajectory from the
  // previous cycle.
  TrajectoryState ref_state(new_trajectory.back());

  using TrajectoryVelocityPtr = std::unique_ptr<TrajectoryVelocity>;
  std::vector<TrajectoryVelocityPtr> adversaryTrajectories;
  for (const auto &adv : adversaries) {
    InertialVector position(adv.x_, adv.y_);
    InertialVector velocity(adv.dx_, adv.dy_);
    adversaryTrajectories.emplace_back(
        new TrajectoryVelocity(position, velocity, route_frame_));
  }

  const double maxS = ref_state.s() + kAheadFilter;  // Can exceed kRouteLength.
  TrajectoryVelocity const *minAhead = nullptr;

  // If ego near end of route and adversary ahead of ego, past segment 0,
  // adversary will have low S value compared to ego's high S value.
  // Give all adversaries S-values greater than ego's to address.
  //
  // Appears to be some kind of singularity at the end of route.  Ensuring
  // entities >1.0 ahead appears to filter it out.
  auto sAhead = [&ref_state](Sx advS) {
    while (advS + 1.0 < ref_state.s()) {
      advS += kRouteLength;
    }
    return advS;
  };

  // Get dist = (adversaryS - egoS) s.t. -L/2 < dist < L/2
  auto boundDist = [] (Sx adv, Sx ego) {
    Sx d = std::fmod(adv, kRouteLength)- std::fmod(ego, kRouteLength);
    constexpr halfRoute = kRouteLength/2;
    if (d < -halfRoute) {
      return d+kRouteLength;
    } else if (d > halfRoute) {
        return d-kRouteLength;
    }
    return d;
  }

  double lane_mins[3] = {kMaxLaneDist, kMaxLaneDist, kMaxLaneDist};
  Sx base = ref_state.s() - 40.0;
  for (const auto &adv : adversaryTrajectories) {
    Dx dx = adv->route().d();
    Sx sx = sAhead(adv->route().s());
    int lane_ind = lane_number(dx);
    double lane_dist = adv->route().s() - std::fmod(ref_state.s(), kRouteLength);
    if (lane_dist < lane_mins[lane_ind]) {
      lane_mins[lane_ind] = lane_dist;
    }

    if (inEnvelope(ref_state.d(), dx) && sx < maxS) {
      if (minAhead) {
        if (sx < sAhead(minAhead->route().s())) {
          minAhead = adv.get();
        }
      } else {
        minAhead = adv.get();
      }
    }
  }

  int target_lane = lane_decider.update(lane_mins, lane_number(ref_state.d()));

  // Create longitudinal trajectory from most limiting adversary.
  time_point start_time = steady_clock::now();
  std::unique_ptr<LongitudinalTrajectory> longitudinal_traj;
  if (minAhead) {
    ConstantSpeedLongitudinalTrajectory blocking(
        sAhead(minAhead->route().s()), minAhead->routeV().s(), start_time);
    std::cout << "ref_stateS: " << ref_state.s()
              << " sAhead(minAheadS): " << sAhead(minAhead->route().s())
              << " blocking.beginS(): " << blocking.begin_s()
              << " egoD: " << ref_state.d()
              << " advD: " << minAhead->route().d();

    longitudinal_traj = std::unique_ptr<FollowCarTrajectory>(
        new FollowCarTrajectory(ref_state.s(), ref_state.sv(), ref_state.sa(),
                                start_time, blocking));
  } else {
    std::cout << "unblocked" << std::endl;
    longitudinal_traj = std::unique_ptr<UnblockedLongitudinalTrajectory>(
        new UnblockedLongitudinalTrajectory(ref_state.s(), ref_state.sv(),
                                            ref_state.sa(), start_time));
  }

  std::unique_ptr<LateralTrajectory> lateral_traj;
  lateral_traj = std::unique_ptr<LateralTrajectory>(new SmoothLateralTrajectory(
      ref_state.d(), ref_state.dv(), ref_state.da(), start_time, target_lane));

  // Sample trajectory points.
  for (seconds t = seconds(0.02); t <= seconds(1.0); t += seconds(0.02)) {
    KinematicPoint long_pt(longitudinal_traj->at(t + start_time));
    KinematicPoint lat_pt(lateral_traj->at(t + start_time));
    if (long_pt.x_ - new_trajectory.back().s() >= kMinSDiff) {
      RouteVector r(long_pt.x_, ref_state.d());
      TrajectoryPoint pt(r, route_frame_);
      new_trajectory.emplace_back(pt, long_pt.v_, long_pt.a_);
    }
  }
  last_trajectory_ = new_trajectory;
  return std::move(new_trajectory);
}
}  // path_planner
