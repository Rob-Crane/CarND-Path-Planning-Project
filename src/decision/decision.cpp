#include "decision/decision.h"
#include "config.h"
#include "trajectory/types.h"

#include <iostream>  // delete!

#include "trajectory/utils.h"

namespace path_planner {

LaneDecision LaneDecider::update(double lane_mins[3], Dx dx, Sv sv) {
  //std::cout << "target: " << decision_.target_ << std::endl;
  time_point now(steady_clock::now());
  auto time_since_last = now - decision_.begin_;

  if (time_since_last < seconds(kTimeBetweenLaneChange) || !in_target(dx) ||
      !in_speed_range(sv)) {
    if (time_since_last < seconds(kTimeBetweenLaneChange)) {
      // std::cout << "!Time: " << time_since_last.count();
    }
    if (!in_target(dx)) {
      // std::cout << "\t!Target: " << dx;
    }
    if (!in_speed_range(sv)) {
      // std::cout << "\t!Speed: " << sv;
    }
    // std::cout<<std::endl;
    return decision_;
  }
  int lo_lane = std::max(0, decision_.target_ - 1);
  int hi_lane = std::min(2, decision_.target_ + 1);
  Sx target_dist = lane_mins[decision_.target_];
  Sx to_beat = std::min(target_dist, kLaneBehindDist + kLaneAheadDist);
  // std::cout<<"\ttarg_dist: " << target_dist;
  // std::cout<<"\tto_beat: " << to_beat;
  for (int lane = lo_lane; lane <= hi_lane; ++lane) {
    if (lane == decision_.target_) {
      continue;
    }
    Sx lane_min = lane_mins[lane];
    // std::cout<<"\tlane_min[: "<<lane<<"]: "<<lane_min;
    if (lane_min > to_beat) {
      decision_.previous_ = decision_.target_;
      decision_.target_ = lane;
      decision_.begin_ = now;
      break;

      // std::cout<<"\tchange!";
    }
  }
  // std::cout<<std::endl;
  return decision_;
}

bool LaneDecider::in_target(Dx dx) const {
  double target_midpoint(midpoint(decision_.target_));
  constexpr double halfCaptureWidth = kLaneCaptureWidth / 2.0;
  return (target_midpoint - halfCaptureWidth) < dx &&
         (target_midpoint + halfCaptureWidth) > dx;
}

bool LaneDecider::in_speed_range(Sv v) const {
  return kMinLaneChangeSpeed < v && (kVMax - kLaneChangeVelTreshold) > v;
}

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
    new_trajectory.emplace_back(curr_pos, 0.0, 0.0, 0.0, 0.0);
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

  // Get dist = (adversaryS - base) s.t. -L/2 < dist < L/2
  auto boundDist = [](Sx adv, Sx base) {
    Sx d = std::fmod(adv, kRouteLength) - std::fmod(base, kRouteLength);
    constexpr double halfRoute = kRouteLength / 2;
    if (d < -halfRoute) {
      return d + kRouteLength;
    } else if (d > halfRoute) {
      return d - kRouteLength;
    }
    return d;
  };

  double lane_mins[3] = {kMaxLaneDist, kMaxLaneDist, kMaxLaneDist};
  Sx base = ref_state.s() - kLaneBehindDist;
  for (const auto &adv : adversaryTrajectories) {
    // Update dist from rear safety zone for lane changes.
    int lane_ind = lane_number(adv->route().d());
    double dist = boundDist(adv->route().s(), base);
    if (dist > 0.0) {
      lane_mins[lane_ind] = std::min(lane_mins[lane_ind], dist);
    }

    // Get nearest entity in our envelope.
    Sx advS = sAhead(adv->route().s());
    if (inEnvelope(ref_state.d(), adv->route().d()) && advS < maxS) {
      if (minAhead) {
        if (advS < sAhead(minAhead->route().s())) {
          minAhead = adv.get();
        }
      } else {
        minAhead = adv.get();
      }
    }
  }

  LaneDecision lane_decision =
      lane_decider_.update(lane_mins, ref_state.d(), ref_state.sv());
  // std::cout << "Lane decision: " << lane_decision.target_ << std::endl;

  // Create longitudinal trajectory from most limiting adversary.
  time_point start_time = steady_clock::now();
  std::unique_ptr<LongitudinalTrajectory> longitudinal_traj;
  if (minAhead) {
    ConstantSpeedLongitudinalTrajectory blocking(
        sAhead(minAhead->route().s()), minAhead->routeV().s(), start_time);

    longitudinal_traj = std::unique_ptr<FollowCarTrajectory>(
        new FollowCarTrajectory(ref_state.s(), ref_state.sv(), ref_state.sa(),
                                start_time, blocking));
  } else {
    longitudinal_traj = std::unique_ptr<UnblockedLongitudinalTrajectory>(
        new UnblockedLongitudinalTrajectory(ref_state.s(), ref_state.sv(),
                                            ref_state.sa(), start_time));
  }

  std::unique_ptr<LateralTrajectory> lateral_traj;
  //std::cout << "prev: " << lane_decision.previous_
            //<< " midpoint: " << midpoint(lane_decision.previous_) << std::endl;
  lateral_traj = std::unique_ptr<LateralTrajectory>(
      new SmoothLateralTrajectory(midpoint(lane_decision.previous_), 0.0, 0.0,
                                  lane_decision.begin_, lane_decision.target_));

  // Sample trajectory points.
  // std::cout << ">>";
  for (seconds t = seconds(0.02); t <= seconds(1.0); t += seconds(0.02)) {
    KinematicPoint long_pt(longitudinal_traj->at(t + start_time));
    KinematicPoint lat_pt(lateral_traj->at(t + start_time));
    //if (long_pt.x_ - new_trajectory.back().s() >= kMinSDiff) {
      // std::cout << "(x: " << lat_pt.x_ << ", v: " << lat_pt.v_
      //<< ", a: " << lat_pt.a_
      //<< ", t: " << lat_pt.t_.time_since_epoch().count() << ") ";
      RouteVector r(long_pt.x_, lat_pt.x_);
      TrajectoryPoint pt(r, route_frame_);
      new_trajectory.emplace_back(pt, long_pt.v_, long_pt.a_, lat_pt.v_,
                                  lat_pt.a_);
    //}
  }
  // std::cout << std::endl;
  last_trajectory_ = new_trajectory;
  return std::move(new_trajectory);
}
}  // path_planner
