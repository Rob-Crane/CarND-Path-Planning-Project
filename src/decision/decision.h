#include <unordered_map>

#include "decision/types.h"
#include "route/route_frame.h"
#include "trajectory/trajectory.h"

namespace path_planner {

using std::chrono::steady_clock;

struct LaneDecision {
  int previous_ = 1;
  int target_ = 1;
  time_point begin_ = steady_clock::now();
};

class LaneDecider {
 public:
  LaneDecision update(double lane_mins[3], Dx dx, Sv sv);

 private:
  bool in_target(Dx dx) const;
  bool in_speed_range(Sv v) const;
  LaneDecision decision_;
};

class Decision {
 public:
  Decision(std::shared_ptr<RouteFrame> route_frame)
      : route_frame_(std::move(route_frame)) {}
  std::vector<TrajectoryState> plan(
      const std::vector<AdversaryObservation> adversaries,
      InertialVector egoPos);

 private:
  int target_lane_;
  std::vector<TrajectoryState> last_trajectory_;
  std::shared_ptr<RouteFrame> route_frame_;
  // Sim bug causes unreliable inertial coordinates at route end,
  // just use dead reckoning after that point.
  double last_x_ = -1.0;
  LaneDecider lane_decider_;
};

}  // path_planner
