#include <unordered_map>

#include "route/route_frame.h"
#include "decision/types.h"

namespace path_planner {

class Decision {
public:
  Trajectory plan(const std::vector<AdversaryObservation> adversaries, InertialVector egoPos, RouteFrame* routeFrame);
private:
  int target_lane_
  std::vector<TrajectoryState> last_trajectory_;
   
}






}  // path_planner
