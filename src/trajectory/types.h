#pragma once
#include <chrono>

namespace path_planner {

using std::chrono::steady_clock;
using seconds = std::chrono::duration<double>;
using time_point = std::chrono::time_point<std::chrono::steady_clock, seconds>;
using std::chrono::milliseconds;

using Sx = double;
using Sv = double;
using Sa = double;

using Dx = double;
using Dv = double;
using Da = double;

}  // path_planner
