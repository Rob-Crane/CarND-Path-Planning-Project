#pragma once
#include <route_types.h>
#include <chrono>

namespace path_planner {

using std::chrono::steady_clock;
using std::chrono::time_point<std::chrono::steady_clock>;
using seconds = std::chrono::duration<double>;
using std::chrono::milliseconds;

using Sx = double;
using Sv = double;

using Dx = double;
using Dv = double;

//struct Lane {
  //Lane(unsigned id, Dx left, Dx middle, Dx right)
      //: id_(id), left_(left), middle_(middle), right_(right) {}
  //unsigned id_;
  //Dx left_;
  //Dx middle_;
  //Dx right_;
//};

}  // path_planner
