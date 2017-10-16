//
// Created by Yury Brodskiy on 10/10/17.
//

#ifndef PATH_PLANNING_OPTION_H
#define PATH_PLANNING_OPTION_H

#include "utils.h"

class Road;
class EgoVehicle;
namespace  utils
{
  struct Option
  {
    int target_lane;
    FrenetState start;
    FrenetState goal;
    double start_time;
    double duration;
    Poly2D poly;
    Trajectory trajectory;
    double cost;

  };
  Option makeMaxVelOption(Road& road, EgoVehicle& car, FrenetState start, int targetLine,  double start_time);
}


#endif //PATH_PLANNING_OPTION_H
