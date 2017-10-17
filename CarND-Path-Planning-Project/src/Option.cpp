//
// Created by Yury Brodskiy on 10/10/17.
//

#include <random>
#include "Option.h"
#include "Road.h"
#include "EgoVehicle.h"

using namespace utils;
using namespace std;


namespace utils
{
  Option
  makeMaxVelOption(Road &road, EgoVehicle &car, FrenetState start, int target_lane, double start_time)
  {
    Option option;
    double target_v = road.max_v;
    double T = car.timeToSpeed(start[1], target_v);
    double speednow = sqrt(pow(start[1],2)+pow(start[4],2));
    double target_s = start[0] + (target_v + speednow) * 0.5 * T;

    Road::Vehicle vehicleToFollow = road.getVehicleToFollow(target_lane, start[0], start_time);

    if (vehicleToFollow.id_ != -1)
    {
      double distance_at_start = road.range_s(road.range_s(vehicleToFollow.state_in(start_time)[0]) - road.range_s(start[0]));
      double distance_at_end = road.range_s(road.range_s(vehicleToFollow.state_in(T + start_time)[0]) - road.range_s(target_s));
      double reaction_distance = 3.0 * target_v; //5.0 seconds reaction duration

      if (!(distance_at_start > reaction_distance && distance_at_end > reaction_distance))
      {
        //target_v = min(target_v, vehicleToFollow.state_in(start_time)[1]);
        const double dist_catch_up = min(distance_at_start, distance_at_end) - vehicleToFollow.vehicleSize[0]*4;
        target_v = min(target_v, vehicleToFollow.state_in(start_time)[1] + dist_catch_up );


        T = car.timeToSpeed(speednow, target_v);

        T = max(T, 3*(start[3] - road.getD(target_lane))/max(0.01, speednow));
        option.target_lane = target_lane;

        target_s = start[0] + (target_v + speednow) * 0.5 * T;

        option.goal = {target_s, target_v, 0, road.getD(option.target_lane), 0, 0};
        option.start = start;

        option.duration = T;


        option.poly = car.getPoly(option.start, option.goal, option.duration);
        option.trajectory = toTrajectory(option.poly,option.duration, car.step_t,road.max_s);
        //double d_at_end = road.range_s(road.range_s(vehicleToFollow.state_in(T + start_time)[0]) - road.range_s(option.trajectory.back()[0]));
        option.cost  = -target_v*2;


        option.cost += road.get_collision_cost(option)*100;
        option.cost += road.get_out_of_line_driving_cost(option);
        //option.cost += road.get_out_of_road_driving_cost(option);
        return option;
      }

    }
    option.target_lane = target_lane;
    option.goal = {target_s, target_v, 0, road.getD(option.target_lane), 0, 0};
    option.start = start;
    option.duration = T;

    option.poly = car.getPoly(option.start, option.goal, option.duration);
    option.trajectory = toTrajectory(option.poly,option.duration, car.step_t,road.max_s);
    option.cost  = -target_v*2  - 20;


    option.cost += road.get_collision_cost(option)*100;
    option.cost += road.get_out_of_line_driving_cost(option);
    //option.cost += road.get_out_of_road_driving_cost(option);
    return option;
  }

}
