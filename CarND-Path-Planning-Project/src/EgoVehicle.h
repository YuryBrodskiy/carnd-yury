//
// Created by Yury Brodskiy on 10/5/17.
//

#ifndef PATH_PLANNING_ERGOVEHICLE_H
#define PATH_PLANNING_ERGOVEHICLE_H


#include "utils.h"
#include "Road.h"

class EgoVehicle {
public:
  std::vector<double> vehicleSize = {5,2};

  struct EgoWaypoint
  {
    utils::Waypoint waypoint;
    utils::FrenetState fstate;
    int target_lane;
  };
  typedef std::vector<EgoWaypoint> EgoWaypoints;
public:
  explicit EgoVehicle(Road& road);
  ~EgoVehicle() = default;
  void update(const std::vector<utils::Point>& prev_waypoints, double x, double y, double yaw, double s, double d);
  EgoWaypoints getTrajectory(const EgoWaypoint& waypoint, double d);
  std::vector<std::vector<double> > getWaypoints();
  utils::Poly2D getPoly(utils::FrenetState start,  utils::FrenetState end, double& T);
  utils::Poly2D getPolyExact(const utils::FrenetState& start, const utils::FrenetState& end, double& T);
  utils::Poly2D getPolySimple(const utils::FrenetState& start, const utils::FrenetState& end, double& T);
  double timeToSpeed(double c_speed, double t_speed);



private:

  EgoWaypoints trajectory_;

public:
  Road* road_;
  const double max_a     = 10*0.7;      //[m/s^2]
  const double max_j     = 50;          //[m/s^3]

  const size_t steps_to_stitch  = 15; // path planner does not have duration to re-plan any thing for steps_to_stitch*step_t after updapte is called

  const size_t prediction_horizion  = 100;
  const double step_t    = 0.02;     //[sec]

};


#endif //PATH_PLANNING_ERGOVEHICLE_H
