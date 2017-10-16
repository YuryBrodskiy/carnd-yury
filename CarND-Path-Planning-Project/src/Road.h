//
// Created by Yury Brodskiy on 10/5/17.
//

#ifndef PATH_PLANNING_ROAD_H
#define PATH_PLANNING_ROAD_H

#include <string>
#include <map>
#include "utils.h"
#include "Option.h"

class Road {
public:
  struct Vehicle
  {
    std::vector<double> vehicleSize = {5,2};
    void update(std::vector<double> sensor_fusion_item);
    utils::FrenetState state_in(double t);
    utils::FrenetState state_;
    int id_;
  };

  explicit Road(std::string road_waipoint_file);

  ~Road() = default;

  const double max_s     = 6945.554; //[m]
  const double max_v     = 21;    //[m/s]
  const double line_width  = 4; //[m]
  // Transform from Frenet s,d coordinates to Cartesian x,y
  utils::Waypoint getXY(double s, double d);

  double range_s(double s) const;

  double get_out_of_line_driving_cost(const utils::Option& option) const;
  double get_out_of_road_driving_cost(const utils::Option& option) const;
  double get_collision_cost(const utils::Option& option) const;


  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  std::vector<double>  getFrenet(double x, double y, double theta);

  Road::Vehicle getVehicleToFollow(int line, double s, double t);
  int getLine(double d) const
  {
    return static_cast<int>(floor(d/line_width));
  }
  double getD(int line) const
  {
    return line_width*(line + 0.5);
  }
  int num_lanes() const
  {
    return 3;
  }

  void update(const std::vector<std::vector<double> > & sensor_fusion);

private:
  utils::Waypoints map_waypoints_;
  std::map<int, Vehicle> vehicles_;
  int prev_wp_;
  utils::Waypoints
  getHighDefinitionMap(const utils::Waypoints &map_points_orig);
};


#endif //PATH_PLANNING_ROAD_H
