#ifndef SRC_UTILS_H_
#define SRC_UTILS_H_
/*
 * utils.h
 *
 *  Created on: Sep 23, 2017
 *      Author: Yury Brodskiy
 */

#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <fstream>
class Road;

namespace utils {
  using namespace std;
  struct Point {
    double x;
    double y;
  };
  struct Waypoint {
    double x;
    double y;
    double s;
    double d;
  };
  typedef std::vector<Waypoint> Waypoints;
  typedef std::array<double, 6> FrenetState;
  struct Poly2D
  {
    std::vector<double> coef_s;
    std::vector<double> coef_d;
  };
  typedef std::vector<FrenetState>  Trajectory;
  Trajectory toTrajectory( const Poly2D& poly, double duration, double step_t, double max_s);


  bool operator ==(const Point& a, const Waypoint& b);

  constexpr double pi() {
    return M_PI;
  }
/**
 * Takes the coefficients of a polynomial and evaluate it at time t
 * @cite Adapted from https://www.coin-or.org/CppAD/Doc/get_started.cpp.xml
 */
  template <class Type>
  Type polyeval(const vector<double> &a, const Type &x)
  {
    Type y   = 0.;  // initialize summation
    Type x_i = 1.;  // initialize x^i
    for (double i : a)
    {
      y   += i * x_i;
      x_i *= x;
    }
    return y;
  }

/** Checks if the SocketIO event has JSON data.
* If there is data the JSON object in string format will be returned,
* else the empty string "" will be returned.
*/
  string hasData(string s);

  double distance(double x1, double y1, double x2, double y2);

  int ClosestWaypoint(double x, double y, const vector<Waypoint> &map_waypoints);
  int NextWaypoint(double x, double y, double theta, const vector<Waypoint> &map_waypoints);


  vector<Point> toWaypoints(const vector<double> &X, vector<double> Y);

  vector<vector<double> > toVector(const vector<Waypoint> &waypoints);


  vector<double> diff(const vector<double> &coef);


  double logistic(double x);


  /**
 * A functor to project the observed landmark to the map using particle state
 */
  struct ProjectToLocal {
    ProjectToLocal(double _px, double _py, double _fi)
        : px(_px), py(_py), fi(_fi) {}

    ProjectToLocal(double _px1, double _py1, double _px2, double _py2)
        : px(_px1), py(_py1), fi(atan2(_py2 - _py1, _px2 - _px1)) {}


    double px;
    double py;
    double fi;

    ProjectToLocal inverse() {

      Point Origin = this->operator()((Point) {0, 0});
      return {Origin.x, Origin.y, -fi};
    }

    Point operator()(const Point &lm) {
      Point obs_on_map;
      obs_on_map.x = (lm.x - px) * cos(fi) + (lm.y - py) * sin(fi);
      obs_on_map.y = -(lm.x - px) * sin(fi) + (lm.y - py) * cos(fi);
      return obs_on_map;
    }

    Waypoint operator()(const Waypoint &lm) {
      Waypoint obs_on_map;
      obs_on_map.x = (lm.x - px) * cos(fi) + (lm.y - py) * sin(fi);
      obs_on_map.y = -(lm.x - px) * sin(fi) + (lm.y - py) * cos(fi);
      obs_on_map.s = lm.s;
      obs_on_map.d = lm.d;
      return obs_on_map;
    }
  };

  double getWarpedTime(const Waypoint& waypoint, const Poly2D& poly, Road& road, double t_current, double step_t);

  class Log
  {
  public:
    explicit Log(std::string file_to_write)
    {
      fout.open(file_to_write, std::ofstream::out | std::ofstream::trunc);
    }
    ~Log()
    {
      fout.close();
    }
    std::ofstream fout;
  };

}
#endif /* SRC_UTILS_H_ */
