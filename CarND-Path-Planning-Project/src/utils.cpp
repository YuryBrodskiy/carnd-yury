/*
 * utils.cpp
 *
 *  Created on: Sep 23, 2017
 *      Author: yurybrodskiy
 */
#include <iostream>
#include "utils.h"
#include "spline.h"
#include "Road.h"

namespace utils
{

  bool operator==(const Point &a, const Waypoint &b)
  {
    return distance(a.x, a.y, b.x, b.y) < 0.001;
  }

  string hasData(string s)
  {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of('[');
    auto b2 = s.find_first_of('}');
    if (found_null != string::npos)
    {
      return "";
    } else if (b1 != string::npos && b2 != string::npos)
    {
      return s.substr(b1, b2 - b1 + 2);
    }

    return "";
  }


  double distance(double x1, double y1, double x2, double y2)
  {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  }

  int NextWaypoint(double x, double y, double theta, const vector<Waypoint> &map_waypoints)
  {
    int closestWaypoint = ClosestWaypoint(x, y, map_waypoints);
    double map_x = map_waypoints[closestWaypoint].x;
    double map_y = map_waypoints[closestWaypoint].y;
    double heading = atan2((map_y - y), (map_x - x));
    double angle = fabs(theta - heading);
    if (angle > pi() / 4)
    {
      closestWaypoint++;
    }
    return closestWaypoint;
  }

  int ClosestWaypoint(double x, double y, const vector<Waypoint> &map_waypoints)
  {
    double closestLen = 100000; //large number
    int closestWaypoint = 0;
    for (int i = 0; i < map_waypoints.size(); i++)
    {
      double map_x = map_waypoints[i].x;
      double map_y = map_waypoints[i].y;
      double dist = distance(x, y, map_x, map_y);
      if (dist < closestLen)
      {
        closestLen = dist;
        closestWaypoint = i;
      }
    }
    return closestWaypoint;
  }


  vector<Point> toWaypoints(const vector<double> &X, vector<double> Y)
  {
    vector<Point> result(X.size());
    assert(X.size() == Y.size());
    for (int i = 0; i < X.size(); i++)
    {
      result[i] = {X[i], Y[i]};
    }
    return result;
  }


  vector<vector<double> > toVector(const vector<Waypoint> &waypoints)
  {
    vector<vector<double> > result(4);
    result[0].resize(waypoints.size());
    result[1].resize(waypoints.size());
    result[2].resize(waypoints.size());
    result[3].resize(waypoints.size());
    for (int i = 0; i < waypoints.size(); i++)
    {
      result[0][i] = waypoints[i].x;
      result[1][i] = waypoints[i].y;
      result[2][i] = waypoints[i].s;
      result[3][i] = waypoints[i].d;
    }
    return result;
  }


  vector<double> diff(const vector<double> &coef)
  {
    vector<double> result(coef.size() - 1);
    result.assign(coef.begin() + 1, coef.end());
    for (int i = 0; i < result.size(); i++)
    {
      result[i] *= (i + 1);
    }
    return result;
  }

  double getWarpedTime(const Waypoint &waypoint, const Poly2D &poly, Road &road, double t_current, double step_t)
  {

    double s_t = road.range_s(polyeval(poly.coef_s, t_current + step_t));
    double d_t = polyeval(poly.coef_d, t_current + step_t);
    double target_dist = sqrt(pow(road.range_s(s_t - waypoint.s), 2) + pow(d_t - waypoint.d, 2));

    auto timeToDist = [&road, &poly, &waypoint](double time)
    {
      double s = road.range_s(polyeval(poly.coef_s, time));
      double d = polyeval(poly.coef_d, time);
      Waypoint target = road.getXY(s, d);
      return sqrt(pow(waypoint.x - target.x, 2) + pow(waypoint.y - target.y, 2));
    };


    double lower_t = t_current + step_t * 0.1;
    double lower_d = timeToDist(lower_t);
    double upper_t = t_current + step_t * 1.9;
    double upper_d = timeToDist(upper_t);

    int max_it = 30;
    while (abs(lower_d - upper_d) > 1e-6 && max_it > 0) // 0.1 mm/0.02 sec
    {
      max_it--;
      double mid_t = (upper_t - lower_t) * 0.5 + lower_t;
      double mid_d = timeToDist(mid_t);
      if (mid_d < target_dist)
      {
        lower_t = mid_t;
        lower_d = mid_d;
      } else
      {
        upper_t = mid_t;
        upper_d = mid_d;
      }
    }

    return lower_t;

  }

/**
 *    A function that returns a value between 0 and 1 for x in the
 *  range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
 *      Useful for cost functions.
 * @param x
 * @return
 */
  double logistic(double x)
  {
    return 2.0 / (1 + exp(-x)) - 1.0;
  }

  Trajectory toTrajectory(const Poly2D &poly, double duration, double step_t, double max_s)
  {
    Trajectory result;
    double t_current = 0;
    while (t_current < duration)
    {
      t_current = t_current + step_t;
      result.push_back(
          {

              fmod(polyeval(poly.coef_s, t_current), max_s),
              polyeval(diff(poly.coef_s), t_current),
              polyeval(diff(diff(poly.coef_s)), t_current),

              polyeval(poly.coef_d, t_current),
              polyeval(diff(poly.coef_d), t_current),
              polyeval(diff(diff(poly.coef_d)), t_current),
          });
    }

    return result;
  }

}




