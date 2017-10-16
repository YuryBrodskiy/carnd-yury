//
// Created by Yury Brodskiy on 10/5/17.
//

#include <iostream>

#include "EgoVehicle.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"


using namespace std;
using namespace utils;

Log trLog("../data/fstate.csv");
Log coLog("../data/laneCost.csv");

EgoVehicle::EgoVehicle(Road &road)
    : road_(&road)
    , steps_to_stitch(10)
{

}

void EgoVehicle::update(std::vector<utils::Point> prev_waypoints, double x, double y, double yaw, double s, double d)
{
  size_t numpoints = trajectory_.size();

  for (EgoWaypoints::iterator it = trajectory_.begin(); it != trajectory_.end();)
  {
    if (find(prev_waypoints.begin(), prev_waypoints.end(), it->waypoint) != prev_waypoints.end())
    {
      it++;
    } else
    {
      trLog.fout << it->waypoint.x << ", " << it->waypoint.y << ", " << it->waypoint.s << ", " << it->waypoint.d << ", "
                 << it->fstate[1] << ", " << it->fstate[2] << ", " << it->fstate[4] << ", " << it->fstate[5] << endl;
      trajectory_.erase(it);

    }
  }

  if (trajectory_.size() > steps_to_stitch)
  {
    trajectory_.resize(steps_to_stitch);
  }

  if (trajectory_.size() == 0)
  {

    cout << "start motion, should never arrive here after start" << endl;
    EgoWaypoint first_waypoint;
    first_waypoint.waypoint = road_->getXY(s, d);
    first_waypoint.fstate[0] = s;
    first_waypoint.fstate[1] = 0;
    first_waypoint.fstate[2] = 0;
    first_waypoint.fstate[3] = d;
    first_waypoint.fstate[4] = 0;
    first_waypoint.fstate[5] = 0;
    first_waypoint.target_lane = road_->getLine(d);
    trajectory_.push_back(first_waypoint);
  }
  double t = trajectory_.size() * step_t;
  while (trajectory_.size() < prediction_horizion)
  {
    EgoWaypoints next_waypoints = getTrajectory(trajectory_.back(), t);
    t = t + next_waypoints.size() * step_t;
    trajectory_.insert(trajectory_.end(), next_waypoints.begin(), next_waypoints.end());
  }
  trajectory_.erase(trajectory_.begin());

}

/**
 * Calculates Jerk Minimizing Trajectory for start, end and T.
 * @param start x,dot_x, ddot_x vector at the time 0
 * @param end  x,dot_x, ddot_x vector at the time T
 * @param T    time to complete trajectory
 * @return vector of 5'th order polynomial to satisfy the boundary conditions
 */
vector<double> getJMT(const vector<double> &start, const vector<double> &end, double T)
{
  using namespace Eigen;

  const double a_0 = start[0];
  const double a_1 = start[1];
  const double a_2 = start[2] / 2.0;

  const double c_0 = a_0 + a_1 * T + a_2 * pow(T, 2);
  const double c_1 = a_1 + 2 * a_2 * T;
  const double c_2 = 2 * a_2;

  MatrixXd A(3, 3);
  A << pow(T, 3), pow(T, 4), pow(T, 5),
      3.0 * pow(T, 2), 4.0 * pow(T, 3), 5.0 * pow(T, 4),
      6.0 * T, 12.0 * pow(T, 2), 20.0 * pow(T, 3);
  VectorXd B(3);
  B << end[0] - c_0,
      end[1] - c_1,
      end[2] - c_2;

  VectorXd tail = A.householderQr().solve(B);
  vector<double> alphas = {a_0, a_1, a_2, tail[0], tail[1], tail[2]};
  return alphas;
}


double max_jerk(const vector<double> &coef, double T)
{

  vector<double> s_dot = diff(coef);
  vector<double> s_d_dot = diff(s_dot);
  vector<double> jerk = diff(s_d_dot);
  double mj = abs(polyeval(jerk, 0));
  for (int i = 0; i < 100; i++)
  {
    double cj = abs(polyeval(jerk, T / 100.0 * i));
    if (abs(mj) < cj)
    {
      mj = cj;
    }
  }
  return mj;
}

double max_acc(const vector<double> &coef, double T)
{

  vector<double> s_dot = diff(coef);
  vector<double> s_d_dot = diff(s_dot);
  double ma = abs(polyeval(s_d_dot, 0));
  for (int i = 0; i < 100; i++)
  {
    double ca = abs(polyeval(s_d_dot, T / 100.0 * i));
    if (abs(ma) < ca)
    {
      ma = ca;
    }
  }
  return ma;
}

vector<double> max_state(const vector<double> &coef, double T)
{
  vector<double> s_dot = diff(coef);
  vector<double> s_d_dot = diff(s_dot);
  vector<double> jerk = diff(s_d_dot);
  double mv = abs(polyeval(s_dot, 0));
  double ma = abs(polyeval(s_d_dot, 0));
  double mj = abs(polyeval(jerk, 0));
  for (int i = 0; i < 100; i++)
  {
    double cv = abs(polyeval(s_dot, T / 100.0 * i));
    double ca = abs(polyeval(s_d_dot, T / 100.0 * i));
    double cj = abs(polyeval(jerk, T / 100.0 * i));
    if (abs(mv) < cv)
    {
      mv = cv;
    }
    if (abs(ma) < ca)
    {
      ma = ca;
    }

    if (abs(mj) < cj)
    {
      mj = cj;
    }
  }
  return {mv, ma, mj};
}

Poly2D EgoVehicle::getPoly(const FrenetState &start, const FrenetState &end, double &T)
{

  double _j;
  double _a ;
  Poly2D result ;
  do
  {
    T = T + 0.2;
    double end_pos = start[0] + 0.5 * (start[1] + end[1]) * T;

    vector<double> s_start = {start[0], start[1], start[2]};
    vector<double> s_end = {end_pos, end[1], end[2]};
    vector<double> d_start = {start[3], start[4], start[5]};
    vector<double> d_end = {end[3], end[4], end[5]};

    result.coef_s = getJMT(s_start, s_end, T);
    result.coef_d = getJMT(d_start, d_end, T);

    vector<double> stat_d = max_state(result.coef_d, T);
    vector<double> stat_s = max_state(result.coef_s, T);

    _a = stat_d[1] + stat_s[1];
    _j = stat_d[2] + stat_s[2];


  } while (_j > max_j || _a > max_a);

  return result;

}

Poly2D EgoVehicle::getPolyExact(const utils::FrenetState &start, const utils::FrenetState &end, double &T)
{

  double _j = max_j * 2;
  double _a = max_a * 2;
  double t = T;

  Poly2D result;

  while (_j > max_j || _a > max_a)
  {

    vector<double> s_start = {start[0], start[1], start[2]};
    vector<double> s_end = {end[0], end[1], end[2]};
    vector<double> d_start = {start[3], start[4], start[5]};
    vector<double> d_end = {end[3], end[4], end[5]};

    result.coef_s = getJMT(s_start, s_end, t);
    result.coef_d = getJMT(d_start, d_end, t);

    //TODO actually the same step jerks should be summed and maximum found
    double j_d = max_jerk(result.coef_d, t);
    double j_s = max_jerk(result.coef_s, t);
    _j = sqrt(j_d * j_d + j_s * j_s);
    double a_d = max_acc(result.coef_d, t);
    double a_s = max_acc(result.coef_s, t);
    _a = sqrt(a_d * a_d + a_s * a_s);
    t = t + 0.01;
  }
  T = t - 0.01;


  return result;

}

utils::Poly2D EgoVehicle::getPolySimple(const utils::FrenetState &start, const utils::FrenetState &end, double &T)
{

  vector<double> s_start = {start[0], start[1], start[2]};
  vector<double> s_end = {end[0], end[1], end[2]};

  vector<double> d_start = {start[3], start[4], start[5]};
  vector<double> d_end = {end[3], end[4], end[5]};

  return {getJMT(s_start, s_end, T), getJMT(d_start, d_end, T)};
}

double EgoVehicle::timeToSpeed(double c_speed, double t_speed)
{
  double T = abs(t_speed - c_speed) / max_a;
  if (T < step_t * 5)
  {
    T = prediction_horizion * step_t * 0.3;
  }
  return T;
}

EgoVehicle::EgoWaypoints EgoVehicle::getTrajectory(const EgoVehicle::EgoWaypoint &start, double t)
{

  vector<Option> options;
  for (int lane = 0; lane < road_->num_lanes(); lane++)
  {
    options.push_back(
        makeMaxVelOption(*road_, *this, start.fstate, lane, t)
    );

  }
  vector<Option>::iterator best_option = min_element(options.begin(), options.end(),
                                                     [](const Option &a, const Option &b)
                                                     {
                                                       return a.cost < b.cost;
                                                     });

  EgoVehicle::EgoWaypoints waypoints;
  waypoints.push_back(start);
  double t_current = 0;
  while (t_current < best_option->duration)
  {
    //t_current = t_current + step_t;
    t_current = getWarpedTime(waypoints.back().waypoint, best_option->poly, *road_, t_current, step_t);

    EgoVehicle::EgoWaypoint next;
    next.target_lane = best_option->target_lane;

    next.fstate[0] = road_->range_s(polyeval(best_option->poly.coef_s, t_current));
    next.fstate[1] = polyeval(diff(best_option->poly.coef_s), t_current);
    next.fstate[2] = polyeval(diff(diff(best_option->poly.coef_s)), t_current);

    next.fstate[3] = polyeval(best_option->poly.coef_d, t_current);
    next.fstate[4] = polyeval(diff(best_option->poly.coef_d), t_current);
    next.fstate[5] = polyeval(diff(diff(best_option->poly.coef_d)), t_current);

    next.waypoint = road_->getXY(next.fstate[0], next.fstate[3]);
    waypoints.push_back(next);
  }
  waypoints.erase(waypoints.begin());

  return waypoints;
}


std::vector<std::vector<double> > EgoVehicle::getWaypoints()
{

  vector<vector<double> > result(2);
  result[0].resize(trajectory_.size());
  result[1].resize(trajectory_.size());
  for (int i = 0; i < trajectory_.size(); i++)
  {
    result[0][i] = trajectory_[i].waypoint.x;
    result[1][i] = trajectory_[i].waypoint.y;
  }
  return result;
}
