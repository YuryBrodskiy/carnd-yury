#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "ekf.h"
using std::vector;

struct Waypoint
{
  double x;
  double y;
};

double deg2rad(double x);
double rad2deg(double x);

class MPC {
  private:
	EKF state_filter_;
    double dt_;
  public:
    MPC();
    virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(vector<double> ptsx_map, vector<double> ptsy_map, double px, double py, double psi, double vel, double delta, double a, double set_vel);


  vector<double> mpc_x_vals;
  vector<double> mpc_y_vals;

  vector<double> next_x_vals;
  vector<double> next_y_vals;
};

#endif /* MPC_H */
