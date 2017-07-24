#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <chrono>

using CppAD::AD;

// Helper functions
// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.

const double Lf = 2.67;
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


/**
 * A functor to project the observed landmark to the map using particle state
 */
struct ProjectToMap
{
  ProjectToMap(double _px,double _py,double _fi)
  :px(_px),py(_py),fi(_fi){}
    double px;
    double py;
    double fi;
    Waypoint operator () (const Waypoint& lm)
    {
      Waypoint obs_on_map;
      obs_on_map.x = (lm.x-px)* cos(fi) + (lm.y-py)*sin(fi);
      obs_on_map.y = -(lm.x-px)* sin(fi) + (lm.y-py)*cos(fi);
      return obs_on_map;
    }
};

// Evaluate a polynomial in AD and double types
//Adapted from
// https://www.coin-or.org/CppAD/Doc/get_started.cpp.xml
template <class Type>
Type polyeval(const Eigen::VectorXd &a, const Type &x)
{
     Type y   = 0.;  // initialize summation
     Type x_i = 1.;  // initialize x^i
     for(size_t i = 0; i < a.size(); i++)
     {     y   += a[i] * x_i;
           x_i *= x;
     }
     return y;
}

template <class Type>
Type der_polyeval(const Eigen::VectorXd &a, const Type &x)
{
     Type y   = 0.;  // initialize summation
     Type x_i = 1.;  // initialize x^i
     for(size_t i = 1; i < a.size(); i++)
     {     y   += a[i] * x_i * i;
           x_i *= x;
     }
     return y;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

const size_t N = 10; //!< Prediction horizon

// Structure of solver variable vector
// Following indexes are start of the respective vectors of state and control input for time frames 0 - N
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;

class FG_eval {
 private:
	double dt_;				 //<! sample time
	double ref_v_;			 //<! target velocity
	double prev_a_;
	double prev_delta_;
    Eigen::VectorXd coeffs_; //<! Fitted polynomial coefficients
	
 public:

  FG_eval(double dt, double ref_v, double prev_a, double prev_delta, const Eigen::VectorXd& coeffs)
	: dt_(dt)
	, ref_v_(ref_v)
	, prev_a_(prev_a)
	, prev_delta_(prev_delta)
  , coeffs_(coeffs)
	{}
// Adopted from Udacity course example
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

        fg[0] = 0;
        // The part of the cost based on the reference state.
        for (int t = 0; t < N; t++) {

          fg[0] += CppAD::pow(vars[cte_start + t], 2) ;
          fg[0] += CppAD::pow(vars[epsi_start + t], 2)*100;
          fg[0] += CppAD::pow(vars[v_start + t] - ref_v_, 2);
          }

        // Minimize the use of actuators.
        for (int t = 0; t < N - 1; t++) {
          fg[0] += CppAD::pow(vars[delta_start + t], 2)*100;
          fg[0] += CppAD::pow(vars[a_start + t], 2);
        }

		
        // Minimize the value gap between sequential actuations.
		fg[0] += CppAD::pow(vars[delta_start + 0] - prev_delta_, 2)*300; //Needed for EKF solution, for Native == 0
        fg[0] += CppAD::pow(vars[a_start + 0] - prev_a_, 2);
        for (int t = 0; t < N - 2; t++) {
          fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2)*300;
          fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
        }

        //
        // Setup Constraints
        //
        // NOTE: In this section you'll setup the model constraints.

        // Initial constraints
        //
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`.
        // This bumps up the position of all the other values.
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        // The rest of the constraints
        for (int t = 1; t < N; t++) {
          // The state at time t+1 .
          AD<double> x1 = vars[x_start + t];
          AD<double> y1 = vars[y_start + t];
          AD<double> psi1 = vars[psi_start + t];
          AD<double> v1 = vars[v_start + t];
          AD<double> cte1 = vars[cte_start + t];
          AD<double> epsi1 = vars[epsi_start + t];

          // The state at time t.
          AD<double> x0 = vars[x_start + t - 1];
          AD<double> y0 = vars[y_start + t - 1];
          AD<double> psi0 = vars[psi_start + t - 1];
          AD<double> v0 = vars[v_start + t - 1];
          AD<double> cte0 = vars[cte_start + t - 1];
          AD<double> epsi0 = vars[epsi_start + t - 1];

          // Only consider the actuation at time t.
          AD<double> delta0 = vars[delta_start + t - 1];
          AD<double> a0 = vars[a_start + t - 1];

          AD<double> f0 = polyeval(coeffs_, x0);
          AD<double> psides0 = CppAD::atan(der_polyeval(coeffs_, x0));

          // Here's `x` to get you started.
          // The idea here is to constraint this value to be 0.
          //
          // Recall the equations for the model:
          // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
          // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
          // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
          // v_[t+1] = v[t] + a[t] * dt
          // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
          // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
          fg[1 + x_start + t] =       x1 - (x0 + v0 * CppAD::cos(psi0) * dt_);
          fg[1 + y_start + t] =       y1 - (y0 + v0 * CppAD::sin(psi0) * dt_);
          fg[1 + psi_start + t] =   psi1 - (psi0 - v0 * delta0 / Lf * dt_);
          fg[1 + v_start + t] =       v1 - (v0 +  a0 * dt_);
          fg[1 + cte_start + t] =   cte1 - (f0 - (y0 + v0 * CppAD::sin(epsi0) * dt_));
          fg[1 + epsi_start + t] = epsi1 - (psides0 - (psi0 - v0 * delta0 / Lf * dt_) );
        }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC(): dt_(0.06)
{

}
MPC::~MPC() {}

Eigen::VectorXd getTrajectoryFunction(vector<double> ptsx_map, vector<double> ptsy_map, double px, double py, double psi)
{
  ProjectToMap projector(px,py,psi);
  vector<double> ptsx_car;
  vector<double> ptsy_car;
  for(std::vector<double>::iterator itx = ptsx_map.begin(), ity = ptsy_map.begin();itx!=ptsx_map.end();itx++,ity++ )
  {
      Waypoint point = projector({*itx,*ity});
      ptsx_car.push_back(point.x);
      ptsy_car.push_back(point.y);
   }
  Eigen::VectorXd ptsxVec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsx_car.data(), ptsx_car.size());
  Eigen::VectorXd ptsyVec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsy_car.data(), ptsy_car.size());
  Eigen::VectorXd coef = polyfit(ptsxVec, ptsyVec, 4);
  return coef;
}

vector<double> MPC::Solve(vector<double> ptsx_map, vector<double> ptsy_map, double px, double py, double ppsi, double vel, double delta, double a, double ref_v)
{

  const Eigen::VectorXd coeffs0 = getTrajectoryFunction(ptsx_map, ptsy_map, px, py, ppsi);

  // use telemetry t and kinematic model of car motion to predict next state and cycle time
  // EKF is used for prediction
  const Eigen::VectorXd measurement = (Eigen::VectorXd(4) << px, py, ppsi, vel).finished();
  const Eigen::VectorXd control = (Eigen::VectorXd(2) << a, delta).finished();
  const Eigen::VectorXd next_state = state_filter_.update(measurement, control);
  dt_ = next_state[4];


  {
  const double px1 = next_state[0];
  const double py1 = next_state[1];
  const double ppsi1 = next_state[2];

  // fit waypoints on the map using  the state of the car at the begining of the new sample
  const Eigen::VectorXd coeffs1 = getTrajectoryFunction(ptsx_map, ptsy_map, px1, py1, ppsi1);

  next_x_vals.resize(0);
  next_y_vals.resize(0);
  for(size_t i = 0; i < N; i++)
  {
    next_x_vals.push_back(ref_v*dt_*i);
    next_y_vals.push_back(polyeval(coeffs1,next_x_vals.back()));
  }
  }

  typedef CPPAD_TESTVECTOR(double) Dvector;

  // number of independent variables
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -std::numeric_limits<double>::max();
    vars_upperbound[i] = std::numeric_limits<double>::max();
  }
  
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -deg2rad(25);
    vars_upperbound[i] = deg2rad(25);
  }
  
  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
  
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

#ifdef EKF_BASED
  Waypoint p = ProjectToMap(px,py,ppsi)({next_state[0],next_state[1]});
  const double x1 = p.x;
  const double y1 = p.y;
  double psi1 = next_state[2] - ppsi;
  psi1 = atan2(sin(psi1), cos(psi1));
  const double vel1 = next_state[3];
  Eigen::VectorXd state(6);

  const double cte1 = polyeval(coeffs0, x1) - y1;
  const double psi_set1 = atan(der_polyeval(coeffs0, x1));
  const double epsi1 = psi_set1 - psi1;
  state << x1, y1, psi1, vel1, cte1, epsi1;
#else

  const double cte = polyeval(coeffs0, 0) - 0;
  const double psi_set = atan(der_polyeval(coeffs0, 0));
  const double epsi = psi_set - ppsi;

  Eigen::VectorXd state(6);
  state << 0, 0, 0, vel, cte, epsi;
  vars[delta_start]  = vars_upperbound[delta_start]  = vars_lowerbound[delta_start]  = delta;
  vars[a_start] = vars_upperbound[a_start] = vars_lowerbound[a_start] = a;
#endif
  // Set the initial variable values
  vars[x_start]    = constraints_upperbound[x_start]    = constraints_lowerbound[x_start]    = state[0];
  vars[y_start]    = constraints_upperbound[y_start]    = constraints_lowerbound[y_start]    = state[1];
  vars[psi_start]  = constraints_upperbound[psi_start]  = constraints_lowerbound[psi_start]  = state[2];
  vars[v_start]    = constraints_upperbound[v_start]    = constraints_lowerbound[v_start]    = state[3];
  vars[cte_start]  = constraints_upperbound[cte_start]  = constraints_lowerbound[cte_start]  = state[4];
  vars[epsi_start] = constraints_upperbound[epsi_start] = constraints_lowerbound[epsi_start] = state[5];



  // object that computes objective and constraints
  FG_eval fg_eval(dt_, ref_v, a, delta, coeffs0);
  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  //options += "Numeric max_cpu_time          0.5\n";
  options += "Numeric tol          1e-6\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound, constraints_upperbound, fg_eval,
      solution);

  // Check some of the solution values
  bool ok = solution.status == CppAD::ipopt::solve_result<Dvector>::success;


    // Cost
    //auto cost = solution.obj_value;
   //  std::cout << "Cost " << cost << std::endl;

    // TODO: Return the first actuator values. The variables can be accessed with
    // `solution.x[i]`.
    //
  if (ok)
  {
    // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
    // creates a 2 element double vector.
    mpc_x_vals.resize(N);
    mpc_y_vals.resize(N);

    for (size_t i = 0; i < N; i++)
    {
      mpc_x_vals[i] = solution.x[x_start + i];
      mpc_y_vals[i] = solution.x[y_start + i];

    }
#ifdef EKF_BASED
    double steer_value = solution.x[delta_start];  //
    double throttle_value = solution.x[a_start];  //
#else
    double steer_value = solution.x[delta_start+1];  //
    double throttle_value = solution.x[a_start+1];  //
#endif

    return { steer_value, throttle_value};
  }
  else
  {
    std::cout << "not ok" << std::endl;
    return { delta, a};
  }

}
