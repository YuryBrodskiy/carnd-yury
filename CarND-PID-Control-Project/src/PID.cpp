#include "PID.h"
#include <iostream>
#include <chrono>
#include <numeric>
#include <math.h>

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {}

PID::~PID() {}

PID& PID::Init(double Kp, double Ki, double Kd) {
  K[P] = Kp;
  K[I] = Ki;
  K[D] = Kd;
  using namespace  std::chrono;
  prev_time = duration_cast< milliseconds >(
      system_clock::now().time_since_epoch()
  ).count();
  prev_error = 0;
  int_error = 0;
  return *this;
}

void PID::InitTwiddle(double dKp, double dKi, double dKd) {
  dK[P] = dKp;
  dK[I] = dKi;
  dK[D] = dKd;
  optimization_value = 0;
  optimization_value_best = numeric_limits<double>::max();
  opt_dim = (Dimention)0;
  twiddle_state = start;
}

PID& PID::UpdateError(double error) {
  using namespace  std::chrono;
  double new_time = duration_cast< milliseconds >(
      system_clock::now().time_since_epoch()
  ).count();
  double dt = (new_time - prev_time)/1000; //[sec]

  if(dt > 0.06) // if cycle time is double of expected, reset controller
  {
    dt = 0.05;
    prev_error = error;
    int_error = 0;
  }

  prev_time = new_time;
  double d_error = (error - prev_error)/dt;
  prev_error = error;
  int_error += error*dt;
  control_value = K[P]*error + K[D]*d_error + K[I]*int_error;
  //minimize the cte and control inputs
  optimization_value += error*error + 0.01*control_value*control_value;
  return *this;
}


double PID::control() {
  return control_value;
}

void PID::twiddle_step() {
  // check if precision boundary is crossed
  if(std::inner_product(std::begin(dK), std::end(dK) ,std::begin(dK), 0.0)< 1e-5)
  {
      std::cout<<"Final result:"<< std::endl;
      std::cout<<" Kp = "<<K[P]<<"\t Ki = "<<K[I]<<"\t Kd = "<<K[D]<<std::endl;
      std::cout<<"dKp = "<<dK[P]<<"\tdKi = "<<dK[I]<<"\tdKd = "<<dK[D]<<std::endl;
      std::cout<<" op = "<<optimization_value<<"\topB = "<<optimization_value_best<<"\tDim = " << opt_dim << std::endl;
    return; //nothing to do
  }
  std::cout<<" Kp = " << K[P]  <<"\t Ki = "<<  K[I] <<"\t Kd = " << K[D] <<std::endl;
  std::cout<<"dKp = " << dK[P] <<"\tdKi = "<< dK[I] <<"\tdKd = " << dK[D] <<std::endl;
  std::cout<<"op = " << optimization_value << "\topB = " << optimization_value_best << "\tDim = " << opt_dim << std::endl;

  // Twiddle algorithm is modified such that the function can be called recurrently
  // everytime performing one single parameter update
  switch (twiddle_state)
  {
  case start:
    if (optimization_value < optimization_value_best )
    {
      optimization_value_best = optimization_value;
      dK[opt_dim] = dK[opt_dim] * 1.5;
      opt_dim = (Dimention)((opt_dim+1) % 3);
      K[opt_dim] = K[opt_dim] + dK[opt_dim];
    }
    else
    {
      K[opt_dim] = K[opt_dim]- 2.0*dK[opt_dim];
      twiddle_state = change;
    }

    break;
  case change:
    if (optimization_value < optimization_value_best )
    {
       optimization_value_best = optimization_value ;
      dK[opt_dim] = -dK[opt_dim];
    }
    else
    {
      K[opt_dim] = K[opt_dim] + dK[opt_dim];
      dK[opt_dim] = dK[opt_dim]/2.0;
    }
    opt_dim = (Dimention)((opt_dim+1) % 3);
    twiddle_state = start;
    K[opt_dim] = K[opt_dim] + dK[opt_dim];
    break;
  }
  optimization_value = 0;
  int_error = 0;
}



