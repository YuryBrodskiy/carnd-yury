#include "ekf.h"
#include <chrono>
#include "Eigen/Dense"
#include <iostream>

const double Lf = 2.67;
using namespace Eigen;
using namespace std;
EKF::EKF()
    : prev_time_(0)
    , nis_(0)
    , x_(VectorXd::Zero(5))
    , P_(MatrixXd::Identity(5,5)*1e-4)
    , T_(MatrixXd::Zero(5,5))
    , R_(MatrixXd::Identity(4,4)*1e-4) // numerical error in measurements
    , I_(MatrixXd::Identity(5,5))
{
  // following are experemetnaly determined good values
  T_(0,0)=0.3;
  T_(1,1)=0.3;
  T_(2,2)=0.2;
  T_(3,3)=1.0;
  T_(4,4)=1e-8;
}

Eigen::VectorXd EKF::update( const Eigen::VectorXd& measure, const Eigen::VectorXd& control )
{
    using namespace  std::chrono;
    double new_time = duration_cast< milliseconds >(system_clock::now().time_since_epoch()).count();
    if((new_time - prev_time_)>1000) // more the 1 sec between calls re-initialization required
    {
        x_ << measure[0], measure[1],measure[2],measure[3],0.06;
        P_ = MatrixXd::Identity(5,5)*1e-4;
        std::cout<<"reset"<<endl;
    }
    prev_time_ = new_time;
    //measurement update
    MatrixXd H = model_H();
    
    const VectorXd y = model_diff(measure, x_);
    const MatrixXd Ht = H.transpose();
    const MatrixXd S = H * P_ * Ht + R_;
    const MatrixXd Si = S.inverse();
    const MatrixXd PHt = P_ * Ht;
    const MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    P_ = (I_ - K * H) * P_;
    // compute nis for tuning of the filter parameters
    nis_ = y.transpose() * Si * y;
    //std::cout<<nis_<<"\t"<<x_[4]<<std::endl;
    // step ahead prediction
    
    const MatrixXd F = model_J(x_, control);
    const MatrixXd T = model_T(x_, control);
    //new estimate
    x_ =  model_step(x_, control);
    P_ = F * P_* F.transpose()  + T;

    return x_;
}

MatrixXd EKF::model_H() const 
{
    MatrixXd result = MatrixXd::Zero(4,5);
    result.topLeftCorner(4,4) = MatrixXd::Identity(4,4);
    return result;
}

VectorXd EKF::model_diff( const Eigen::VectorXd& measure, const Eigen::VectorXd& x ) const
{
   const MatrixXd J = model_H();
   VectorXd  err = measure - J* x;
   err[2] = atan2(sin(err[2]),cos(err[2]));
   return err;
}

MatrixXd EKF::model_J(const Eigen::VectorXd& x, const Eigen::VectorXd& control ) const
{
    MatrixXd result = MatrixXd::Zero(5,5);

    const double psi0 = x[2];
    const double v0 = x[3];
    const double dt = x[4];

    const double a0 = control[0];
    const double delta0 = control[1];

    result<< 1, 0, -v0*sin(psi0)*dt,      cos(psi0)*dt,      v0 * cos(psi0),
             0, 1,  v0*cos(psi0)*dt,      sin(psi0)*dt,      v0 * sin(psi0),
             0, 0,                1, -delta0 / Lf * dt,  - v0 * delta0 / Lf,
             0, 0,                0,                 1,                  a0,
             0, 0,                0,                 0,                   1;
    return result;
}

MatrixXd EKF::model_T(const Eigen::VectorXd& x, const Eigen::VectorXd& control ) const
{
  
    MatrixXd G = model_J(x, control);

    return G*T_*G.transpose();
}

Eigen::VectorXd EKF::model_step(const Eigen::VectorXd& x, const Eigen::VectorXd& control ) const
{
    double x0 = x[0];
    double y0 = x[1];
    double psi0 = x[2];
    double v0 = x[3];
    double dt = std::fabs(x[4]);

    const double a0 = control[0];
    const double delta0 = control[1];

    VectorXd result = VectorXd::Zero(5);

      x0 = x0 + v0 * cos(psi0) * dt;
      y0 = y0 + v0 * sin(psi0) * dt;
      psi0 = psi0 - v0 * delta0 / Lf * dt;
      v0 = v0 + a0 * dt;
    return (VectorXd(5)<< x0,y0,psi0,v0,dt).finished();
}
