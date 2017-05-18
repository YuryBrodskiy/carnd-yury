#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

const double KalmanFilter::kTimeToUnknown = 3600; //[sec] time with out observing object

KalmanFilter::KalmanFilter() :
    I_(MatrixXd::Identity(4, 4)), Qv_(MatrixXd::Zero(2, 2))
{

}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::Init(const Eigen::VectorXd &measurement, const Sensor& sensor)
{
  x_ = sensor.InitialValues(measurement);
  P_ = Eigen::MatrixXd::Zero(4, 4);
  // Initialize the covariance matrix to unknown state
  PredictMotion(kTimeToUnknown);
  // Now normal update will bring the covariance matrix to sensor almost to sensor values
  Update(measurement, sensor);
}

void KalmanFilter::PredictMotion(const double& dt)
{
  //Process noise model
  MatrixXd G = MatrixXd(4, 2);
  G << dt * dt / 2.0,  0.0,
                 0.0, dt * dt / 2.0,
                  dt, 0.0,
                 0.0, dt;

  MatrixXd Q_ = G * Qv_ * G.transpose();

  //Motion model
  MatrixXd F(4, 4);
  F << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

  MatrixXd Ft = F.transpose();
  x_ = F * x_;
  P_ = F * P_ * Ft + Q_;
}


void KalmanFilter::Update(const Eigen::VectorXd &measurement, const Sensor& sensor)
{
  MatrixXd H = sensor.CalculateJacobian(x_);
  // if estimated Jacobian is singular, use a measurement values to get an estimation of the Jacobian
  if (H.norm() < 1e-10)
  {
    H = sensor.CalculateJacobian(sensor.InitialValues(x_));
  }
  // in here we rely on the fact that EKF and KF are identical equations if measurement function is linear
  // thus by encapsulating properties of the measurement function into sensor class
  // EKF equations can be reused
  const VectorXd y = sensor.MeasurementError(measurement, x_);
  const MatrixXd Ht = H.transpose();
  const MatrixXd S = H * P_ * Ht + sensor.Noise();
  const MatrixXd Si = S.inverse();
  const MatrixXd PHt = P_ * Ht;
  const MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * H) * P_;
}

void KalmanFilter::SetProcessNoise(const Eigen::MatrixXd &noise)
{
  Qv_ = noise;
}

void Sensor::SetProcessNoise(const Eigen::MatrixXd &noise)
{
  R_sensor_ = noise;
}

Eigen::MatrixXd Sensor::Noise() const
{
  return R_sensor_;
}

Lidar::Lidar() :
    Jacobian_(MatrixXd::Zero(2, 4))
{
  Jacobian_ << 1, 0, 0, 0,
               0, 1, 0, 0;
}

Eigen::VectorXd Lidar::MeasurementError(const Eigen::VectorXd& measurement, const Eigen::VectorXd& x_state) const
{
  return measurement - Jacobian_ * x_state;
}

Eigen::MatrixXd Lidar::CalculateJacobian(const VectorXd& x_state) const
{
  return Jacobian_;
}

Eigen::VectorXd Lidar::InitialValues(const VectorXd& measure) const
{
  VectorXd x_init_state(4);
  x_init_state << measure[0], measure[1], 0.0, 0.0;
  return x_init_state;
}

Radar::Radar()
{

}

Eigen::VectorXd Radar::MeasurementError(const Eigen::VectorXd& measurement, const Eigen::VectorXd& x_state) const
{
  const double px = x_state[0];
  const double py = x_state[1];
  const double vx = x_state[2];
  const double vy = x_state[3];
  Eigen::VectorXd polar = VectorXd::Zero(3);
  const double radius2 = px * px + py * py;
  //in exceptional situation update for part of the measurement is skipped
  const double radius = radius2 != 0 ? sqrt(radius2) : measurement[0];
  const double anlge = px != 0 ? atan2(py, px) : measurement[1];
  polar << radius, anlge, (vx * px + vy * py) / radius;

  VectorXd y = measurement - polar;

  while (y[1] > M_PI)
  {
    y[1] = y[1] - M_PI * 2;
  }
  while (y[1] < -M_PI)
  {
    y[1] = y[1] + M_PI * 2;
  }

  return y;
}

Eigen::MatrixXd Radar::CalculateJacobian(const VectorXd& x_state) const
{
  const double px = x_state(0);
  const double py = x_state(1);
  const double vx = x_state(2);
  const double vy = x_state(3);
  const double den2 = px * px + py * py;

  MatrixXd Hj = MatrixXd::Zero(3, 4);

  if (den2 > 1e-10)
  {
    const double den = sqrt(den2);
    const double den3 = den2 * den;
    Hj <<                       px / den,                       py / den,      0.0,     0.0,
                              -py / den2,                      px / den2,      0.0,     0.0,
        py * (vx * py - vy * px) / den3, px * (vy * px - vx * py) / den3, px / den, py / den;
  }

  // if px and py are == 0  return zero matrix as result of the derivation
  // the client code will handle the consequences
  return Hj;
}

Eigen::VectorXd Radar::InitialValues(const VectorXd& measure) const
{
  const double radius = measure[0];
  const double angle = measure[1];
  // NB: it is not possible to derive the Cart speed as angle rate is not known
  VectorXd x_init_state(4);
  x_init_state << radius * cos(angle), radius * sin(angle), 0.0, 0.0;
  return x_init_state;
}

