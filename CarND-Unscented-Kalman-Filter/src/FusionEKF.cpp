#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF()
{
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  MatrixXd R_laser(2, 2);
  R_laser << 0.0225, 0.0,
              0.0, 0.0225;

  //measurement covariance matrix - laser

  MatrixXd R_radar(3, 3);
  //measurement covariance matrix - radar
  R_radar << 0.09, 0.0, 0.0,
              0.0, 0.0009, 0.0,
              0.0, 0.0, 0.09;
  // motion noise
  MatrixXd R_acc(2, 2);
  const double acc_expectation = 9.0;
  const double acc_variance = acc_expectation * acc_expectation;
  R_acc << acc_variance, 0.0,
            0.0, acc_variance;
  //futher initialization is done in sensor/ kalman filter classes
  ekf_.SetProcessNoise(R_acc);
  radar_.SetProcessNoise(R_radar);
  lidar_.SetProcessNoise(R_laser);

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF()
{
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{


  if (!is_initialized_)
  {
    // the differences in initialization are handled by dispatch to specific sensor class
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      ekf_.Init(measurement_pack.raw_measurements_, radar_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      ekf_.Init(measurement_pack.raw_measurements_, lidar_);
    }
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;

  }
  else
  {
    //compute the time elapsed between the current and previous measurements
    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;
    ekf_.PredictMotion(dt);
    // the differences in updates are handled by dispatch to specific sensor class
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      // Radar updates
      ekf_.Update(measurement_pack.raw_measurements_, radar_);
    }
    else
    {
      // Laser updates
      ekf_.Update(measurement_pack.raw_measurements_, lidar_);
    }
  }

}
