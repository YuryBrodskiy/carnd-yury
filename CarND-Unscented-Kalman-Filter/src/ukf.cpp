#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"
#include <math.h>
#include <limits>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


bool validUpdate( const VectorXd& z_diff, const MatrixXd& K ) 
{
	const double max_val = std::numeric_limits<double>::max();
	//if any element is nan or inf then norm will be nan or inf
	//thus result will be false
    bool result = K.norm() < max_val && z_diff.norm()< max_val;

    return result;
}
bool isValidState(const VectorXd& x, const MatrixXd& P)
{
	//if values are Nan or Inf estimation on the previous step failed
	//if P norm is huge, reinitalizing the filter is more efficent then waiting for converge
	// test case where this check is required is quick switching between Data Set 1 and Data set 2, without restarting the filter
	return validUpdate(x, P) && P.norm()<1e30;
}

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  is_initialized_ = false; 
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.3;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  n_x_   = 5;
  n_aug_ = 7;
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //add measurement noise covariance matrix
  MatrixXd lidarR = MatrixXd(2, 2);
  lidarR <<    std_laspx_*std_laspx_, 0,
            0, std_laspy_*std_laspy_;
  lidar_.SetProcessNoise(lidarR);

  //add measurement noise covariance matrix
  MatrixXd radarR = MatrixXd(3, 3);
  radarR <<    std_radr_*std_radr_, 0, 0,
         0, std_radphi_*std_radphi_, 0,
        0, 0,std_radrd_*std_radrd_;
  radar_.SetProcessNoise(radarR);

  lambda_ = 3 - n_aug_;
  weights_ = VectorXd::Zero(2 * n_aug_ + 1);
  weights_.fill(0.5 / (n_aug_ + lambda_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  // the differences in updates are handled by dispatch to specific sensor class
 
  unscent::Sensor *sensor = NULL;
  const double sTimeToUnknow = 1.0;
  //Re initialize in case NaN or Inf state
  if (!is_initialized_ || !isValidState(x_, P_))
  {
      if (meas_package.sensor_type_ == MeasurementPackage::RADAR )
      {
          sensor = &radar_;
      }
      if (meas_package.sensor_type_ == MeasurementPackage::LASER )
      {
          sensor = &lidar_;
      }
    x_ = sensor->InitialValues(meas_package.raw_measurements_);
    P_ = Eigen::MatrixXd::Zero(5, 5);
    // Predict movement for 1 sec
    // split in steps to capture nonlinear effects in motion model
    double time = 0;
    const double timeStep = 0.2;

    while(time<sTimeToUnknow)
    {
        Prediction(timeStep);
        time+=timeStep;
    }

    // Now normal update will bring the covariance matrix to sensor almost to sensor values
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
  }
  else
  {
    double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;  //dt - expressed in seconds
	if(dt>sTimeToUnknow)
	{
		dt=sTimeToUnknow;
	}
    Prediction(dt);
  }


  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
  {
      sensor = &radar_;
  }
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
  {
      sensor = &lidar_;
  }
  if ( sensor!= NULL)
  {
      Update(meas_package.raw_measurements_, *sensor);
  }
 
  time_us_ = meas_package.timestamp_;
}

MatrixXd unscentTransform(const VectorXd& x, const MatrixXd& P)
{
    //Generate sigma points
    const int n = x.rows();
    const double lambda = 3 - n;
    const double sqrtL = sqrt(lambda+n);
    const MatrixXd A = P.llt().matrixL();
    //create sigma point matrix
    MatrixXd Xsig = MatrixXd::Zero(n, 2 * n + 1);
    //set first column of sigma point matrix
    Xsig.col(0)  = x;
    //set remaining sigma points
    for (int i = 0; i < n; i++)
    {
        Xsig.col(i+1)   = x + sqrtL * A.col(i);
        Xsig.col(i+1+n) = x - sqrtL * A.col(i);
    }
    return Xsig;
}


MatrixXd motionModel(const MatrixXd& x, double dt)
{
  MatrixXd result = MatrixXd::Zero(5, x.cols());
  for(int i = 0; i < x.cols();i++)
  {
    const double px   =  x.col(i)(0);
    const double py   =  x.col(i)(1);
    const double v    =  x.col(i)(2);
    const double fi   =  x.col(i)(3);
    const double dfi  =  x.col(i)(4);
    const double va   =  x.col(i)(5);
    const double ddfi =  x.col(i)(6);
    if (fabs(dfi) < 0.0001)
    {
        //            value   + derivative*time + noise
        result.col(i)(0) = px  + v*cos(fi)*dt    + 0.5*dt*dt*cos(fi)*va;
        result.col(i)(1) = py  + v*sin(fi)*dt    + 0.5*dt*dt*sin(fi)*va;
    }
    else
    {
        result.col(i)(0) = px  +  v/dfi*( sin(fi + dfi*dt) -sin(fi)) + 0.5*dt*dt*cos(fi)*va;
        result.col(i)(1) = py  +  v/dfi*(-cos(fi + dfi*dt) +cos(fi)) + 0.5*dt*dt*sin(fi)*va;
    }
    result.col(i)(2) = v   +            0    + dt*va;
    result.col(i)(3) = fi  +       dfi*dt    + 0.5*dt*dt*ddfi;
    result.col(i)(4) = dfi +            0    + dt*ddfi;
   }
   return result;
}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double dt) {
  /**
  TODO:
  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  //create augmented mean vector
  VectorXd x_aug = VectorXd::Zero(n_aug_);
  x_aug.head(n_x_) = x_;
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug.topLeftCorner( n_x_ ,n_x_ )= P_;
  P_aug( n_x_ ,n_x_) = std_a_*std_a_;
  P_aug( n_x_+1 ,n_x_+1) = std_yawdd_*std_yawdd_;


  //Generate sigma points
  const MatrixXd Xsig_aug = unscentTransform(x_aug, P_aug);

  Xsig_pred_ = motionModel(Xsig_aug, dt);
  x_ = Xsig_pred_ * weights_;

  MatrixXd X_diff = Xsig_pred_;

  X_diff.colwise() -= x_;
  //Note: I don't normalize yaw on purpose. 
  //It is not important to do so as all functions that use this angle already do that. 
  //It actually makes possible to detect if the bicycle has made more then 1 cycle 

  P_ = X_diff * weights_.asDiagonal() * X_diff.transpose();


}




/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::Update(VectorXd z, unscent::Sensor& sensor) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */


   const MatrixXd Y_k_k1 = sensor.PredictMeasurement(Xsig_pred_);
   const VectorXd y_pred = Y_k_k1* weights_;
   //create matrix for sigma points in measurement space
   const MatrixXd Y_diff = sensor.MeasurementError(Y_k_k1, y_pred);
   const MatrixXd Pyy = Y_diff*weights_.asDiagonal()*Y_diff.transpose();
   const MatrixXd S = sensor.AddNoise(Pyy);


   MatrixXd X_diff = Xsig_pred_;
   X_diff.colwise() -= x_;

   const MatrixXd Pxy = X_diff * weights_.asDiagonal() * Y_diff.transpose();
   const MatrixXd iS = S.inverse();
   const MatrixXd K = Pxy * iS;
   const VectorXd z_diff = sensor.MeasurementError(z, y_pred);

   //ignore update incase Nan or Inf inovations
   if (validUpdate(z_diff, K))
   {
       x_ = x_ + K * z_diff;
       P_ = P_ - K*S*K.transpose();
       sensor.nis_ = z_diff.transpose() * iS * z_diff;
       nis_ = sensor.nis_;
       inBound_ = sensor.inBound05(nis_);
   }


}


VectorXd UKF::Estimate() const
{
  const double p_x = x_(0);
  const double p_y = x_(1);
  const double v  = x_(2);
  const double yaw = x_(3);

  const double v1 = cos(yaw)*v;
  const double v2 = sin(yaw)*v;

  return (VectorXd(4) <<p_x,p_y,v1,v2).finished();

}

void unscent::Sensor::SetProcessNoise(const Eigen::MatrixXd &noise)
{
  R_sensor_ = noise;
}

Eigen::MatrixXd unscent::Sensor::AddNoise(const Eigen::MatrixXd& Cov) const
{
  return Cov + R_sensor_;
}

bool unscent::Sensor::inBound05( double nis )
{
    return lower_< nis && nis< upper_;
}


unscent::Lidar::Lidar() 
    : Jacobian_(MatrixXd::Zero(2, 5))
{
  Jacobian_ << 1, 0, 0, 0, 0,
               0, 1, 0, 0, 0;
  lower_ = 0.10;
  upper_ = 5.99;
}
Eigen::MatrixXd unscent::Lidar::PredictMeasurement(const Eigen::MatrixXd& x_state) const
{
  return Jacobian_ * x_state;
}
Eigen::MatrixXd unscent::Lidar::MeasurementError(const Eigen::MatrixXd& left, const Eigen::VectorXd& right) const
{
  return left.colwise() - right;
}
Eigen::VectorXd unscent::Lidar::InitialValues(const VectorXd& measure) const
{
  return (VectorXd(5) << measure[0], measure[1], 0.0, 0.0, 0.0).finished();
}

unscent::Radar::Radar()
{
    lower_ = 0.35;
    upper_ = 7.82;
}

Eigen::MatrixXd unscent::Radar::PredictMeasurement( const Eigen::MatrixXd& x_state ) const
{
    Eigen::MatrixXd polar = MatrixXd::Zero(3,x_state.cols());
    for (int  i = 0; i< x_state.cols(); i++)
    {
        const double px = x_state.col(i)[0];
        const double py = x_state.col(i)[1];
        const double v = x_state.col(i)[2];
        const double yaw = x_state.col(i)[3];
        const double vx = cos(yaw)*v;
        const double vy = sin(yaw)*v;
        const double radius2 = px * px + py * py;
        //in exceptional situation update for part of the measurement is skipped
        const double radius = sqrt(radius2) ;
        const double anlge =  atan2(py, px) ;
        polar.col(i) << radius, anlge, (vx * px + vy * py) / radius;
    }
	//NB: no zero devision treatment main algorithm handles Nan and Inf inovations
    return polar;
}

Eigen::MatrixXd unscent::Radar::MeasurementError( const Eigen::MatrixXd& left, const Eigen::VectorXd& right ) const
{
    MatrixXd y = left.colwise() - right;
    for (int i = 0; i < y.cols(); i++)
    {
        NormalizeAngle(y.col(i)(1));
    }
    return y;
}

Eigen::VectorXd unscent::Radar::InitialValues( const VectorXd& measure ) const
{
    const double radius = measure[0];
    const double angle = measure[1];
    // NB: it is not possible to derive the Cart speed as angle rate is not known
    VectorXd x_init_state(5);
    x_init_state << radius * cos(angle), radius * sin(angle), 0.0, 0.0,0.0;
    return x_init_state;
}
