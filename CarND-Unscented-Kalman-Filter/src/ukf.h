#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace unscent{
class Sensor
{

public:
   Sensor():nis_(0){}
  virtual ~Sensor()
  {
  }
  virtual Eigen::VectorXd InitialValues(const VectorXd& measure) const = 0;
  virtual Eigen::MatrixXd MeasurementError(const Eigen::MatrixXd& left, const Eigen::VectorXd& right) const = 0;
  virtual Eigen::MatrixXd PredictMeasurement(const Eigen::MatrixXd& x_state) const = 0;
  virtual Eigen::MatrixXd AddNoise(const Eigen::MatrixXd& Cov) const;
  void SetProcessNoise(const Eigen::MatrixXd &noise);
  bool inBound05(double nis_);
  double nis_;
protected:
  Eigen::MatrixXd R_sensor_;
  double upper_;
  double lower_;
};

/**
 * Measurements specific for laser
 * Linear measurement function
 */
class Lidar: public Sensor
{
private:
  Eigen::MatrixXd Jacobian_;
public:

  Lidar();
  Eigen::MatrixXd PredictMeasurement(const Eigen::MatrixXd& x_state) const;
  Eigen::MatrixXd MeasurementError(const Eigen::MatrixXd& left, const Eigen::VectorXd& right) const;
  Eigen::VectorXd InitialValues(const VectorXd& measure) const;
};

/**
 * Measurements specific for laser
 * Linear measurement function
 */
class Radar: public Sensor
{
public:
  Radar();
  Eigen::MatrixXd PredictMeasurement(const Eigen::MatrixXd& x_state) const;
  Eigen::MatrixXd MeasurementError(const Eigen::MatrixXd& left, const Eigen::VectorXd& right) const;
  Eigen::VectorXd InitialValues(const VectorXd& measure) const;
};
}

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;
  double nis_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;
  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  unscent::Lidar lidar_;
  unscent::Radar radar_;
  int inBound_;
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a sensor measurement
   * @param meas_package The measurement at k+1
   */
  void Update(VectorXd z, unscent::Sensor& sensor);

  /**
   * Return an estimate method for comparison with ground truth
   */
  Eigen::VectorXd Estimate() const;

};

#endif /* UKF_H */
