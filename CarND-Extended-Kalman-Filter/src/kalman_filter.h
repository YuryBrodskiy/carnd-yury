#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

/**
 *  Template class that encapsulates the sensor behavior
 *  Calculation of innovations,
 *  Jacobian of the measurement function
 *  Initial value guess
 *  Measurement noise covariance
 */
class Sensor
{

public:
  virtual ~Sensor()
  {
  }

  virtual Eigen::VectorXd MeasurementError(const Eigen::VectorXd& measure, const Eigen::VectorXd& x_state) const = 0;
  virtual Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state) const = 0;
  virtual Eigen::VectorXd InitialValues(const Eigen::VectorXd& measure) const = 0;
  void SetProcessNoise(const Eigen::MatrixXd &noise);
  Eigen::MatrixXd Noise() const;
protected:
  Eigen::MatrixXd R_sensor_;
};

/**
 *  Measurements specific for radar
 *  Non-linear MeasurementError and calculation of corresponding Jacobian
 */
class Radar: public Sensor
{

public:
  Radar();
  Eigen::VectorXd MeasurementError(const Eigen::VectorXd& measure, const Eigen::VectorXd& x_state) const;
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state) const;
  Eigen::VectorXd InitialValues(const Eigen::VectorXd& measure) const;

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
  Eigen::VectorXd MeasurementError(const Eigen::VectorXd& measure, const Eigen::VectorXd& x_state) const;
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state) const;
  Eigen::VectorXd InitialValues(const Eigen::VectorXd& measure) const;
};

class KalmanFilter
{
public:

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();
  void SetProcessNoise(const Eigen::MatrixXd &noise);

  /**
   * Init Initializes Kalman filter
   * @param z The first measurement
   */
  void Init(const Eigen::VectorXd &z, const Sensor& sensor);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void PredictMotion(const double& dt);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */

  void Update(const Eigen::VectorXd &measurement, const Sensor& sensor);

public:
  static const double kTimeToUnknown;
  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;
private:
  Eigen::MatrixXd I_;
  Eigen::MatrixXd Qv_;

};

#endif /* KALMAN_FILTER_H_ */
