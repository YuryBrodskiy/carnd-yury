#pragma once

#include <vector>
#include "Eigen-3.3/Eigen/Core"


class EKF {
 public:
  EKF();

  virtual ~EKF(){};

  Eigen::VectorXd update(const Eigen::VectorXd& measure, const Eigen::VectorXd& control);

private:
    Eigen::MatrixXd model_H() const;
    Eigen::VectorXd model_diff(const Eigen::VectorXd& measure, const Eigen::VectorXd& x ) const;
    Eigen::MatrixXd model_J(const Eigen::VectorXd& x, const Eigen::VectorXd& control ) const;
    Eigen::MatrixXd model_T(const  Eigen::VectorXd& x, const Eigen::VectorXd& control ) const;
    Eigen::VectorXd model_step(const  Eigen::VectorXd& x, const Eigen::VectorXd& control ) const;
private:
    double prev_time_; //<! wall time to check if init is required
    double nis_;
    Eigen::VectorXd x_; //<! car state
    Eigen::MatrixXd P_; //<! covariance matrix
    Eigen::MatrixXd T_; //<! Process noise
    Eigen::MatrixXd R_; //<! Measurement noise
    Eigen::MatrixXd I_; //<! Identity
};

