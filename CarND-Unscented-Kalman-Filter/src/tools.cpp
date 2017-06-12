#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

void NormalizeAngle(double& phi)
{
  phi = atan2(sin(phi), cos(phi));
}
Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.size() == ground_truth.size() && estimations.size() != 0)
  {
    //accumulate squared residuals
    for (size_t i = 0; i < estimations.size(); ++i)
    {
      VectorXd residual = estimations[i] - ground_truth[i];
      VectorXd residual2 = residual.array() * residual.array();
      rmse = rmse + residual2;
    }
    rmse /= estimations.size();
    rmse = rmse.array().sqrt();
  }

  return rmse;
}
