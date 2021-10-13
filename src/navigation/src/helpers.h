#ifndef HELPERS_H
#define HELPERS_H

#include<Eigen/Core>

void weightedResample(const Eigen::VectorXd& logWeights, const int& N, Eigen::Matrix<int,Eigen::Dynamic,1>& idx);
double logSumExponential(const Eigen::VectorXd& logWeights);
// void unwrap(Eigen::MatrixXd& angles);

#endif