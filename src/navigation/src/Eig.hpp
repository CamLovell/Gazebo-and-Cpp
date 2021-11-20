#ifndef EIG_H
#define EIG_H

#include<Eigen/Core>

// Custom namespace for functions Eigen forgot to add
namespace Eig {
    void atan2(const Eigen::VectorXd& numerator,const Eigen::VectorXd& denominator,Eigen::VectorXd& result);
}
Eigen::MatrixXd readCSV(std::string file, int rows, int cols);

#endif