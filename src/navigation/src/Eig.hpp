#ifndef EIG_H
#define EIG_H

#include<Eigen/Core>

// Custom namespace for functions Eigen forgot to add
namespace Eig {
    void atan2(const Eigen::VectorXd& numerator,const Eigen::VectorXd& denominator,Eigen::VectorXd& result);
}

// static Eig_funcs Eig;

#endif