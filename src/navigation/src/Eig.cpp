#include<cmath>
#include<Eigen/Core>
#include <iostream>

#include"Eig.hpp"

void Eig::atan2(const Eigen::VectorXd& numerator,const Eigen::VectorXd& denominator,Eigen::VectorXd& result){
    int length = numerator.size();
    assert(denominator.size() == length); // Ensure numerator and denominatior are the same size
    result.resize(length);
    for(int i=0; i<length; i++){
        result(i) = std::atan2(numerator(i),denominator(i));
    }
    // std::cout << result << std::endl;
}