#include<cmath>
#include<Eigen/Core>
#include <iostream>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <sstream> 
#include <string> 

#include"Eig.hpp"

// Vectorized atan2 -- Kinda
void Eig::atan2(const Eigen::VectorXd& numerator,const Eigen::VectorXd& denominator,Eigen::VectorXd& result){
    int length = numerator.size();
    assert(denominator.size() == length); // Ensure numerator and denominatior are the same size
    result.resize(length);
    for(int i=0; i<length; i++){
        result(i) = std::atan2(numerator(i),denominator(i));
    }
    // std::cout << result << std::endl;
}

// Read CSV file into eigen matrix, thankyou stack overflow,
Eigen::MatrixXd readCSV(std::string file, int rows, int cols) {

    std::ifstream in(file);
    
    std::string line;

    int row = 0;
    int col = 0;
    if(!in.is_open()){
        std::cout << "no file" << std::endl;
    }
    std::vector<double> values;
    while (std::getline(in, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
    }

    Eigen::Map<Eigen::MatrixXd,0,Eigen::Stride<1,400>> res(values.data(),rows,cols);
    in.close();
    return res;
}