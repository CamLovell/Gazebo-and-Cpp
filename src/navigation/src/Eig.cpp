#include<cmath>
#include<Eigen/Core>
#include <iostream>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <sstream> 
#include <string> 

#include"Eig.hpp"

// Elementwise atan2 for Eigen Vectors of angles
void Eig::atan2(const Eigen::VectorXd& numerator,const Eigen::VectorXd& denominator,Eigen::VectorXd& result){

    // Assign vector length
    int length = numerator.size();

    // Ensure numerator and denominatior are the same size
    assert(denominator.size() == length); 

    // Calculate atan2 for each element
    result.resize(length);
    for(int i=0; i<length; i++){
        result(i) = std::atan2(numerator(i),denominator(i));
    }
}

// Read CSV file into Eigen matrix, thankyou stack overflow,
Eigen::MatrixXd readCSV(std::string file, int rows, int cols) {
    // Open file
    std::ifstream in(file);
    if(!in.is_open()){
        std::cout << "no file" << std::endl;
    }

    std::string line;

    int row = 0;
    int col = 0;

    // Loop until no more lines of file available
    std::vector<double> values;
    while (std::getline(in, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        // Loop through elements sepatated by commas (,)
        while (std::getline(lineStream, cell, ',')) {
            // Assign value to std::vector
            values.push_back(std::stod(cell));
        }
    }

    // Map std::vector to Eigen matrix
    Eigen::Map<Eigen::MatrixXd,0,Eigen::Stride<1,400>> res(values.data(),rows,cols);

    // Close File
    in.close();

    // Return matrix
    return res;
}