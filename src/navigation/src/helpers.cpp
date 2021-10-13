#include<Eigen/Core>
#include<iostream>

#include "helpers.h"

void weightedResample(const Eigen::VectorXd& logWeights, const int& N, Eigen::Matrix<int,Eigen::Dynamic,1>& idx){
    
    std::srand((unsigned int) time(nullptr));
    // Declare nessecary variables
    Eigen::VectorXd sample(N+1),tempLin,tempRand(N), weights;
    int i=0,j=0;
    weights = logWeights.array().exp();
    double sum = weights(0);
    
    // Set output to correct size and fill with "-1" for error checking of output
    idx.resize(N);
    idx.fill(-1);
    // assert(logWeights.array().sum()==1);
    // Create a vector of distrubuted sudo-random numbers to cover the 0 to 1 probability space at intervals of 1/N
    sample << tempLin.setLinSpaced(N,0,(1.0-1.0/N)).array()+(0.5/N)*(tempRand.setRandom().array()+1),1; //.Random() distributes over [-1,1], some manupulation to distribute over [0,1]    

    while(i < N){
        // std::cout << i << " " << j << std::endl;
        // std::cout << sample(i) << " " << sum << std::endl;
        if(sample(i) < sum || abs(sample(i)-sum)< 1e-15){
            idx(i) = j;
            i++;
        }
        else{
            j++;
            // Add next weight to sum for next cumulative sum element
            // Hack as Eigen does not have cumulative sum functionality
            sum += weights(j);
            

        }
        
    }
        // std::cout << idx.transpose() << std::endl;
}

double logSumExponential(const Eigen::VectorXd& logWeights){ // Rework to take Matrix of weight vectors maybe?
    // Find maximum weight
    double maxW = logWeights.maxCoeff(); 
    double out = (maxW + log((logWeights.array()-maxW).exp().sum()));
    
    
    return out;
}