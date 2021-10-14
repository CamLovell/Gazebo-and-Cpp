#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <random>
#include <Eigen/Core>

#include "particleFilter.h"

void particleInit(const int& M, Eigen::VectorXd& logWeights, Eigen::MatrixXd& particleStates){
    // Initialise srand seed
    std::srand((unsigned int) time(nullptr));

    // Initialise all particles to have even weights
    Eigen::VectorXd relW(M),tempRand(M);
    relW.fill(1.0);

    // Normalise particle weigths
    logWeights = (relW.array()/relW.array().sum()).log();

    // Create Normal distribution
    double sigma = 2, sigmaPsi = 0.1/3*M_PI/180; // Inital position uncertainty in m
    std::default_random_engine generator;
    std::normal_distribution<double> ndE(-162.178285,sigma),ndN(531.564687,sigma),ndpsi(-2.15,sigmaPsi); // Assumed starting position, in reality will be 0,0 and NED origin will be set to this lat and long
    // std::normal_distribution<double> ndN(0,sigma),ndE(0,sigma),ndpsi(-2.15,sigmaPsi); // Assumed starting position, in reality will be 0,0 and NED origin will be set to this lat and long
    
    // Weird objects used in "NullaryExpr" to generate outputs
    auto Ndist = [&] (double) {return ndN(generator);};
    auto Edist = [&] (double) {return ndE(generator);};
    auto psiDist = [&] (double) {return ndpsi(generator);};

    // Initialise all states of all particles to 0
    particleStates.resize(6,M);
    particleStates.setZero();

    // Set N, E and psi states acordingly
    particleStates.row(0) = Eigen::VectorXd::NullaryExpr(M, Ndist);
    particleStates.row(1) = Eigen::VectorXd::NullaryExpr(M, Edist);
    particleStates.row(2) = Eigen::VectorXd::NullaryExpr(M, psiDist);
}