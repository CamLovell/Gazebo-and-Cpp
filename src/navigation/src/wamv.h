#ifndef WAMV_H
#define WAMV_H

#include<Eigen/Core>

struct boatParams{
    Eigen::VectorXd rLCb, rRCb, sigmaU, sigmaX;
};

void intiBoat(boatParams& params);
void boatDynamics(const Eigen::MatrixXd& x, const Eigen::MatrixXd& u, const boatParams& param, Eigen::MatrixXd& dx);
void processModel(const Eigen::MatrixXd& xt, const Eigen::MatrixXd& u, const boatParams& param,const double& dt, Eigen::MatrixXd& xtp1);

#endif