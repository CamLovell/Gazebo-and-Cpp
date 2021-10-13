#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <random>
#include <Eigen/Core>

#include "wamv.h"

void intiBoat(boatParams& params){
    double d = 2, l = 2;

    params.rLCb.resize(3);
    params.rLCb << -l,-d/2,0;
    params.rRCb.resize(3);
    params.rRCb << -l,d/2,0;
    
    params.sigmaU.resize(4);
    params.sigmaU << 200,0.5,200,0.5;
    params.sigmaX.resize(6);
    params.sigmaX << 0.5,0.5,0.1*M_PI/180,0.001,0.001,0.001;
    // params.sigmaX << 5,5,1*M_PI/180,0.1,0.1,0.1;
    // params.sigmaX.setZero();

}
// Vectorised Boat Dynamics to deal with particle input
void boatDynamics(const Eigen::MatrixXd& x, const Eigen::MatrixXd& u, const boatParams& param, Eigen::MatrixXd& dx){
    
    assert(x.rows() == 6);
    assert(u.rows() == 4);
    // assert(u.cols() == 1);

    // Extract magnitude of velocities
    Eigen::MatrixXd magNu = x.block(3,0,3,x.cols()).array().abs();

    // Precompute trig terms for motor angles
    Eigen::MatrixXd cLa,cRa,sLa,sRa,a,b;
    cLa = u.row(1).array().cos();
    cRa = u.row(3).array().cos();
    sLa = u.row(1).array().sin();
    sRa = u.row(3).array().sin();

    // Calculate dx
    a = u.row(0).array()*sLa.array()+u.row(2).array()*sRa.array()-((696*magNu.row(1).array()+8361*magNu.row(2).array()+884)*x.row(4).array()+(-167.61*x.row(3).array()+814*magNu.row(2).array()+307*magNu.row(1).array()-202)*x.row(5).array());
    b = u.row(0).array()*(-param.rLCb(1)*cLa.array()+param.rLCb(0)*sLa.array())+u.row(2).array()*(-param.rRCb(1)*cRa.array()+param.rRCb(0)*sRa.array())-((-475*x.row(4).array()+1736*x.row(5).array())*x.row(3).array()+(-1516*magNu.row(1).array()-425*magNu.row(2).array()+552)*x.row(4).array()+(-2884*magNu.row(2).array()-202*magNu.row(1).array()+1258)*x.row(5).array());

    dx.resize(6,x.cols());

    dx << x.row(3).array()*x.row(2).array().cos() - x.row(4).array()*x.row(2).array().sin(),x.row(4).array()*x.row(2).array().cos() + x.row(3).array()*x.row(2).array().sin(), x.row(5).array(), 0.001156885202281*(u.row(0).array()*cLa.array()+u.row(2).array()*cRa.array()-((85*magNu.row(0).array()+120)*x.row(3).array()+(642.61*x.row(4).array()-1736*x.row(5).array())*x.row(5).array())),0.000694851544311*a.array()+0.000240704132172*b.array(),-0.000039932482757*a.array()+0.000185712389164*b.array();

}

void processModel(const Eigen::MatrixXd& xt, const Eigen::MatrixXd& u, const boatParams& param,const double& dt, Eigen::MatrixXd& xtp1){
    // Steps for finner random noise in inputs
    int k = 50;
    double ds = dt/k;

    Eigen::MatrixXd ustep, dx, xTemp(xt.rows(),xt.cols());

    // Assign initial input to resused input variable
    ustep = u*1000;
    
    // Setup normal distributions usable with eigen matricies
    std::srand((unsigned int) time(nullptr));
    std::default_random_engine generator;
    std::normal_distribution<double> ndu0(u(0),param.sigmaU(0)),ndu1(u(1),param.sigmaU(1)),ndu2(u(2),param.sigmaU(2)),ndu3(u(3),param.sigmaU(3)); // Assumed starting position, in reality will be 0,0 and NED origin will be set to this lat and long
    // Distributions of inputs              
    auto u0dist = [&] (double) {return ndu0(generator);};
    auto u1dist = [&] (double) {return ndu1(generator);};
    auto u2Dist = [&] (double) {return ndu2(generator);};
    auto u3Dist = [&] (double) {return ndu3(generator);};
    // Incrementely check dynamics with fine input randomness
    for(int i = 0; i<k; i++){
        boatDynamics(xt, ustep, param, dx);
        xTemp = xt+dx*ds;
        ustep.row(0) = Eigen::VectorXd::NullaryExpr(u.cols(), u0dist);
        ustep.row(1) = Eigen::VectorXd::NullaryExpr(u.cols(), u1dist);
        ustep.row(2) = Eigen::VectorXd::NullaryExpr(u.cols(), u2Dist);
        ustep.row(3) = Eigen::VectorXd::NullaryExpr(u.cols(), u3Dist);
    }

    // Distributions of states
    std::normal_distribution<double> ndx0(xTemp(0),param.sigmaX(0)),ndx1(xTemp(1),param.sigmaX(1)),ndx2(xTemp(2),param.sigmaX(2)),ndx3(xTemp(3),param.sigmaX(3)),ndx4(xTemp(4),param.sigmaX(4)),ndx5(xTemp(5),param.sigmaX(5)); // Assumed starting position, in reality will be 0,0 and NED origin will be set to this lat and long
    auto x0dist = [&] (double) {return ndx0(generator);};
    auto x1dist = [&] (double) {return ndx1(generator);};
    auto x2Dist = [&] (double) {return ndx2(generator);};
    auto x3Dist = [&] (double) {return ndx3(generator);};
    auto x4Dist = [&] (double) {return ndx4(generator);};
    auto x5Dist = [&] (double) {return ndx5(generator);};

    // Ensure output is correct size
    xtp1.resize(xt.rows(),xt.cols());

    xtp1.row(0) = Eigen::VectorXd::NullaryExpr(u.cols(), x0dist);
    xtp1.row(1) = Eigen::VectorXd::NullaryExpr(u.cols(), x1dist);
    xtp1.row(2) = Eigen::VectorXd::NullaryExpr(u.cols(), x2Dist);
    xtp1.row(3) = Eigen::VectorXd::NullaryExpr(u.cols(), x3Dist);
    xtp1.row(3) = Eigen::VectorXd::NullaryExpr(u.cols(), x4Dist);
    xtp1.row(3) = Eigen::VectorXd::NullaryExpr(u.cols(), x5Dist);

}