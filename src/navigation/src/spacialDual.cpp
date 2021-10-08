#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <Eigen/Core>

#include "spacialDual.h"
#include "sensorData.h"
#include "helpers.h"
#include "Eig.hpp"


static Eigen::VectorXd rNOg(3),rNOe(3);
static Eigen::MatrixXd Rne; //Latitude and longitude of NED origin in vrx sydneyRegatta
static double a,b,d,f,gpsSigma;
static bool isInit;

//-------------------------------------------------------------------------------------------------------------------------------------------------------
// GPS Initialisation Function, change/add any constants here!!
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// void gpsInit(void){
//     // if(!isInit){
//         rNOg << -33.724223, 150.679736, 0.0; //Geodedic coordinates of NED origin
//         rotateLatLong(rNOg, Rne);
//         // rNOe << -4630032, 2600407, -3521046; //ECEF coordinates of NED origin
//         rNOe << -4630032.213502892,   2600406.528857937,  -3521045.936084332;
//         //Paramaters to describe earth's elipsoid
//         a = 6378137.0;
//         b = 6356752.3142;
//         d = 0.08181919; 
//         f = std::sqrt((a*a-b*b)/(b*b));
//         isInit = true;
//     // }    
// }
void gpsInit(void){
    // if(!isInit){
        isInit = true;

        rNOe << -4630032.213502892,   2600406.528857937,  -3521045.936084332;
        rNOg << -33.724223, 150.679736, 0.0; //Geodedic coordinates of NED origin
        rotateLatLong(rNOg, Rne);
        
        // rNOe << -4630032, 2600407, -3521046; //ECEF coordinates of NED origin
        
        //Paramaters to describe earth's elipsoid
        a = 6378137.0;
        b = 6356752.3142;
        d = 0.08181919; 
        f = std::sqrt((a*a-b*b)/(b*b));
        
        double sigmaNE = 0.08;
        double sigmaD = 0.1;
        Eigen::MatrixXd sig(3,1), sigGeo;
        sig << 0,0,0;
        
        NEDtoGeo(sig,sigGeo);

        gpsSigma = abs(sigGeo(0,0)-rNOg(0));
        // std::cout << gpsSigma << std::endl;
        
        
    // }    
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------
// Log Likelihoods // Combine these into one maybe? will save 1 funciton call per loop?
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void gpsLogLiklihood(const Eigen::VectorXd& y, const Eigen::MatrixXd& x, const int& M, Eigen::VectorXd& lw){
    Eigen::MatrixXd partCoords, partWeight;

    // Initialise outputs with zeros to avoid any issues with existing memory values
    lw.setZero(M);

    // Convert NED coordinates to lat and long as measured
    NEDtoGeo(x,partCoords);
    // double sig2 = gpsSigma*gpsSigma; // Include this as a pramater?

    // Non-log version
    // w = (1/sqrt(2*M_PI*gpsSigma*gpsSigma))*(0.5*((partCoords.array().square().colwise()-y.array()))/(gpsSigma*gpsSigma)).exp();
   
    // Calculate log likelihood 
    partWeight = log(1/sqrt(2*M_PI*gpsSigma*gpsSigma))+(0.5*((partCoords.array().square().colwise()-y.array()))/(gpsSigma*gpsSigma));

    // Hacky loop for poor LSE implementation, need to look at matrix input LSE
    for(int i = 0; i<M; i++){
        Eigen::VectorXd temp;
        temp = partWeight.col(i);

        lw(i) = logSumExponential(temp);
    }
}
void imuLogLiklihood(const Eigen::VectorXd& y, const Eigen::MatrixXd& x, const int& M, Eigen::VectorXd& lw){

}

//-------------------------------------------------------------------------------------------------------------------------------------------------------
// Coordinate conversions between NED and Geodetic
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void NEDtoGeo(const Eigen::MatrixXd& rBNn, Eigen::MatrixXd& rBOg){
    // Ensure input dimensions are correct
    assert(isInit);
    assert(rBNn.rows() == 3);     
    
    // Declare Required Variables
    Eigen::VectorXd p(rBNn.cols()), theta(rBNn.cols()), pN(rBNn.cols()), sintheta3(rBNn.cols()), costheta3(rBNn.cols()), temp0, temp1, temp2;
    Eigen::MatrixXd rBOe(3,rBNn.cols());  
    
    // Convert to ECEF coordinates
    rBOe = Rne*rBNn + rNOe;
    
    // Calculate ECEF paramaters
    p = (rBOe.row(0).array().square()+rBOe.row(1).array().square()).sqrt().matrix();
    pN = (rNOe.row(0).array().square()+rNOe.row(1).array().square()).sqrt().matrix();
    Eig::atan2(a*rBOe.row(2),b*p,theta); // custom eigen elementwise atan2
    
    // Caluclate geodedic coordinates
    rBOg.resize(3,rBNn.cols());
    sintheta3 = (theta.array().sin().square()*theta.array().sin()).matrix();
    costheta3 = (theta.array().cos().square()*theta.array().cos()).matrix();

    Eig::atan2(rBOe.row(2).array()+f*f*b*sintheta3.array(),p.array()-d*d*a*costheta3.array(),temp0);
    Eig::atan2(rBOe.row(1),p+rBOe.row(0),temp1);

    //!!Altitude measurement is broken however this is not used in the boat model so not a big deal for now!!
    //// rBOg.row(2) = ((p.array()/temp0.array().cos()))+(pN.array()/((rNOe.row(0).array()*(M_PI/180)).cos()))).matrix();
// p.array()/(temp0.array().cos()+cos(rNOg(0))) 
//(pN.array()/cos(rNOg(0)*M_PI/180))-(p.array()/temp0.array().cos())
// rBOe.norm()-rNOe.norm()

    // Alt is dodgy as fuck!!!
    rBOg << temp0*180/M_PI, 2*temp1*180/M_PI,(rBOe.norm()-rNOe.norm()+4.60823*rBNn(2));
    // <latitude_deg>-33.724223</latitude_deg>
    // <longitude_deg>150.679736</longitude_deg>
    // <elevation>0.0</elevation>
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------
// Rotation representation conversions between Roll Pitch Yaw and Quaternions
//-------------------------------------------------------------------------------------------------------------------------------------------------------

void RPYtoQuaternion(const Eigen::MatrixXd& RPY, Eigen::MatrixXd& quaternion){
    // Declare variables for use
    Eigen::VectorXd cphi(RPY.cols()), ctheta(RPY.cols()), cpsi(RPY.cols()), sphi(RPY.cols()), stheta(RPY.cols()), spsi(RPY.cols());
    
    // Pre-compute trig functions to avoid multiple computations
    sphi = (0.5*RPY.row(0).array()).sin().matrix();
    cphi = (0.5*RPY.row(0).array()).cos().matrix();
    stheta = (0.5*RPY.row(1).array()).sin().matrix();
    ctheta = (0.5*RPY.row(1).array()).cos().matrix();
    spsi = (0.5*RPY.row(2).array()).sin().matrix();
    cpsi = (0.5*RPY.row(2).array()).cos().matrix();

    // Convert angles to Quaternion (in form [w,x,y,z])
    quaternion.resize(4,RPY.cols());
    quaternion.row(0) = cphi.array()*ctheta.array()*cpsi.array() + sphi.array()*stheta.array()*spsi.array();
    quaternion.row(1) = sphi.array()*ctheta.array()*cpsi.array() - cphi.array()*stheta.array()*spsi.array();
    quaternion.row(2) = cphi.array()*stheta.array()*cpsi.array() + sphi.array()*ctheta.array()*spsi.array();
    quaternion.row(3) = cphi.array()*ctheta.array()*spsi.array() - sphi.array()*stheta.array()*cpsi.array();
}

void quaterniontoRPY(const Eigen::MatrixXd& quaternion, Eigen::MatrixXd& RPY){
    // Declare variables for use
    Eigen::VectorXd qw2(quaternion.cols()), qx2(quaternion.cols()), qy2(quaternion.cols()), qz2(quaternion.cols()), temp0, temp2;
    
    // Extract quaternion elements and square them for later use
    // q in form [w,x,y,z]
    qw2 = quaternion.row(0).array().square();
    qx2 = quaternion.row(1).array().square();
    qy2 = quaternion.row(2).array().square();
    qz2 = quaternion.row(3).array().square();    

    // Eig_funcs Eig;
    RPY.resize(3,quaternion.cols());

    // Calculate Euler angles in a ZYX rotation form
    // Implementation using custom elementwise Eigen atan2
    Eig::atan2(2*(quaternion.row(0).array()*quaternion.row(1).array() + quaternion.row(2).array()*quaternion.row(3).array()),1-2*(qx2.array()+qy2.array()),temp0);
    Eig::atan2(2*(quaternion.row(1).array()*quaternion.row(2).array() + quaternion.row(0).array()*quaternion.row(3).array()),1-2*(qy2.array()+qz2.array()),temp2);
    RPY << temp0,(2*(quaternion.row(0).array()*quaternion.row(2).array() - quaternion.row(1).array()*quaternion.row(3).array())).asin(),temp2;
    
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------
// Rotation of coordinates given latitude and longitude
//-------------------------------------------------------------------------------------------------------------------------------------------------------

// Create rotation matrix for a given lonitude and latitude (input in degrees)
void  rotateLatLong(const Eigen::VectorXd& geoCoordDeg, Eigen::MatrixXd& R){
    // Initialise rotation components
    Eigen::MatrixXd RZ(geoCoordDeg.rows(),geoCoordDeg.rows()), RY(geoCoordDeg.rows(),geoCoordDeg.rows()),adjCoord;
    adjCoord = geoCoordDeg;
    adjCoord(0) = -adjCoord(0)-90;
    //Deg to rad conversion
    Eigen::VectorXd geoCoordRad = (M_PI/180.0)*adjCoord; 
    
    // z and y axis roations (latitude and longitude respectively)
    RZ << cos(geoCoordRad(1)),-sin(geoCoordRad(1)),0,sin(geoCoordRad(1)),cos(geoCoordRad(1)),0,0,0,1;
    RY << cos(geoCoordRad(0)),0,sin(geoCoordRad(0)),0,1,0,-sin(geoCoordRad(0)),0,cos(geoCoordRad(0));
    // Compute final rotation matrix
    Rne.resize(geoCoordDeg.rows(),geoCoordDeg.rows());
    R << RZ*RY;
    // std::cout << R << std::endl;

}