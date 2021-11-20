#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "navigation/state.h"
#include "std_msgs/Float32.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "gazebo_msgs/ModelStates.h"
#include "spacialDual.h"
#include "particleFilter.h"
#include "helpers.h"
#include "wamv.h"

static Eigen::VectorXd gpsMeas(3), imuMeas(4), u(4);

void gpsCallBack(const sensor_msgs::NavSatFix::ConstPtr& fix){
    // Extract and assign gps measurement
    gpsMeas << fix->latitude, fix->longitude, fix->altitude;
}
void imuCallBack(const sensor_msgs::Imu::ConstPtr& imuMsg){
    // Extract and assign imu measurement
    imuMeas << imuMsg->orientation.w, imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z;

}

// Extraction functions for getting inputs
void lThrustCallback(const std_msgs::Float32::ConstPtr& thrstMsg){
    u(0) = thrstMsg->data;
}
void lAngleCallback(const std_msgs::Float32::ConstPtr& angleMsg){
    u(1) = angleMsg->data;
}
void rThrustCallback(const std_msgs::Float32::ConstPtr& thrstMsg){
    u(2) = thrstMsg->data;
}
void rAngleCallback(const std_msgs::Float32::ConstPtr& angleMsg){
    u(3) = angleMsg->data;
}

int main(int argc, char **argv){
    // Initialise ros node
    ros::init(argc, argv, "navigation");

    // Assign required variables
    Eigen::VectorXd x0(6),gpslw, imulw, lw, lseW;
    Eigen::MatrixXd xp, xt, uMat;
    Eigen::Matrix<int,Eigen::Dynamic,1> idxWeighted;
    boatParams param;
    int M = 2000;
    double dt = 0.25;
    u.setZero();
    
    // Initialise necessary components
    intiBoat(param);
    particleInit(M,lw,xp);
    gpsInit();

    // Create publisher and subscriber handlers
    ros::NodeHandle handler;
    ros::Subscriber gpsSub = handler.subscribe<sensor_msgs::NavSatFix>("/wamv/sensors/gps/gps/fix", 1, gpsCallBack);
    ros::Subscriber imuSub = handler.subscribe<sensor_msgs::Imu>("/wamv/sensors/imu/imu/data", 1, imuCallBack);

    ros::Subscriber lThrustSub = handler.subscribe<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 1, lThrustCallback);
    ros::Subscriber lAngleSub = handler.subscribe<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 1, lAngleCallback);
    ros::Subscriber rThrustSub = handler.subscribe<std_msgs::Float32>("/wamv/thrusters/left_thrust_angle", 1, rThrustCallback);
    ros::Subscriber rAngleSub = handler.subscribe<std_msgs::Float32>("/wamv/thrusters/right_thrust_angle", 1, rAngleCallback);

    ros::Publisher statePub = handler.advertise<navigation::state>("/NUMarine/state/likely", 100);   
    
    // Create desired loop rate
    ros::Rate rate(1/dt);
    int count = 0;

    while(ros::ok()){
        // Grab all current messages
        ros::spinOnce();

        // Initialise state estimate message
        navigation::state mostLikely;       

        // Calculate likelihoods from sensors
        imuLogLiklihood(imuMeas, xp, M, imulw);
        gpsLogLiklihood(gpsMeas, xp, M, gpslw);

        // Combine log weights
        lw = gpslw.array() + imulw.array();
        
        // Normailise log weights
        lseW = lw;
        lw = lw.array() - logSumExponential(lseW);

        // Extract Most Likely Particle
        Eigen::MatrixXd::Index maxIdx;
        lw.maxCoeff(&maxIdx);

        // Resample particles based on weights
        weightedResample(lw,M,idxWeighted);
        xt = xp(Eigen::all,idxWeighted.array());

        // Update particles through process model for next time step
        uMat = u.replicate(1,M);
        processModel(xt, uMat, param,dt, xp);

        // Assign estimated states to appropriate message
        mostLikely.N = xp(0,maxIdx);
        mostLikely.E = xp(1,maxIdx);
        mostLikely.psi = xp(2,maxIdx);
        mostLikely.u = xp(3,maxIdx);
        mostLikely.v = xp(4,maxIdx);
        mostLikely.r = xp(5,maxIdx);

        // Publish states to state estimate topic
        statePub.publish(mostLikely);

        // Sleep to meet desired loop frequency
        bool realTime = rate.sleep();

        // Visual output to show running with indication of realtime or not
        if(count==8){
        std::cout << "\33[2K" << "\r"; //Reset dots
        count=0;
        }
        if(realTime){
            std::cout << "\33[2K" << "Running in realtime";
        }
        else{
            std::cout << "\33[2K" << "Running slower than realtime";
        }
        std::cout << std::string(count/2+1, '.') << "\r" ;
        std::cout.flush();
        count++;
    }
    return 0;
}