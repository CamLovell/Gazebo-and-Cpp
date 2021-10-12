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
    gpsMeas << fix->latitude, fix->longitude, fix->altitude;
    // ROS_INFO("Latitude = %.4f\nLongitude = %.4f\naltitude = %.4f\n", fix->latitude, fix->longitude, fix->altitude);
}
void imuCallBack(const sensor_msgs::Imu::ConstPtr& imuMsg){
    // Eigen::MatrixXd q(4,1), RPY;
    imuMeas << imuMsg->orientation.w, imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z;
    // quaterniontoRPY(q, RPY);
    // ROS_INFO("x = %.4f\ny = %.4f\nz = %.4f\nw = %.4f\n", imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z,imuMsg->orientation.w);
    // ROS_INFO("R = %.4f\nP = %.4f\nY = %.4f\n", (180/M_PI)*RPY(0,0),(180/M_PI)*RPY(1,0),(180/M_PI)*RPY(2,0));
}


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


void nedCallBack(const gazebo_msgs::ModelStates::ConstPtr& nedMsg){
    // Eigen::MatrixXd rBNn(3,1), rBOg;
    // rBNn << nedMsg->pose[11].position.x, nedMsg->pose[11].position.y, nedMsg->pose[11].position.z; 
    // // rBNn.setZero();
    // NEDtoGeo(rBNn, rBOg);
    // ROS_INFO("Lat = %.6f\nLong = %.6f\nAlt = %.6f\n", rBOg(0),rBOg(1),rBOg(2));
    // ROS_INFO("x = %.6f\ny = %.6f\nz = %.6f\n", rBNn(0),rBNn(1),rBNn(2));
    // // ROS_INFO("Lat = %.6f\nLong = %.6f\nAlt = %.6f\n", rBNn(0),rBNn(1),rBNn(2));
}

int main(int argc, char **argv){
    ros::init(argc, argv, "navigation");

    Eigen::VectorXd x0(6),gpslw, imulw, lw, lseW;
    Eigen::MatrixXd xp, xt, uMat;
    Eigen::Matrix<int,Eigen::Dynamic,1> idxWeighted;
    boatParams param;
    int M = 200;
    double dt = 0.25;
    u.setZero();
    // Initialise nessecary components
    intiBoat(param);
    particleInit(M,lw,xp);
    gpsInit();

    ros::NodeHandle handler;
    ros::Subscriber gpsSub = handler.subscribe<sensor_msgs::NavSatFix>("/wamv/sensors/gps/gps/fix", 1, gpsCallBack);
    ros::Subscriber nedSub = handler.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, nedCallBack);
    ros::Subscriber imuSub = handler.subscribe<sensor_msgs::Imu>("/wamv/sensors/imu/imu/data", 1, imuCallBack);

    ros::Subscriber lThrustSub = handler.subscribe<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 1, lThrustCallback);
    ros::Subscriber lAngleSub = handler.subscribe<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 1, lAngleCallback);
    ros::Subscriber rThrustSub = handler.subscribe<std_msgs::Float32>("/wamv/thrusters/left_thrust_angle", 1, rThrustCallback);
    ros::Subscriber rAngleSub = handler.subscribe<std_msgs::Float32>("/wamv/thrusters/right_thrust_angle", 1, rAngleCallback);

    ros::Publisher statePub = handler.advertise<navigation::state>("/NUMarine/state/likely", 100);   
    
    ros::Rate rate(1/dt);

    while(ros::ok()){
        ros::spinOnce();
        navigation::state mostLikely;       
        // std::cout << "test" << xp << std::endl;
        // Calculate likelihoods from sensors
        imuLogLiklihood(imuMeas, xp, M, imulw);
        // std::cout << xp << std::endl;
        gpsLogLiklihood(gpsMeas, xp, M, gpslw);

        lw = gpslw.array()+imulw.array();
        // std::cout << imulw << std::endl;
        // std::cout << gpslw << std::endl;

        // Normailise log weights
        lseW = lw;
        logSumExponential(lseW);
        // std::cout << lseW << std::endl;
        lw = lw.array() - lseW.array();

        Eigen::MatrixXd::Index maxIdx;
        lw.maxCoeff(&maxIdx);
        
        // Resample particles based on weights
        weightedResample(lw,M,idxWeighted);
        xt = xp(Eigen::all,idxWeighted.array());

        // Update particles through process model for next time step
        // std::cout << u << std::endl;
        uMat = u.replicate(1,M);
        processModel(xt, uMat, param,dt, xp);

        mostLikely.N = xp(0,maxIdx);
        mostLikely.E = xp(1,maxIdx);
        mostLikely.psi = xp(2,maxIdx);
        mostLikely.u = xp(3,maxIdx);
        mostLikely.v = xp(4,maxIdx);
        mostLikely.r = xp(5,maxIdx);

        statePub.publish(mostLikely);
        rate.sleep();
    }
    return 0;
}