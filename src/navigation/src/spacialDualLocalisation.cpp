#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <Eigen/Core>

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "gazebo_msgs/ModelStates.h"
#include "spacialDual.h"

void gpsCallBack(const sensor_msgs::NavSatFix::ConstPtr& fix){
    ROS_INFO("Latitude = %.4f\nLongitude = %.4f\naltitude = %.4f\n", fix->latitude, fix->longitude, fix->altitude);
}
void imuCallBack(const sensor_msgs::Imu::ConstPtr& imuMsg){
    Eigen::MatrixXd q(4,1), RPY;
    q << imuMsg->orientation.w, imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z;
    quaterniontoRPY(q, RPY);
    // ROS_INFO("x = %.4f\ny = %.4f\nz = %.4f\nw = %.4f\n", imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z,imuMsg->orientation.w);
    ROS_INFO("R = %.4f\nP = %.4f\nY = %.4f\n", (180/M_PI)*RPY(0,0),(180/M_PI)*RPY(1,0),(180/M_PI)*RPY(2,0));
}
void nedCallBack(const gazebo_msgs::ModelStates::ConstPtr& nedMsg){
    Eigen::MatrixXd rBNn(3,1), rBOg;
    rBNn << nedMsg->pose[11].position.x, nedMsg->pose[11].position.y, nedMsg->pose[11].position.z; 
    // rBNn.setZero();
    NEDtoGeo(rBNn, rBOg);
    ROS_INFO("Lat = %.6f\nLong = %.6f\nAlt = %.6f\n", rBOg(0),rBOg(1),rBOg(2));
    // ROS_INFO("Lat = %.6f\nLong = %.6f\nAlt = %.6f\n", rBNn(0),rBNn(1),rBNn(2));
}

int main(int argc, char **argv){
    ros::init(argc, argv, "gpsReader");
    gpsInit();

    ros::NodeHandle gpsHandler;
    ros::Subscriber gpsSub = gpsHandler.subscribe<sensor_msgs::NavSatFix>("/wamv/sensors/gps/gps/fix", 1, gpsCallBack);
    ros::NodeHandle nedHandler;
    ros::Subscriber nedSub = nedHandler.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, nedCallBack);
    ros::NodeHandle imuHandler;
    ros::Subscriber imuSub = imuHandler.subscribe<sensor_msgs::Imu>("/wamv/sensors/imu/imu/data", 1, imuCallBack);
    
    
    ros::Rate rate(5);
    while(ros::ok()){
        ros::spinOnce();
        // Do stuff and do it fast!
        rate.sleep();
    }
    return 0;
}