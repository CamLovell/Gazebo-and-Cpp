#include "ros/ros.h"
#include "std_msgs/Float32.h"

#define _USE_MATH_DEFINES
#include<cmath>
#include<sstream>

const int freq = 100;

int main(int argc, char **argv){
    // Initialise ROS node and handle
    ros::init(argc,argv,"sin_drive");

    ros::NodeHandle n;

    ros::Publisher L_thrustpub = n.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 1000);
    ros::Publisher R_thrustpub = n.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 1000);
    ros::Publisher L_anglepub = n.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_angle", 1000);
    ros::Publisher R_anglepub = n.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_angle", 1000);

    ros::Rate loop(freq);

    int stepCount = 0;
    while(ros::ok()){
        std_msgs::Float32 thrustL, thrustR, angleL, angleR;

        thrustL.data = std::abs(1*cos(stepCount/(freq*20.0)));
        thrustR.data = std::abs(1*cos(stepCount/(freq*20.0)));
        angleL.data = 0*M_PI/4*(sin(stepCount/(freq*200.0)));
        angleR.data = 0*M_PI/4*(sin(stepCount/(freq*200.0)));

        // thrustL.data = 1;
        // thrustR.data = 1;
        // angleL.data = 0;
        // angleR.data = 0;
        
        // ROS_INFO("Left Thrust Sent = %f", thrustL.data);
        // ROS_INFO("Right Thrust Sent = %f", thrustR.data);
        // ROS_INFO("Left Angle Sent = %f", angleL.data);
        // ROS_INFO("Right Angle Sent = %f", angleR.data);

        L_thrustpub.publish(thrustL);
        R_thrustpub.publish(thrustR);
        L_anglepub.publish(angleL);
        R_anglepub.publish(angleR);
        
        ros::spinOnce();
        loop.sleep();
        stepCount++;

    }
    return 0;
}