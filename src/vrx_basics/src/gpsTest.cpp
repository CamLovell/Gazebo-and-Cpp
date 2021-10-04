#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"

void gpsCallBack(const sensor_msgs::NavSatFix::ConstPtr& fix){
    ROS_INFO("Latitude = %.4f\nLongitude = %.4f\naltitude = %.4f\n", fix->latitude, fix->longitude, fix->altitude);
}
void imuCallBack(const sensor_msgs::Imu::ConstPtr& imuMsg){
    ROS_INFO("x = %.4f\ny = %.4f\nz = %.4f\nw = %.4f\n", imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z,imuMsg->orientation.w);
}


int main(int argc, char **argv){
    
    ros::init(argc, argv, "gpsReader");
    ros::NodeHandle gpsHandler;
    ros::Subscriber gpsSub = gpsHandler.subscribe<sensor_msgs::NavSatFix>("/wamv/sensors/gps/gps/fix", 1, gpsCallBack);
    ros::NodeHandle imuHandler;
    ros::Subscriber imuSub = imuHandler.subscribe<sensor_msgs::Imu>("/wamv/sensors/imu/imu/data", 1, imuCallBack);
    
    
    ros::Rate rate(1);
    while(ros::ok()){
        ros::spinOnce();
        // Do stuff and do it fast!
        rate.sleep();
    }
    return 0;
}