#include <ros/ros.h>

// Uses Point Cloud Library
// Follow install instructions at: https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html
#include <cmath>
#include <iostream>
#include <Eigen/Core>
#include <pcl_ros/point_cloud.h> 
#include <pcl/point_types.h>
#include<sensor_msgs/PointCloud2.h>


#include <boost/foreach.hpp>
#include "lidarMeasure.h"
#include "extractRange.h"
#include "grabRange.h"

// Define simpler reference to point cloud message type
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

static int i = 0, count = 0;

// Vector of lidar scan lines and their constituent properties
std::vector<lidarSim::lidarScanLine> lidarPass[0] = lidarSim::emptyScan;

void callback(const PointCloud::ConstPtr& msg)
{
  
    // Extract Components from point cloud
    if(!i){
        printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
        // Loop through each detected point
        BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
            // Calculate range and angles for each point
            lidarSim::lidarScanLine temp;
            temp.range = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
            temp.heading = atan2(pt.y,pt.x)*180/M_PI;
            temp.pitch = atan2(pt.z,temp.range)*180/M_PI;
            // printf ("\t%f:(%f, %f, %f)\n", range,pt.x, pt.y, pt.z);
            lidarPass.push_back(temp);
        }
        // Print components
        for(int i=0;i<lidarPass.size();i++){
            std::cout << "Range = " << lidarPass[i].range << " Heading = " << lidarPass[i].heading << " Pitch = " << lidarPass[i].pitch << std::endl;
        }
        
        // increment counters and checks
        count = 0;
        i++;
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("/wamv/sensors/lidars/lidar_wamv/points", 1, callback);
  ros::spin();
}

void getRange(double& Pass){
    Pass = 23.6;
}