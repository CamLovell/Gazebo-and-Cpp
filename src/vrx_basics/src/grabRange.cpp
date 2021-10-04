#include <ros/ros.h>
#include "lidarMeasure.h"
#include "extractRange.h"

lidarSim current;

int main(void){
    
    std::cout << "Grabbed range = "  << lidarPass[0].range << std::endl;
}

void setLidar(void){
    // current.lidarPass = nextPass;
}