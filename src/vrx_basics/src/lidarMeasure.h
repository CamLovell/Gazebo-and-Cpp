#ifndef LIDARMEASURE_H 
#define LIDARMEASURE_H

#include<vector>


class lidarSim {
    public:
    struct lidarScanLine{
    double range, heading, pitch; 
    };
    static const lidarScanLine emptyScan;
    
};
// lidarSim::emptyScan.range
extern std::vector<lidarSim::lidarScanLine> lidarPass;
// [0] = lidarSim::emptyScan;

// std::vector<lidarSim::lidarScanLine> lidarSim::lidarPass;

#endif