#ifndef LIDAR_H
#define LIDAR_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <fstream>

#include "map.h"

// Put structs here to make cpp a little cleaner, allows for use in other files if need be
struct Lidar{
    Eigen::VectorXd theta;
    double x0, y0, psi0,
        startDeg, resDeg,
        maxRange, minRange,
        sigma2, lambda,
        lwHit, lwShort, lwRand;
    int numScans;
};

int linSegInt(const double & x1, const double & x2, const double & y1,const double & y2,const double & x3, const double & x4, const double & y3, const double & y4, double *xint, double *yint);
void findRange(const Map & map,const Lidar & lidarM8,const Eigen::VectorXd& pose, Eigen::VectorXd & range, Eigen::VectorXd & xr, Eigen::VectorXd & yr, Eigen::VectorXd & C);
void lidatInit(Lidar& lidarParams);


#endif 
// i love u 