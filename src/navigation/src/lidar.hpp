#ifndef LIDAR_H
#define LIDAR_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <fstream>

// Put structs here to make cpp a little cleaner, allows for use in other files if need be
struct Map{
    double x0, y0, dx, dy;
    Eigen::VectorXd x, y;
    Eigen::MatrixXd z;

    int numX, numY;
};

struct Scanner{
    Eigen::VectorXd theta;
    double x0, y0, psi0;
    double startDeg, resDeg, maxRange;
    int numScans;
};

struct Pose{
    double x, y, psi;
};

int linSegInt(const double & x1, const double & x2, const double & y1,const double & y2,const double & x3, const double & x4, const double & y3, const double & y4, double *xint, double *yint);
void findRange(Map & map, Scanner & scanner, Pose & pose, Eigen::VectorXd & range, Eigen::VectorXd & xr, Eigen::VectorXd & yr, Eigen::VectorXd & C);


#endif 
// i love u 