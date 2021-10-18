#ifndef MAP_H
#define MAP_H

struct Map{
    double E0, N0, dE, dN;
    Eigen::VectorXd E, N;
    Eigen::MatrixXd z, logOdds;
    int numX, numY;
    double passChange = 0.02, hitChange = 0.04, hitDepth = 0.5;
};

#endif