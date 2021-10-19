#define _USE_MATH_DEFINES
#include <cmath>
#include <Eigen/Core>

#include <iostream>
#include <catch2/catch.hpp>

#include "map.h"
#include "lidar.h"
#include "Eig.hpp"

SCENARIO("Update map with known pose and map") {
    GIVEN("A lidar measurment from a known map"){
        Map map;

        // Intialise map paramaters
        map.E.resize(400);
        map.N.resize(400);        
        map.dE = 0.5;
        map.dN = map.dE;
        map.E0 = 0;
        map.N0 = map.E0;
        map.E.setLinSpaced(400,map.E0,(400-1)*map.dE);
        map.N.setLinSpaced(400,map.N0,(400-1)*map.dN);
        // std::cout << map.N << std::endl;
        map.numX = 400;
        map.numY = map.numX;

        std::string path = "navigation/data/testMap.csv";
        map.z = readCSV(path,400,400); 
        map.logOdds.setZero(map.z.rows(),map.z.cols());

        Lidar M8;
        lidatInit(M8);
        

        Eigen::VectorXd pose(3);
        pose << 100, 100, 90;

        Eigen::VectorXd range, xr, yr,C;

        findRange(map, M8, pose, range, xr, yr, C);
        mapUpdate(range, pose, M8, map);

        Eigen::MatrixXd ecpectedLog, expectedZ;
        std::string path1 = "navigation/data/logodds.csv";
        ecpectedLog = readCSV(path1,400,400);
        // std::cout << ecpectedLog.block(190,200,10,10) << std::endl;
        // std::cout << map.z.block(190,200,10,10) << std::endl;
        std::string path2 = "navigation/data/occupancy.csv";
        expectedZ = readCSV(path2,400,400);
        
        CHECK(map.z.isApprox(expectedZ,1e-1));
        CHECK(map.logOdds.isApprox(ecpectedLog,1e-2));

    }
}