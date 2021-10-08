#include <catch2/catch.hpp>
#include <Eigen/Core>
#include <iostream>
#include "spacialDual.h"

SCENARIO("Converting NED to Latitude and Longitude: Origin") {
    GIVEN("Origin as input"){
        
        Eigen::MatrixXd NED(3,1), geo;
        NED <<  0, 0, 0;
        REQUIRE(NED.rows()==3);
        REQUIRE(NED.cols()==1);

        WHEN("Converting to latitude and longitude"){
            gpsInit();
            NEDtoGeo(NED,geo);
            REQUIRE(geo.rows()==3);
            REQUIRE(geo.cols()==1);
            THEN("Reuslts are correct"){
                CHECK(geo(0) == Approx(-33.724223));
                CHECK(geo(1) == Approx(150.679736));
                // Altitude is Known to be broken right now
                CHECK(geo(2) == Approx(0.0));
            }
        }
    }
}
SCENARIO("Converting NED to Latitude and Longitude: Random") {
    GIVEN("Point away from orign as input"){        
        Eigen::MatrixXd NED(3,1), geo;
        NED <<  531.564687, -162.178285, -0.146976;
        REQUIRE(NED.rows()==3);
        REQUIRE(NED.cols()==1);

        WHEN("Converting to latitude and longitude"){
            gpsInit();
            NEDtoGeo(NED,geo);
            REQUIRE(geo.rows()==3);
            REQUIRE(geo.cols()==1);
            THEN("Reuslts are correct"){
                CHECK(geo(0) == Approx(-33.719431));
                CHECK(geo(1) == Approx(150.677986));
                // Altitude is Known to be broken right now
                CHECK(geo(2) == Approx(1.1405));
                std::cout << (geo(2)-1.1405)/0.146976 << std::endl;
            }
        }
    }
}
