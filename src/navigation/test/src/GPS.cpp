#include <catch2/catch.hpp>
#include <Eigen/Core>
#include <iostream>
#include "spacialDual.h"
#include "particleFilter.h"

SCENARIO("Converting NED to Latitude and Longitude") {
    GIVEN("Origin as input"){
        Eigen::VectorXd logWeights;
        Eigen::MatrixXd particleStates;
        
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
    GIVEN("Point away from orign as input"){        
        Eigen::MatrixXd NED(3,1), geo;
        NED << 531.564687, -162.178285, -0.146976;
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
                // std::cout << geo << std::endl;
            }
        }
    }
}
SCENARIO("Converting NED to Latitude and Longitude: Multiple inputs") {
    GIVEN("Origin and other point as input"){
        Eigen::VectorXd logWeights;
        Eigen::MatrixXd particleStates;
        
        Eigen::MatrixXd NED(3,3), geo;
        NED.col(0) <<  0, 0, 0;
        NED.col(1) <<  531.564687, -162.178285, -0.146976;
        NED.col(2) <<  531.564687, -162.178285, -0.146976;

        REQUIRE(NED.rows()==3);
        REQUIRE(NED.cols()==3);

        WHEN("Converting to latitude and longitude"){
            gpsInit();
            NEDtoGeo(NED,geo);
            REQUIRE(geo.rows()==3);
            REQUIRE(geo.cols()==3);
            THEN("Reuslts are correct"){
                CHECK(geo(0,0) == Approx(-33.724223));
                CHECK(geo(1,0) == Approx(150.679736));
                CHECK(geo(0,1) == Approx(-33.719431));
                CHECK(geo(1,1) == Approx(150.677986));
                // Altitude is Known to be broken right now
                // std::cout << geo << std::endl;

                CHECK(geo(2,0) == Approx(0.0));
                CHECK(geo(2,1) == Approx(1.1405));
            }
        }
    }
}
SCENARIO("Converting Latitude and Longitude to NED") {
    GIVEN("Origin as input"){
        
        Eigen::MatrixXd NED, geo(3,1);
        geo <<  -33.724223, 150.679736, 0.0;
        REQUIRE(geo.rows()==3);
        REQUIRE(geo.cols()==1);

        WHEN("Converting to latitude and longitude"){
            gpsInit();
            geotoNED(geo,NED);
            REQUIRE(NED.rows()==3);
            REQUIRE(NED.cols()==1);
            THEN("Reuslts are correct"){
                CHECK(NED(0) == Approx(0.0));
                CHECK(NED(1) == Approx(0.0));
                // Altitude is Known to be broken right now
                CHECK(NED(2) == Approx(0.0));
            }
        }
    }
    // GIVEN("Point away from orign as input"){        
        
    //     Eigen::MatrixXd NED, geo(3,1);
    //     geo <<  -33.7228, 150.6740, 1.2287;
    //     REQUIRE(geo.rows()==3);
    //     REQUIRE(geo.cols()==1);

    //     WHEN("Converting to latitude and longitude"){
    //         gpsInit();
    //         geotoNED(geo,NED);
    //         REQUIRE(NED.rows()==3);
    //         REQUIRE(NED.cols()==1);
    //         THEN("Reuslts are correct"){
    //             CHECK(NED(0) == Approx(531.564687).margin(0.1));
    //             CHECK(NED(1) == Approx(-162.178285).margin(0.1));
    //             // Altitude is Known to be broken right now
    //             // CHECK(NED(2) == Approx(-0.146976).margin(0.1));
    //         }
    //     }
    // }
}