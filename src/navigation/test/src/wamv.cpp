#define _USE_MATH_DEFINES
#include <cmath>
#include <Eigen/Core>

#include <iostream>
#include <catch2/catch.hpp>

#include "wamv.h"

// Testing boat dynamics output
SCENARIO("Single vector inputs into dynamics") {
    GIVEN("Zero initial states and straight thrusters"){
        Eigen::MatrixXd x(6,1),u(4,1),dx;
        x << 0, 0, 0, 0, 0, 0;
        u << 2000, 0, 2000, 0;

        boatParams param;
        intiBoat(param);

        WHEN("Calculating dx from dynamics"){
            
            boatDynamics(x, u, param, dx);
            REQUIRE(dx.size()==6);

            THEN("Reuslts are correct"){
                CHECK(dx(0) == Approx(0));
                CHECK(dx(1) == Approx(0));
                CHECK(dx(2) == Approx(0));
                CHECK(dx(3) == Approx(4.627540809124000));
                CHECK(dx(4) == Approx(0));
                CHECK(dx(5) == Approx(0));
            }
        }
    }

    GIVEN("Non-Zero initial states and Straight thrusters"){
        Eigen::MatrixXd x(6,1),u(4,1),dx;
        x << 22, 12, 1.2, 3, 4, 0.1;
        u << 1500, 0, 1500, 0;

        boatParams param;
        intiBoat(param);

        WHEN("Calculating dx from dynamics"){
            
            boatDynamics(x, u, param, dx);
            REQUIRE(dx.size()==6);

            THEN("Reuslts are correct"){
                CHECK(dx(0) == Approx(-2.641083080438884));
                CHECK(dx(1) == Approx(4.245548275808373));
                CHECK(dx(2) == Approx(0.1));
                CHECK(dx(3) == Approx(1.891872881453356));
                CHECK(dx(4) == Approx(-5.970002531572293));
                CHECK(dx(5) == Approx(5.806852136648306));
            }
        }
    }

    GIVEN("Non-Zero initial states and Varied thrusters"){
        Eigen::MatrixXd x(6,1),u(4,1),dx;
        x << 31, 4, 1.6, 2.5, 3.7, 0.3;
        u << 1300, -0.9, 1800, 0.7;

        boatParams param;
        intiBoat(param);

        WHEN("Calculating dx from dynamics"){
            
            boatDynamics(x, u, param, dx);
            REQUIRE(dx.size()==6);

            THEN("Reuslts are correct"){
                CHECK(dx(0) == Approx(-3.771421137006791));
                CHECK(dx(1) == Approx(2.390895775088994));
                CHECK(dx(2) == Approx(0.3));
                CHECK(dx(3) == Approx(0.921461141225237));
                CHECK(dx(4) == Approx(-10.219774807771469));
                CHECK(dx(5) == Approx(4.883606835081329));
            }
        }
    }
}

// Testing boat dynamics output for multiple particles simultaneously
SCENARIO("Matrix input for multiple particles") {
    GIVEN("All single vector cases combined into one input"){
        Eigen::MatrixXd x(6,3),u(4,3),dx;

        x.col(0) << 0, 0, 0, 0, 0, 0;
        x.col(1) << 22, 12, 1.2, 3, 4, 0.1;
        x.col(2) <<  31, 4, 1.6, 2.5, 3.7, 0.3;

        u.col(0) << 2000, 0, 2000, 0;
        u.col(1) << 1500, 0, 1500, 0;
        u.col(2) << 1300, -0.9, 1800, 0.7;

        boatParams param;
        intiBoat(param);

        WHEN("Calculating dx from dynamics"){
            
            boatDynamics(x, u, param, dx);
            REQUIRE(dx.rows()==6);
            REQUIRE(dx.cols()==3);

            THEN("Reuslts are correct"){
                // First "particle"
                CHECK(dx(0,0) == Approx(0));
                CHECK(dx(1,0) == Approx(0));
                CHECK(dx(2,0) == Approx(0));
                CHECK(dx(3,0) == Approx(4.627540809124000));
                CHECK(dx(4,0) == Approx(0));
                CHECK(dx(5,0) == Approx(0));
                // Second "particle"
                CHECK(dx(0,1) == Approx(-2.641083080438884));
                CHECK(dx(1,1) == Approx(4.245548275808373));
                CHECK(dx(2,1) == Approx(0.1));
                CHECK(dx(3,1) == Approx(1.891872881453356));
                CHECK(dx(4,1) == Approx(-5.970002531572293));
                CHECK(dx(5,1) == Approx(5.806852136648306));
                // Third "particle"
                CHECK(dx(0,2) == Approx(-3.771421137006791));
                CHECK(dx(1,2) == Approx(2.390895775088994));
                CHECK(dx(2,2) == Approx(0.3));
                CHECK(dx(3,2) == Approx(0.921461141225237));
                CHECK(dx(4,2) == Approx(-10.219774807771469));
                CHECK(dx(5,2) == Approx(4.883606835081329));
            }
        }
    }
}