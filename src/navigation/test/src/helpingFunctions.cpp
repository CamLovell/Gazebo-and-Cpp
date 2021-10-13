#include <catch2/catch.hpp>
#include <Eigen/Core>
#include <iostream>
#include <fstream>
#include "helpers.h"

SCENARIO("Testing weighted resample") {
    GIVEN("Equal wieghted input"){
        Eigen::VectorXd w(10), lw;
        Eigen::Matrix<int,Eigen::Dynamic,1> idx;
        int N=10;
        w.fill(0.1);
        lw = w.array().log();

        // w(0)=0.1;
        WHEN("Calling weighted resample"){
            weightedResample(lw,N,idx);
            REQUIRE(idx.rows() == N);
            REQUIRE(idx.cols() == 1);

            THEN("Correct indicies deteremined"){
                CHECK(idx(0) == 0);
                CHECK(idx(1) == 1);
                CHECK(idx(2) == 2);
                CHECK(idx(3) == 3);
                CHECK(idx(4) == 4);
                CHECK(idx(5) == 5);
                CHECK(idx(6) == 6);
                CHECK(idx(7) == 7);
                CHECK(idx(8) == 8);
                CHECK(idx(9) == 9);

            }
        }
    // std::cout << idx << std::endl;
    }
    
    GIVEN("Varied wieghted input"){
        Eigen::VectorXd w(10), lw;
        Eigen::Matrix<int,Eigen::Dynamic,1> idx;
        int N=10;
        w << 0.5,0.4,0.1/9,0.1/9,0.1/9,0.1/9,0.1/9,0.1/9,0.1/9,0.1/9;
        lw = w.array().log();

        // w(0)=0.1;
        WHEN("Calling weighted resample"){
            weightedResample(lw,N,idx);
            REQUIRE(idx.rows() == N);
            REQUIRE(idx.cols() == 1);

            THEN("Correct indicies deteremined"){
                CHECK(idx(0) == 0);
                CHECK(idx(1) == 0);
                CHECK(idx(2) == 0);
                CHECK(idx(3) == 0);
                CHECK(idx(4) == 0);
                CHECK(idx(5) == 1);
                CHECK(idx(6) == 1);
                CHECK(idx(7) == 1);
                CHECK(idx(8) == 1);
                CHECK((idx(9) != 0) & (idx(9) != 1)); // Due to random nature, could be any other number
            }
        }
    // std::cout << idx << std::endl;
    }
}

SCENARIO("Normalising with Log sum exponential"){
    GIVEN("A normalised input"){
        Eigen::VectorXd w(10), lw(10);
        w << 0.1,0.05,0.025,0.15,0.075,0.17,0.03,0.28,0.04,0.08;
        lw = w.array().log();

        WHEN("Calculating LSE"){
            double out = logSumExponential(lw);
            THEN("Correct output"){
                CHECK(out == Approx(0.0).margin(1e-15));// Failed with just "Approx(0.0)" giving error of 2e-16??
            }
        }
    }
    GIVEN("A not normalised input input"){
        Eigen::VectorXd w(10), lw(10);
        w << 0.5,0.08,0.25,0.9,0.07,0.167,0.0323,0.27238,0.634,0.2366;
        lw = w.array().log();

        WHEN("Calculating LSE"){
            double out = logSumExponential(lw);
            THEN("Correct output"){
                CHECK(out == Approx(1.144948651076145)); // Calculation done with LSE in matlab
            }
        }
    }
}