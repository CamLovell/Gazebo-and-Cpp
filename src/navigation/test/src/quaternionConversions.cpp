#include <catch2/catch.hpp>
#include <Eigen/Core>
#include "spacialDual.h"

SCENARIO("Quaternion measurment to roll pitch yaw") {
    GIVEN("A quaternion measument of rotation"){
        Eigen::MatrixXd Q(4,1), resultRPY;
        Q <<  0.8512568, 0.3419754, 0.1761598, -0.3569066;
        // expectedRPY << 0.5752221, 0.5752221, -0.619449;
        REQUIRE(Q.rows()==4);
        REQUIRE(Q.cols()==1);
        WHEN("Converting to RPY"){
            quaterniontoRPY(Q,resultRPY);
            REQUIRE(resultRPY.rows()==3);
            REQUIRE(resultRPY.cols()==1);
            // EXPECT_TRUE(resultRPY.isApprox(expectedRPY));
            // if(!resultRPY.isApprox(expectedRPY)){
            THEN("Results are correct"){
            CHECK(resultRPY(0) == Approx(0.5752221).margin(1e-5));
            CHECK(resultRPY(1) == Approx(0.5752221).margin(1e-5));
            CHECK(resultRPY(2) == Approx(-0.619449).margin(1e-5));
            }
        }
    }
}

SCENARIO("Roll pitch yaw measurment to quaternion") {
    GIVEN("An RPY measument of rotation"){
        Eigen::MatrixXd RPY(3,1), resultQ;
        RPY <<  0.5752221, 0.5752221, -0.619449;
        REQUIRE(RPY.rows()==3);
        REQUIRE(RPY.cols()==1);
        WHEN("Converting to quaternion"){
            RPYtoQuaternion(RPY,resultQ);

            REQUIRE(resultQ.rows()==4);
            REQUIRE(resultQ.cols()==1);

            THEN("Results are correct"){
                CHECK(resultQ(0) == Approx(0.8512568).margin(1e-5));
                CHECK(resultQ(1) == Approx(0.3419754).margin(1e-5));
                CHECK(resultQ(2) == Approx(0.1761598).margin(1e-5));
                CHECK(resultQ(3) == Approx(-0.3569066).margin(1e-5));
            }
        }
    } 
}