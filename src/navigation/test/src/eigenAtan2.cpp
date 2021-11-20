
#define _USE_MATH_DEFINES
#include <cmath>
#include <Eigen/Core>

#include <iostream>
#include <catch2/catch.hpp>

#include "Eig.hpp"

// Testing Eigen Atan2 function
SCENARIO("Testing atan2: axis inputs") {
    GIVEN("Equal vector input and output lengths"){
        Eigen::VectorXd num(9),den(9),expected(9),result, testing(3);
        num << 0.0,0.0,0.0,1.0,1.0,1.0,-1.0,-1.0,-1.0;
        den << 0.0,1.0,-1.0,0.0,1.0,-1.0,0.0,1.0,-1.0;
        
        REQUIRE(num.size()==den.size());
        WHEN("Calculating atan2 elementwise"){
            Eig::atan2(num,den,result);
            REQUIRE(result.size()==num.size());
            THEN("Output is correct"){   
                CHECK(result(0) == Approx(0.0));
                CHECK(result(1) == Approx(0.0));
                CHECK(result(2) == Approx(M_PI));
                CHECK(result(3) == Approx(M_PI/2));
                CHECK(result(4) == Approx(M_PI/4));
                CHECK(result(5) == Approx(3*M_PI/4));
                CHECK(result(6) == Approx(-M_PI/2));
                CHECK(result(7) == Approx(-M_PI/4));
                CHECK(result(8) == Approx(-3*M_PI/4));
            }
        }
    }   
}
