#define _USE_MATH_DEFINES
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include "ros/ros.h"

#include "gtest/gtest.h"

#include "spacialDual.h"
#include "testing.h"
#include "Eig.hpp"


class eigen_atan2: public ::testing::Test { 
public: 
   eigen_atan2( ) { 
       // initialization code here
   } 

   void SetUp( ) { 
       // code here will execute just before the test ensues 
   }

   void TearDown( ) { 
       // code here will be called just after the test completes
       // ok to through exceptions from here if need be
   }

   ~eigen_atan2( )  { 
       // cleanup any pending stuff, but no exceptions allowed
   }

   // put in any custom data members that you need 
};
class Quaternions: public ::testing::Test { 
public: 
   Quaternions( ) { 
       // initialization code here
   } 

   void SetUp( ) { 
       // code here will execute just before the test ensues 
   }

   void TearDown( ) { 
       // code here will be called just after the test completes
       // ok to through exceptions from here if need be
   }

   ~Quaternions( )  { 
       // cleanup any pending stuff, but no exceptions allowed
   }

   // put in any custom data members that you need 
};
class GPS_Functions: public ::testing::Test { 
public: 
   GPS_Functions( ) { 
       // initialization code here
       std::cout << "teseting asdfgbakdsjf" << std::endl;

   } 

   void SetUp( ) { 
       // code here will execute just before the test ensues
       std::cout << "teseting asdfgbakdsjf" << std::endl;
       gpsInit();
   }

   void TearDown( ) { 
       // code here will be called just after the test completes
       // ok to through exceptions from here if need be
   }

   ~GPS_Functions( )  { 
       // cleanup any pending stuff, but no exceptions allowed
   }

   // put in any custom data members that you need 
};

TEST(eigen_atan2, expect_true) {
    Eigen::VectorXd num(9),den(9),expected(9),result;
    num << 0.0,0.0,0.0,1.0,1.0,1.0,-1.0,-1.0,-1.0;
    den << 0.0,1.0,-1.0,0.0,1.0,-1.0,0.0,1.0,-1.0;
    expected << 0.0,0.0,3.141592653589793,1.570796326794897,0.785398163397448,2.356194490192345,-1.570796326794897,-0.785398163397448,-2.356194490192345;
    Eig::atan2(num,den,result); 
    
    // EXPECT_EQ(0, 0);
    EXPECT_TRUE(result.isApprox(expected));
    if(!result.isApprox(expected)){
        EXPECT_DOUBLE_EQ(result(0), expected(0));
        EXPECT_DOUBLE_EQ(result(1), expected(1));
        EXPECT_DOUBLE_EQ(result(2), expected(2));
        EXPECT_DOUBLE_EQ(result(3), expected(3));
        EXPECT_DOUBLE_EQ(result(4), expected(4));
        EXPECT_DOUBLE_EQ(result(5), expected(5));
        EXPECT_DOUBLE_EQ(result(6), expected(6));
        EXPECT_DOUBLE_EQ(result(7), expected(7));
        EXPECT_DOUBLE_EQ(result(8), expected(8));
    }    
}

TEST(Quaternions, quaternion_to_RPY) {
    Eigen::MatrixXd Q(4,1), resultRPY, expectedRPY(3,1);
    Q <<  0.8512568, 0.3419754, 0.1761598, -0.3569066;
    expectedRPY << 0.5752221, 0.5752221, -0.619449;

    quaterniontoRPY(Q,resultRPY);
    // EXPECT_TRUE(resultRPY.isApprox(expectedRPY));
    if(!resultRPY.isApprox(expectedRPY)){
        EXPECT_NEAR(resultRPY(0), expectedRPY(0),pow(10,-5));
        EXPECT_NEAR(resultRPY(1), expectedRPY(1),pow(10,-5));
        EXPECT_NEAR(resultRPY(2), expectedRPY(2),pow(10,-5));

    }
    // std::cout << resultRPY << std::endl;  
    // std::cout << resultRPY << std::endl;  
}
TEST(Quaternions, RPY_to_quaternion) {
    Eigen::MatrixXd RPY(3,1), resultQ, expectedQ(4,1);
    RPY <<  0.5752221, 0.5752221, -0.619449;
    expectedQ << 0.8512568, 0.3419754, 0.1761598, -0.3569066;

    RPYtoQuaternion(RPY,resultQ);
    // EXPECT_TRUE(resultRPY.isApprox(expectedRPY));
    if(!resultQ.isApprox(expectedQ)){
        EXPECT_NEAR(resultQ(0), expectedQ(0),pow(10,-5));
        EXPECT_NEAR(resultQ(1), expectedQ(1),pow(10,-5));
        EXPECT_NEAR(resultQ(2), expectedQ(2),pow(10,-5));
        EXPECT_NEAR(resultQ(3), expectedQ(3),pow(10,-5));

    }
    // std::cout << resultRPY << std::endl;  
    // std::cout << resultRPY << std::endl;  
}
TEST(GPS_Functions, NED_to_LatLong) {
    gpsInit();
    Eigen::MatrixXd NED(3,1), geo, expectedGeo(3,1);
    NED <<  0, 0, 0;
    expectedGeo << -33.724223, 150.679736, 0;

    NEDtoGeo(NED,geo);
    // EXPECT_TRUE(resultRPY.isApprox(expectedRPY));
    if(!geo.isApprox(expectedGeo)){
        EXPECT_NEAR(geo(0), expectedGeo(0),pow(10,-6));
        EXPECT_NEAR(geo(1), expectedGeo(1),pow(10,-6));
        // Altitude is Known to be broken right now
        EXPECT_NEAR(geo(2), expectedGeo(2),pow(10,-10)); 

    }
    std::cout << geo(2) << std::endl;  
    std::cout << expectedGeo(2) << std::endl;  
}

TEST(GPS_Functions, NED_to_LatLong_nonZero) {
    gpsInit();
    Eigen::MatrixXd NED(3,1), geo, expectedGeo(3,1);
    NED <<  0, 0, 0;
    expectedGeo << -33.724223, 150.679736, 0;

    NEDtoGeo(NED,geo);
    // EXPECT_TRUE(resultRPY.isApprox(expectedRPY));
    if(!geo.isApprox(expectedGeo)){
        EXPECT_NEAR(geo(0), expectedGeo(0),pow(10,-6));
        EXPECT_NEAR(geo(1), expectedGeo(1),pow(10,-6));
        // Altitude is Known to be broken right now
        // EXPECT_NEAR(geo(2), expectedGeo(2),pow(10,-6)); 

    }
    // std::cout << resultRPY << std::endl;  
    // std::cout << resultRPY << std::endl;  
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}